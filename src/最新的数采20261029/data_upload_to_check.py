#!/usr/bin/env python3
"""
项目          修改人     修改日期       改动
无人挖掘机    wuql17    2026/07/22    选择特定文件夹数据上传NAS进行质检   python3 data_upload_to_check.py /XXXXXXX/excavator_data_20260707/20260707082204
无人挖掘机    wuql17    2026/07/23    修复上传时多个任务并发执行报错504





"""


"""Upload one G500 acquisition session to DS923_app.py without deleting it."""

import argparse
import hashlib
import json
import os
import re
import sys
import tarfile
import tempfile
import time
from collections import namedtuple
from urllib.parse import urlparse

import requests


DEFAULT_NAS_URL = "http://192.168.1.240:8082"
DEFAULT_CHUNK_SIZE = 8 * 1024 * 1024
DEFAULT_CONNECT_TIMEOUT = 5.0
DEFAULT_READ_TIMEOUT = 300.0
DEFAULT_RETRIES = 3
DEFAULT_COMPLETE_WAIT_TIMEOUT = 1800.0
DEFAULT_STATUS_INTERVAL = 5.0
COPY_BUFFER_SIZE = 1024 * 1024

DATASET_RE = re.compile(r"^excavator_data_(\d{8})$")
SESSION_RE = re.compile(r"^\d{14}$")
ALLOWED_TOP_LEVELS = frozenset({"bag", "camera_data", "grayPicture", "rangePicture"})

CandidateFile = namedtuple("CandidateFile", "absolute_path relative_path size mtime_ns")


class UploadError(RuntimeError):
    """An error that should be reported to the command-line user."""


def positive_int(value):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("必须是整数")
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def non_negative_int(value):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("必须是整数")
    if parsed < 0:
        raise argparse.ArgumentTypeError("不能小于 0")
    return parsed


def positive_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("必须是数字")
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def posix_path(path):
    return path.replace(os.sep, "/")


def validate_base_url(value):
    value = (value or "").strip().rstrip("/")
    parsed = urlparse(value)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise UploadError("NAS 地址必须是有效的 http/https URL: {}".format(value))
    if parsed.path not in ("", "/") or parsed.params or parsed.query or parsed.fragment:
        raise UploadError("NAS 地址只能包含协议、主机和端口，不能包含接口路径: {}".format(value))
    return value


def _ensure_plain_directory(path, description):
    if os.path.islink(path):
        raise UploadError("{}不能是符号链接: {}".format(description, path))
    if not os.path.isdir(path):
        raise UploadError("{}不存在或不是目录: {}".format(description, path))


def collect_session_files(session_dir):
    """Validate a session and return dataset metadata plus files by archive kind."""
    session_dir = os.path.abspath(os.path.expanduser(session_dir))
    _ensure_plain_directory(session_dir, "会话目录")

    session_name = os.path.basename(session_dir)
    if not SESSION_RE.fullmatch(session_name):
        raise UploadError("会话目录名必须是 14 位时间格式 YYYYMMDDhhmmss: {}".format(session_name))

    dataset_dir = os.path.dirname(session_dir)
    _ensure_plain_directory(dataset_dir, "日期数据目录")
    dataset_name = os.path.basename(dataset_dir)
    dataset_match = DATASET_RE.fullmatch(dataset_name)
    if dataset_match is None:
        raise UploadError("父目录名必须符合 excavator_data_YYYYMMDD: {}".format(dataset_name))
    if session_name[:8] != dataset_match.group(1):
        raise UploadError(
            "会话日期 {} 与父目录日期 {} 不一致".format(session_name[:8], dataset_match.group(1))
        )

    top_entries = sorted(os.listdir(session_dir))
    for name in top_entries:
        path = os.path.join(session_dir, name)
        if name not in ALLOWED_TOP_LEVELS:
            raise UploadError("会话目录包含接收端不允许的顶层项目: {}".format(path))
        _ensure_plain_directory(path, "数据子目录")

    grouped = {"camera": [], "state": []}
    for current_root, dirnames, filenames in os.walk(session_dir, topdown=True, followlinks=False):
        dirnames.sort()
        filenames.sort()

        for dirname in dirnames:
            directory_path = os.path.join(current_root, dirname)
            if os.path.islink(directory_path):
                raise UploadError("不支持目录符号链接: {}".format(directory_path))

        for filename in filenames:
            absolute_path = os.path.join(current_root, filename)
            if os.path.islink(absolute_path):
                raise UploadError("不支持文件符号链接: {}".format(absolute_path))
            if not os.path.isfile(absolute_path):
                raise UploadError("不支持非普通文件: {}".format(absolute_path))

            relative_to_session = os.path.relpath(absolute_path, session_dir)
            components = relative_to_session.split(os.sep)
            if any("\\" in component for component in components):
                raise UploadError("文件名不能包含反斜杠: {}".format(absolute_path))
            top_level = components[0]
            archive_kind = "camera" if top_level == "camera_data" else "state"
            relative_path = posix_path(os.path.join(session_name, relative_to_session))
            stat_result = os.stat(absolute_path)
            grouped[archive_kind].append(
                CandidateFile(
                    absolute_path=absolute_path,
                    relative_path=relative_path,
                    size=stat_result.st_size,
                    mtime_ns=stat_result.st_mtime_ns,
                )
            )

    for files in grouped.values():
        files.sort(key=lambda item: item.relative_path)
    total_files = sum(len(files) for files in grouped.values())
    if total_files == 0:
        raise UploadError("会话目录中没有可上传的普通文件: {}".format(session_dir))

    return {
        "session_dir": session_dir,
        "session_name": session_name,
        "dataset_dir": dataset_dir,
        "dataset_name": dataset_name,
        "grouped_files": grouped,
        "total_files": total_files,
    }


def _assert_source_unchanged(candidate):
    if os.path.islink(candidate.absolute_path) or not os.path.isfile(candidate.absolute_path):
        raise UploadError("创建归档时源文件已消失或类型发生变化: {}".format(candidate.absolute_path))
    current = os.stat(candidate.absolute_path)
    if current.st_size != candidate.size or current.st_mtime_ns != candidate.mtime_ns:
        raise UploadError("创建归档时源文件发生变化，请稍后重试: {}".format(candidate.absolute_path))


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as file_handle:
        while True:
            block = file_handle.read(COPY_BUFFER_SIZE)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def create_archive(session_info, archive_kind, candidates, temp_dir):
    """Create a deterministic-path TAR and return all metadata needed by the API."""
    prefix = "nas_check_{}_{}_".format(archive_kind, session_info["session_name"])
    file_descriptor, archive_path = tempfile.mkstemp(prefix=prefix, suffix=".tar", dir=temp_dir)
    os.close(file_descriptor)
    try:
        with tarfile.open(archive_path, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for candidate in candidates:
                _assert_source_unchanged(candidate)
                archive.add(candidate.absolute_path, arcname=candidate.relative_path, recursive=False)

        for candidate in candidates:
            _assert_source_unchanged(candidate)

        archive_digest = sha256_file(archive_path)
        upload_id = "check_{}_{}_{}_{}".format(
            archive_kind,
            session_info["dataset_name"],
            session_info["session_name"],
            archive_digest[:24],
        )
        archive_size = os.path.getsize(archive_path)
        return {
            "archive_kind": archive_kind,
            "archive_path": archive_path,
            "archive_name": upload_id + ".tar",
            "archive_size": archive_size,
            "upload_id": upload_id,
            "source_files": [candidate.relative_path for candidate in candidates],
        }
    except Exception:
        try:
            os.remove(archive_path)
        except OSError:
            pass
        raise


def response_payload(response, operation):
    try:
        payload = response.json()
    except ValueError:
        text = (response.text or "").strip()
        if len(text) > 500:
            text = text[:500] + "..."
        raise UploadError("{} 返回的不是 JSON（HTTP {}）: {}".format(operation, response.status_code, text))
    if not isinstance(payload, dict):
        raise UploadError("{} 返回的 JSON 不是对象".format(operation))
    return payload


class NasUploader(object):
    def __init__(
        self,
        nas_url,
        chunk_size,
        connect_timeout,
        read_timeout,
        retries,
        complete_wait_timeout,
        status_interval,
    ):
        self.nas_url = validate_base_url(nas_url)
        self.chunk_size = chunk_size
        self.timeout = (connect_timeout, read_timeout)
        self.retries = retries
        self.complete_wait_timeout = complete_wait_timeout
        self.status_interval = status_interval
        self.session = requests.Session()
        # The NAS is a LAN endpoint.  Do not accidentally route it through an
        # HTTP(S)_PROXY configured on the controller.
        self.session.trust_env = False

    def close(self):
        self.session.close()

    def _request(self, method, endpoint, operation, **kwargs):
        url = self.nas_url + endpoint
        last_exception = None
        for attempt in range(self.retries + 1):
            try:
                response = self.session.request(method, url, timeout=self.timeout, **kwargs)
            except requests.RequestException as exc:
                last_exception = exc
                if attempt >= self.retries:
                    raise UploadError("{} 请求失败: {}".format(operation, exc))
            else:
                if 200 <= response.status_code < 300:
                    payload = response_payload(response, operation)
                    if payload.get("ok") is not True:
                        raise UploadError("{} 未返回 ok=true: {}".format(operation, payload))
                    return payload

                retryable = response.status_code in (408, 429) or response.status_code >= 500
                try:
                    payload = response_payload(response, operation)
                    message = payload.get("error") or json.dumps(payload, ensure_ascii=False)
                except UploadError as exc:
                    if not retryable or attempt >= self.retries:
                        raise
                    last_exception = exc
                    payload = None
                    message = str(exc)
                if not retryable or attempt >= self.retries:
                    raise UploadError("{} 失败（HTTP {}）: {}".format(operation, response.status_code, message))
                if payload is not None:
                    last_exception = UploadError(
                        "{} 暂时失败（HTTP {}）: {}".format(operation, response.status_code, message)
                    )

            wait_seconds = min(2 ** attempt, 8)
            print(
                "[WARN] {}，{} 秒后重试（{}/{}）: {}".format(
                    operation, wait_seconds, attempt + 1, self.retries, last_exception
                ),
                file=sys.stderr,
            )
            time.sleep(wait_seconds)

        raise UploadError("{} 请求失败".format(operation))

    def _wait_for_completion(self, upload_id, total_chunks, reason):
        deadline = time.monotonic() + self.complete_wait_timeout
        print(
            "[WARN] upload_complete 结果暂未确认（{}）；"
            "停止重复提交，改为查询 NAS 任务状态，最多等待 {:.0f} 秒".format(
                reason, self.complete_wait_timeout
            ),
            file=sys.stderr,
        )

        while True:
            if time.monotonic() >= deadline:
                raise UploadError(
                    "等待 NAS 完成任务超时（{:.0f} 秒），upload_id={}；"
                    "分块已保留，请稍后重新运行原命令继续确认".format(
                        self.complete_wait_timeout, upload_id
                    )
                )

            status = self._request(
                "GET",
                "/upload_status/{}".format(upload_id),
                "upload_status",
            )
            state = status.get("state")
            if state == "completed":
                print("[INFO] NAS 状态查询已确认任务完成：{}".format(upload_id))
                return status
            if state == "error":
                raise UploadError(
                    "NAS 完成任务失败，upload_id={}：{}".format(
                        upload_id, status.get("last_error") or "未提供错误详情"
                    )
                )

            try:
                uploaded_chunk_count = int(status.get("uploaded_chunk_count", -1))
            except (TypeError, ValueError):
                raise UploadError("upload_status 返回了无效的 uploaded_chunk_count")
            if uploaded_chunk_count < total_chunks:
                raise UploadError(
                    "NAS 任务分块不完整，upload_id={}，期望={}，实际={}".format(
                        upload_id, total_chunks, uploaded_chunk_count
                    )
                )
            if state not in ("ready_to_complete", "uploading"):
                raise UploadError(
                    "NAS 返回了无法识别的任务状态，upload_id={}，state={}".format(
                        upload_id, state
                    )
                )

            remaining = max(0.0, deadline - time.monotonic())
            print(
                "[INFO] NAS 正在合并/解压：upload_id={}，state={}，"
                "剩余等待时间约 {:.0f} 秒".format(upload_id, state, remaining)
            )
            time.sleep(min(self.status_interval, remaining))

    def _complete_upload(self, upload_id, total_chunks):
        operation = "upload_complete"
        url = self.nas_url + "/upload_complete"
        try:
            response = self.session.request(
                "POST",
                url,
                timeout=self.timeout,
                json={"upload_id": upload_id},
            )
        except requests.ReadTimeout as exc:
            return self._wait_for_completion(
                upload_id,
                total_chunks,
                "响应读取超时：{}".format(exc),
            )
        except requests.RequestException as exc:
            raise UploadError("{} 请求失败: {}".format(operation, exc))

        if 200 <= response.status_code < 300:
            payload = response_payload(response, operation)
            if payload.get("ok") is not True:
                raise UploadError("{} 未返回 ok=true: {}".format(operation, payload))
            return payload

        if response.status_code in (502, 504):
            return self._wait_for_completion(
                upload_id,
                total_chunks,
                "HTTP {}".format(response.status_code),
            )

        payload = response_payload(response, operation)
        message = payload.get("error") or json.dumps(payload, ensure_ascii=False)
        raise UploadError(
            "{} 失败（HTTP {}）: {}".format(operation, response.status_code, message)
        )

    def _validate_completed(self, payload, expected_file_count, upload_id):
        if payload.get("state") != "completed":
            raise UploadError(
                "上传完成接口未确认 completed，upload_id={}，state={}".format(
                    upload_id, payload.get("state")
                )
            )
        try:
            files_saved = int(payload.get("files_saved", -1))
        except (TypeError, ValueError):
            files_saved = -1
        if files_saved != expected_file_count:
            raise UploadError(
                "NAS 保存文件数不符，upload_id={}，期望={}，实际={}".format(
                    upload_id, expected_file_count, payload.get("files_saved")
                )
            )

    def upload_archive(self, artifact, dataset_name):
        archive_size = artifact["archive_size"]
        total_chunks = max(1, (archive_size + self.chunk_size - 1) // self.chunk_size)
        upload_id = artifact["upload_id"]
        source_files = artifact["source_files"]
        init_payload = {
            "upload_id": upload_id,
            "archive_kind": artifact["archive_kind"],
            "dataset_root": dataset_name,
            "archive_name": artifact["archive_name"],
            "total_size": archive_size,
            "chunk_size": self.chunk_size,
            "total_chunks": total_chunks,
            "source_files": source_files,
        }

        print(
            "[INFO] 初始化 {} 上传：files={} tar={} bytes chunks={}".format(
                artifact["archive_kind"], len(source_files), archive_size, total_chunks
            )
        )
        status = self._request(
            "POST", "/upload_init", "upload_init", json=init_payload
        )
        if status.get("state") == "completed":
            self._validate_completed(status, len(source_files), upload_id)
            print("[INFO] NAS 已存在完整任务，跳过上传：{}".format(upload_id))
            return status

        uploaded_chunks_raw = status.get("uploaded_chunks", [])
        if not isinstance(uploaded_chunks_raw, list):
            raise UploadError("upload_init 返回的 uploaded_chunks 不是数组")
        try:
            uploaded_chunks = set(int(index) for index in uploaded_chunks_raw)
        except (TypeError, ValueError):
            raise UploadError("upload_init 返回了无效的 uploaded_chunks")
        if any(index < 0 or index >= total_chunks for index in uploaded_chunks):
            raise UploadError("upload_init 返回了超出范围的 uploaded_chunks")

        with open(artifact["archive_path"], "rb") as archive_handle:
            for chunk_index in range(total_chunks):
                if chunk_index in uploaded_chunks:
                    print(
                        "[INFO] 跳过 NAS 已接收分块：{}/{}".format(
                            chunk_index + 1, total_chunks
                        )
                    )
                    continue
                offset = chunk_index * self.chunk_size
                expected_size = min(self.chunk_size, archive_size - offset)
                archive_handle.seek(offset)
                chunk = archive_handle.read(expected_size)
                if len(chunk) != expected_size:
                    raise UploadError(
                        "读取 TAR 分块失败：index={}，期望={}，实际={}".format(
                            chunk_index, expected_size, len(chunk)
                        )
                    )
                files = {
                    "file": (
                        "{}.{:06d}.part".format(upload_id, chunk_index),
                        chunk,
                        "application/octet-stream",
                    )
                }
                form_data = {"upload_id": upload_id, "chunk_index": str(chunk_index)}
                status = self._request(
                    "POST",
                    "/upload_chunk",
                    "upload_chunk[{}]".format(chunk_index),
                    data=form_data,
                    files=files,
                )
                bytes_received = status.get("bytes_received", 0)
                try:
                    bytes_received_int = int(bytes_received)
                except (TypeError, ValueError):
                    raise UploadError("upload_chunk 返回了无效的 bytes_received")
                progress = 100.0 * min(bytes_received_int, archive_size) / float(archive_size)
                print(
                    "[INFO] 分块进度：{}/{}，NAS bytes={}/{} ({:.1f}%)".format(
                        chunk_index + 1, total_chunks, bytes_received, archive_size, progress
                    )
                )

        status = self._complete_upload(upload_id, total_chunks)
        self._validate_completed(status, len(source_files), upload_id)
        print(
            "[OK] {} 上传完成：upload_id={}，files_saved={}".format(
                artifact["archive_kind"], upload_id, status.get("files_saved")
            )
        )
        return status


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="将 G500 中的单个数采会话目录分块上传到 DS923_app.py"
    )
    parser.add_argument(
        "session_dir",
        help="会话目录，例如 /media/nvidia/G500/excavator_data_20260707/20260707082204",
    )
    parser.add_argument("--nas-url", default=DEFAULT_NAS_URL, help="NAS Flask 服务地址")
    parser.add_argument(
        "--chunk-size", type=positive_int, default=DEFAULT_CHUNK_SIZE, help="分块字节数"
    )
    parser.add_argument(
        "--connect-timeout",
        type=positive_float,
        default=DEFAULT_CONNECT_TIMEOUT,
        help="连接超时秒数",
    )
    parser.add_argument(
        "--read-timeout",
        type=positive_float,
        default=DEFAULT_READ_TIMEOUT,
        help="响应读取超时秒数",
    )
    parser.add_argument(
        "--retries",
        type=non_negative_int,
        default=DEFAULT_RETRIES,
        help="单个 HTTP 请求失败后的重试次数",
    )
    parser.add_argument(
        "--complete-wait-timeout",
        type=positive_float,
        default=DEFAULT_COMPLETE_WAIT_TIMEOUT,
        help="upload_complete 结果未知时，等待 NAS 后台完成的最长秒数",
    )
    parser.add_argument(
        "--status-interval",
        type=positive_float,
        default=DEFAULT_STATUS_INTERVAL,
        help="等待 NAS 后台完成时的状态查询间隔秒数",
    )
    parser.add_argument(
        "--temp-dir",
        default=None,
        help="临时 TAR 所在目录；默认使用系统临时目录",
    )
    return parser


def run(args):
    nas_url = validate_base_url(args.nas_url)
    session_info = collect_session_files(args.session_dir)
    grouped_files = session_info["grouped_files"]
    print(
        "[INFO] 会话校验通过：dataset={} session={} total={} state={} camera={}".format(
            session_info["dataset_name"],
            session_info["session_name"],
            session_info["total_files"],
            len(grouped_files["state"]),
            len(grouped_files["camera"]),
        )
    )

    temp_parent = None
    if args.temp_dir:
        temp_parent = os.path.abspath(os.path.expanduser(args.temp_dir))
        if os.path.exists(temp_parent) and not os.path.isdir(temp_parent):
            raise UploadError("--temp-dir 不是目录: {}".format(temp_parent))
        try:
            temp_inside_session = os.path.commonpath(
                [session_info["session_dir"], temp_parent]
            ) == session_info["session_dir"]
        except ValueError:
            temp_inside_session = False
        if temp_inside_session:
            raise UploadError("--temp-dir 不能位于待上传的会话目录内: {}".format(temp_parent))
        os.makedirs(temp_parent, exist_ok=True)

    uploader = NasUploader(
        nas_url=nas_url,
        chunk_size=args.chunk_size,
        connect_timeout=args.connect_timeout,
        read_timeout=args.read_timeout,
        retries=args.retries,
        complete_wait_timeout=args.complete_wait_timeout,
        status_interval=args.status_interval,
    )
    try:
        with tempfile.TemporaryDirectory(prefix="nas_upload_check_", dir=temp_parent) as work_dir:
            for archive_kind in ("state", "camera"):
                candidates = grouped_files[archive_kind]
                if not candidates:
                    print("[INFO] {} 类别没有文件，跳过".format(archive_kind))
                    continue
                artifact = create_archive(session_info, archive_kind, candidates, work_dir)
                print(
                    "[INFO] 已创建临时 TAR：kind={} files={} bytes={} sha256={}".format(
                        archive_kind,
                        len(candidates),
                        artifact["archive_size"],
                        artifact["upload_id"].rsplit("_", 1)[-1],
                    )
                )
                uploader.upload_archive(artifact, session_info["dataset_name"])
    finally:
        uploader.close()

    print(
        "[OK] 会话全部上传完成，G500 源数据已保留：{}".format(session_info["session_dir"])
    )


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n[ERROR] 用户中断上传；重新运行可继续缺失分块", file=sys.stderr)
        return 130
    except (UploadError, OSError, tarfile.TarError) as exc:
        print("[ERROR] {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
