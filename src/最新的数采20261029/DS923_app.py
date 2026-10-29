"""
项目          修改人     修改日期       改动
无人挖掘机    wuql17    2025/10/14    将挖掘机的状态数据发送到 NAS DS923+
无人挖掘机    wuql17    2025/10/15    将灰度图、相机图片按录制信号保存到 NAS DS923+
无人挖掘机    wuql17    2026/04/30    增加高度数据和前向灰度图保存逻辑
无人挖掘机    wuql17    2026/05/26    新增 chunk 断点续传、上传状态查询、G500 镜像目录兼容
无人挖掘机    wuql17    2026/05/29    修改代码，将不同机型的数据存放到不同的文件夹下面
无人挖掘机    wuql17    2026/07/12    修改代码，适配新的文件夹上传目录结构
无人挖掘机    wuql17    2026/08/10    新增完整会话ZIP的并发分块、断点续传、SHA-256/CRC安全校验与原子归档
无人挖掘机    wuql17    2026/08/11    新增上传缓存TTL清理、跨进程任务锁与过期后安全重传



"""

import base64
import fcntl
import hashlib
import json
import os
import re
import shutil
import stat
import tarfile
import threading
import time
import zipfile
from contextlib import contextmanager
from dataclasses import dataclass, field
from queue import Full, Queue

from flask import Flask, jsonify, request

DEFAULT_ROOT_DIR = "/volume1/sy200_data"
CLIENT_ROOT_DIR_MAP = {
    "192.168.1.102": "/volume1/sy750_data",
    "192.168.1.103": "/volume1/sy870_data",
    "192.168.1.104": "/volume1/sy1350_data",
}
SESSION_HEADER = "X-SY200-Session-Ts-Sec"
IMAGE_QUEUE_SIZE = int(os.environ.get("SY200_IMAGE_QUEUE_SIZE", "5000"))
IMAGE_WORKER_COUNT = int(os.environ.get("SY200_IMAGE_WORKERS", "4"))
STATE_FLUSH_EVERY = int(os.environ.get("SY200_STATE_FLUSH_EVERY", "20"))
STATE_FLUSH_INTERVAL_S = float(os.environ.get("SY200_STATE_FLUSH_INTERVAL_S", "0.5"))
UPLOAD_CACHE_CLEANUP_INTERVAL_S = max(
    60.0, float(os.environ.get("SY200_UPLOAD_CACHE_CLEANUP_INTERVAL_S", "3600"))
)
UPLOAD_CACHE_TTL_S = max(
    60.0, float(os.environ.get("SY200_UPLOAD_CACHE_TTL_S", "604800"))
)
DATASET_RE = re.compile(r"^excavator_data_\d{8}$")
SESSION_DIR_RE = re.compile(r"^\d{14}$")
UPLOAD_ID_RE = re.compile(r"^[A-Za-z0-9_.-]{1,200}$")
ALLOWED_MIRROR_TOP_LEVELS = frozenset({"bag", "camera_data", "grayPicture", "rangePicture"})
ALLOWED_ARCHIVE_KINDS = frozenset({"camera", "state", "session_zip"})
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COPY_BUFFER_SIZE = 1024 * 1024
INCOMING_DIRNAME = ".incoming"
CHUNKED_DIRNAME = ".incoming_chunked"


@dataclass
class SessionState:
    client_ip: str
    root_dir: str
    session_ts_sec: int
    dataset_root: str
    session_root_dir: str
    bag_dir: str
    gray_session_dir: str
    range_session_dir: str
    camera_dirs: dict
    json_tmp_path: str
    json_final_path: str
    json_file: object = None
    seen_keys: set = field(default_factory=set)
    frames_written: int = 0
    pending_flush_count: int = 0
    last_flush_monotonic: float = field(default_factory=time.monotonic)


class UploadConflictError(Exception):
    pass


_lock = threading.Lock()
_recording_by_client = {}
_last_signal_by_client = {}
_active_session_ts_sec_by_client = {}
_sessions = {}
_image_save_q = Queue(maxsize=IMAGE_QUEUE_SIZE)
_workers_started = False
_upload_cache_cleanup_started = False
_upload_cache_cleanup_start_lock = threading.Lock()
_upload_cache_cleanup_thread = None
_upload_cache_cleanup_pid = None


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def remove_file_if_exists(path: str):
    try:
        if path and os.path.exists(path):
            os.remove(path)
    except OSError:
        pass


def write_json_atomic(path: str, payload):
    ensure_dir(os.path.dirname(path))
    tmp_path = path + ".tmp"
    try:
        with open(tmp_path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=True)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_path, path)
    finally:
        remove_file_if_exists(tmp_path)


def normalize_client_ip(client_ip: str):
    client_ip = (client_ip or "").strip()
    if client_ip.startswith("::ffff:"):
        client_ip = client_ip[len("::ffff:"):]
    return client_ip or "unknown"


def get_request_client_ip():
    return normalize_client_ip(request.remote_addr)


def resolve_root_dir_for_client(client_ip: str):
    return CLIENT_ROOT_DIR_MAP.get(client_ip, DEFAULT_ROOT_DIR)


def resolve_root_dir_for_request():
    client_ip = get_request_client_ip()
    return client_ip, resolve_root_dir_for_client(client_ip)


def build_session_paths(root_dir: str, session_ts_sec: int):
    local_time = time.localtime(session_ts_sec)
    ymd = time.strftime("%Y%m%d", local_time)
    compact = time.strftime("%Y%m%d%H%M%S", local_time)
    bag_name = time.strftime("%Y%m%d-%H%M%S", local_time)
    human = time.strftime("%Y-%m-%d_%H-%M-%S", local_time)
    dataset_root = f"excavator_data_{ymd}"
    session_root_dir = os.path.join(root_dir, dataset_root, compact)
    bag_dir = os.path.join(session_root_dir, "bag")
    gray_dir = os.path.join(session_root_dir, "grayPicture", f"{human}_png")
    range_dir = os.path.join(session_root_dir, "rangePicture", f"{human}_range_images")
    camera_root = os.path.join(session_root_dir, "camera_data")
    camera_dirs = {
        "camera_0": os.path.join(camera_root, f"{compact}_camera_0"),
        "camera_1": os.path.join(camera_root, f"{compact}_camera_1"),
    }
    json_final_path = os.path.join(bag_dir, f"{bag_name}.json")
    json_tmp_path = json_final_path + ".tmp"
    return {
        "dataset_root": dataset_root,
        "session_root_dir": session_root_dir,
        "bag_dir": bag_dir,
        "gray_session_dir": gray_dir,
        "range_session_dir": range_dir,
        "camera_dirs": camera_dirs,
        "json_tmp_path": json_tmp_path,
        "json_final_path": json_final_path,
    }


def open_session_file_locked(session: SessionState):
    if session.json_file is not None:
        return
    ensure_dir(session.bag_dir)
    session.json_file = open(session.json_tmp_path, "a", encoding="utf-8", buffering=1)


def session_key(client_ip: str, session_ts_sec: int):
    return client_ip, session_ts_sec


def ensure_session_state_locked(client_ip: str, root_dir: str, session_ts_sec: int):
    key = session_key(client_ip, session_ts_sec)
    session = _sessions.get(key)
    if session is not None:
        if session.json_file is None:
            open_session_file_locked(session)
        return session

    paths = build_session_paths(root_dir, session_ts_sec)
    ensure_dir(paths["session_root_dir"])

    session = SessionState(
        client_ip=client_ip,
        root_dir=root_dir,
        session_ts_sec=session_ts_sec,
        dataset_root=paths["dataset_root"],
        session_root_dir=paths["session_root_dir"],
        bag_dir=paths["bag_dir"],
        gray_session_dir=paths["gray_session_dir"],
        range_session_dir=paths["range_session_dir"],
        camera_dirs=paths["camera_dirs"],
        json_tmp_path=paths["json_tmp_path"],
        json_final_path=paths["json_final_path"],
    )
    open_session_file_locked(session)
    _sessions[key] = session
    return session


def flush_session_locked(session: SessionState, force: bool = False):
    if session.json_file is None:
        return
    now = time.monotonic()
    if not force:
        if session.pending_flush_count < STATE_FLUSH_EVERY and (now - session.last_flush_monotonic) < STATE_FLUSH_INTERVAL_S:
            return

    session.json_file.flush()
    if force:
        try:
            os.fsync(session.json_file.fileno())
        except OSError:
            pass
    session.pending_flush_count = 0
    session.last_flush_monotonic = now


def close_session_locked(session: SessionState):
    if session.json_file is None:
        return None
    flush_session_locked(session, force=True)
    session.json_file.close()
    session.json_file = None
    ensure_dir(session.bag_dir)
    if os.path.exists(session.json_tmp_path):
        os.replace(session.json_tmp_path, session.json_final_path)
        return session.json_final_path
    return None


def parse_session_ts_header():
    header_value = request.headers.get(SESSION_HEADER)
    if header_value in (None, ""):
        return None, None
    try:
        session_ts_sec = int(header_value)
        if session_ts_sec <= 0:
            raise ValueError
    except ValueError:
        return None, jsonify({"error": f"{SESSION_HEADER} must be a positive integer"})
    return session_ts_sec, None


def resolve_request_session_context():
    client_ip, root_dir = resolve_root_dir_for_request()
    session_ts_sec, error_response = parse_session_ts_header()
    if error_response is not None:
        return client_ip, root_dir, None, error_response
    if session_ts_sec is not None:
        return client_ip, root_dir, session_ts_sec, None
    with _lock:
        return client_ip, root_dir, _active_session_ts_sec_by_client.get(client_ip), None


def image_target_for_kind_locked(session: SessionState, kind: str):
    if kind == "gridmap":
        return session.gray_session_dir, ".png", ""
    if kind == "rangepic":
        return session.range_session_dir, ".png", "range_"
    return session.camera_dirs[kind], ".jpg", ""


def image_writer_loop():
    while True:
        kind, client_ip, root_dir, session_ts_sec, batch = _image_save_q.get()
        try:
            with _lock:
                session = ensure_session_state_locked(client_ip, root_dir, session_ts_sec)
                session_dir, ext, prefix = image_target_for_kind_locked(session, kind)
            ensure_dir(session_dir)

            for ts_ms, meta in batch.items():
                try:
                    if not isinstance(meta, dict):
                        raise ValueError("meta not dict")
                    b64 = meta.get("data_base64")
                    if not b64:
                        raise ValueError("missing data_base64")
                    raw = base64.b64decode(b64)
                    out_path = os.path.join(session_dir, f"{prefix}{ts_ms}{ext}")
                    with open(out_path, "wb") as fh:
                        fh.write(raw)
                except Exception as exc:
                    print(f"[ERROR] save {kind} frame {ts_ms} failed: {exc}")
        except Exception as exc:
            print(f"[ERROR] image writer failed: {exc}")
        finally:
            _image_save_q.task_done()


def start_image_workers_once():
    global _workers_started
    with _lock:
        if _workers_started:
            return
        for index in range(IMAGE_WORKER_COUNT):
            thread = threading.Thread(target=image_writer_loop, daemon=True, name=f"image-writer-{index}")
            thread.start()
        _workers_started = True


def session_response_payload(session: SessionState):
    if session is None:
        return {
            "dataset_root": None,
            "gray_session_dir": None,
            "range_session_dir": None,
            "camera_dirs": {"camera_0": None, "camera_1": None},
            "session_root": None,
            "session_ts_sec": None,
            "json_path": None,
        }
    return {
        "dataset_root": session.dataset_root,
        "gray_session_dir": session.gray_session_dir,
        "range_session_dir": session.range_session_dir,
        "camera_dirs": session.camera_dirs,
        "session_root": session.session_root_dir,
        "session_ts_sec": session.session_ts_sec,
        "json_path": session.json_final_path,
    }


def validate_dataset_root(dataset_root: str):
    if not DATASET_RE.fullmatch(dataset_root or ""):
        raise ValueError("dataset_root must match excavator_data_YYYYMMDD")
    return dataset_root


def normalize_relative_path(relative_path: str, dataset_root: str = ""):
    normalized = (relative_path or "").replace("\\", "/")
    if normalized.startswith("/"):
        raise ValueError("relative_path must not be absolute")
    if len(normalized) >= 2 and normalized[1] == ":":
        raise ValueError("relative_path must not contain a drive prefix")

    parts = []
    for piece in normalized.split("/"):
        if piece in ("", "."):
            continue
        if piece == "..":
            raise ValueError("relative_path must not contain '..'")
        parts.append(piece)
    if not parts:
        raise ValueError("relative_path is required")

    if parts[0] in ALLOWED_MIRROR_TOP_LEVELS:
        pass
    elif SESSION_DIR_RE.fullmatch(parts[0]):
        if len(parts) < 2 or parts[1] not in ALLOWED_MIRROR_TOP_LEVELS:
            raise ValueError(
                "session relative_path must continue with bag, camera_data, grayPicture, or rangePicture"
            )
        if dataset_root:
            dataset_root = validate_dataset_root(dataset_root)
            if parts[0][:8] != dataset_root.rsplit("_", 1)[-1]:
                raise ValueError("session directory date must match dataset_root")
    else:
        raise ValueError(
            "relative_path must start with bag, camera_data, grayPicture, rangePicture, "
            "or a 14-digit session directory"
        )
    return os.path.join(*parts), "/".join(parts)


def build_mirror_target(root_dir: str, dataset_root: str, relative_path: str):
    dataset_root = validate_dataset_root(dataset_root)
    relative_path_os, relative_path_posix = normalize_relative_path(relative_path, dataset_root)
    dataset_dir = os.path.abspath(os.path.join(root_dir, dataset_root))
    final_path = os.path.abspath(os.path.join(dataset_dir, relative_path_os))
    if os.path.commonpath([dataset_dir, final_path]) != dataset_dir:
        raise ValueError("relative_path escapes dataset_root")
    return dataset_dir, final_path, relative_path_posix


def parse_source_mtime_ms(value):
    if value in (None, ""):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("source_mtime_ms must be an integer") from exc
    if parsed < 0:
        raise ValueError("source_mtime_ms must be >= 0")
    return parsed


def parse_archive_kind(value):
    archive_kind = (value or "").strip()
    if archive_kind not in ALLOWED_ARCHIVE_KINDS:
        raise ValueError("archive_kind must be 'camera', 'state', or 'session_zip'")
    return archive_kind


def validate_session_name(value: str, dataset_root: str):
    session_name = (value or "").strip()
    if not SESSION_DIR_RE.fullmatch(session_name):
        raise ValueError("session_name must be YYYYMMDDhhmmss")
    if not DATASET_RE.fullmatch(dataset_root) or session_name[:8] != dataset_root[-8:]:
        raise ValueError("session_name date must match dataset_root")
    return session_name


def validate_sha256(value: str):
    digest = (value or "").strip().lower()
    if not SHA256_RE.fullmatch(digest):
        raise ValueError("archive_sha256 must be a lowercase SHA-256 hex digest")
    return digest


def sha256_file(path: str):
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        while True:
            block = fh.read(COPY_BUFFER_SIZE)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def session_zip_final_path(root_dir: str, dataset_root: str, archive_name: str):
    return os.path.join(root_dir, dataset_root, archive_name)


def validate_session_zip(archive_path: str, session_name: str, expected_entries: int):
    allowed_top_levels = ALLOWED_MIRROR_TOP_LEVELS
    regular_count = 0
    seen = set()
    try:
        with zipfile.ZipFile(archive_path, "r", allowZip64=True) as archive:
            for info in archive.infolist():
                raw_name = info.filename.replace("\\", "/")
                if raw_name.startswith("/") or re.match(r"^[A-Za-z]:", raw_name):
                    raise ValueError(f"unsafe absolute ZIP member: {raw_name}")
                parts = [part for part in raw_name.split("/") if part not in ("", ".")]
                if any(part == ".." for part in parts):
                    raise ValueError(f"unsafe parent ZIP member: {raw_name}")
                if not parts or parts[0] != session_name:
                    raise ValueError(f"ZIP member is outside session directory: {raw_name}")
                if len(parts) >= 2 and parts[1] not in allowed_top_levels:
                    raise ValueError(f"ZIP member has unsupported top-level directory: {raw_name}")
                unix_mode = (info.external_attr >> 16) & 0xFFFF
                if stat.S_ISLNK(unix_mode):
                    raise ValueError(f"ZIP symbolic links are not allowed: {raw_name}")
                file_type = stat.S_IFMT(unix_mode)
                if info.is_dir():
                    if file_type not in (0, stat.S_IFDIR):
                        raise ValueError(f"unsupported ZIP directory type: {raw_name}")
                    continue
                if file_type not in (0, stat.S_IFREG):
                    raise ValueError(f"unsupported ZIP member type: {raw_name}")
                if len(parts) < 3:
                    raise ValueError(f"ZIP regular file must be inside a data directory: {raw_name}")
                normalized = "/".join(parts)
                if normalized in seen:
                    raise ValueError(f"duplicate ZIP member: {normalized}")
                seen.add(normalized)
                regular_count += 1
            bad_member = archive.testzip()
            if bad_member is not None:
                raise ValueError(f"ZIP CRC failed: {bad_member}")
    except ValueError:
        raise
    except (zipfile.BadZipFile, RuntimeError, OSError) as exc:
        raise ValueError(f"invalid ZIP archive: {exc}") from exc
    if regular_count <= 0:
        raise ValueError("ZIP contains no regular files")
    # Legacy damaged archives may be reconstructed locally without their old
    # entry-count state. A zero count means "verify and report the actual count".
    if expected_entries > 0 and regular_count != expected_entries:
        raise ValueError(
            f"ZIP entry count mismatch: expected {expected_entries}, got {regular_count}"
        )
    return regular_count


def parse_source_files_value(payload, dataset_root: str = ""):
    if isinstance(payload, list):
        parsed = payload
    else:
        raw = payload
        if raw in (None, ""):
            raise ValueError("source_files_json is required")
        if isinstance(raw, str):
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError("source_files_json must be valid JSON") from exc
        else:
            raise ValueError("source_files_json must be valid JSON")

    if not isinstance(parsed, list) or not parsed:
        raise ValueError("source_files_json must be a non-empty JSON array")

    normalized_paths = []
    seen = set()
    for item in parsed:
        if not isinstance(item, str):
            raise ValueError("source_files_json items must be strings")
        _, relative_path_posix = normalize_relative_path(item, dataset_root)
        if relative_path_posix in seen:
            raise ValueError("source_files_json must not contain duplicates")
        seen.add(relative_path_posix)
        normalized_paths.append(relative_path_posix)
    return normalized_paths


def save_upload_atomically(file_storage, final_path: str):
    ensure_dir(os.path.dirname(final_path))
    tmp_path = final_path + ".tmp"
    try:
        file_storage.save(tmp_path)
        os.replace(tmp_path, final_path)
    except Exception:
        remove_file_if_exists(tmp_path)
        raise


def apply_source_mtime(final_path: str, source_mtime_ms):
    if source_mtime_ms is None:
        return
    mtime_sec = source_mtime_ms / 1000.0
    os.utime(final_path, (mtime_sec, mtime_sec))


def save_stream_atomically(stream, final_path: str, source_mtime_ms=None):
    ensure_dir(os.path.dirname(final_path))
    tmp_path = final_path + ".tmp"
    try:
        with open(tmp_path, "wb") as fh:
            while True:
                chunk = stream.read(COPY_BUFFER_SIZE)
                if not chunk:
                    break
                fh.write(chunk)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_path, final_path)
        apply_source_mtime(final_path, source_mtime_ms)
    except Exception:
        remove_file_if_exists(tmp_path)
        raise


def incoming_archive_path(root_dir: str, dataset_root: str, archive_kind: str, original_name: str):
    safe_name = os.path.basename(original_name or "upload.tar")
    if not safe_name.endswith(".tar"):
        safe_name = safe_name + ".tar"
    incoming_dir = os.path.join(root_dir, INCOMING_DIRNAME, dataset_root)
    ensure_dir(incoming_dir)
    token = f"{archive_kind}_{int(time.time() * 1000)}_{threading.get_ident()}"
    return os.path.join(incoming_dir, f"{token}_{safe_name}")


def extract_tar_archive(archive_path: str, root_dir: str, dataset_root: str, expected_paths):
    expected_sorted = sorted(expected_paths)
    saved_paths = []

    with tarfile.open(archive_path, "r:") as archive:
        members = archive.getmembers()
        actual_paths = []
        regular_members = []

        for member in members:
            if member.isdir():
                continue
            if not member.isfile():
                raise ValueError(f"unsupported tar member type: {member.name}")
            _, _, normalized_path = build_mirror_target(root_dir, dataset_root, member.name)
            regular_members.append((member, normalized_path))
            actual_paths.append(normalized_path)

        if not regular_members:
            raise ValueError("archive does not contain any regular files")
        if sorted(actual_paths) != expected_sorted:
            raise ValueError("archive members do not match source_files_json")

        for member, normalized_path in regular_members:
            _, final_path, _ = build_mirror_target(root_dir, dataset_root, normalized_path)
            extracted = archive.extractfile(member)
            if extracted is None:
                raise ValueError(f"failed to extract tar member: {member.name}")
            with extracted:
                save_stream_atomically(extracted, final_path, int(member.mtime * 1000))
            saved_paths.append(normalized_path)

    return saved_paths


def parse_positive_int(field_name: str, value, *, minimum: int = 1):
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be an integer") from exc
    if parsed < minimum:
        raise ValueError(f"{field_name} must be >= {minimum}")
    return parsed


def validate_upload_id(upload_id: str):
    upload_id = (upload_id or "").strip()
    if not UPLOAD_ID_RE.fullmatch(upload_id):
        raise ValueError("upload_id is invalid")
    return upload_id


def chunk_upload_root(root_dir: str, upload_id: str):
    return os.path.join(root_dir, CHUNKED_DIRNAME, upload_id)


def chunk_parts_dir(root_dir: str, upload_id: str):
    return os.path.join(chunk_upload_root(root_dir, upload_id), "parts")


def chunk_meta_path(root_dir: str, upload_id: str):
    return os.path.join(chunk_upload_root(root_dir, upload_id), "meta.json")


@contextmanager
def upload_meta_lock(root_dir: str, upload_id: str, *, blocking: bool = True):
    """Cross-thread/process lock for one upload's mutable metadata."""
    upload_root = chunk_upload_root(root_dir, upload_id)
    ensure_dir(upload_root)
    lock_path = os.path.join(upload_root, ".lock")
    with open(lock_path, "a+b") as lock_fh:
        lock_flags = fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB)
        acquired = False
        try:
            try:
                fcntl.flock(lock_fh.fileno(), lock_flags)
                acquired = True
            except BlockingIOError:
                yield False
                return
            yield True
        finally:
            if acquired:
                fcntl.flock(lock_fh.fileno(), fcntl.LOCK_UN)


@contextmanager
def upload_data_lock(root_dir: str, upload_id: str, *, shared: bool, blocking: bool = True):
    """Shared for concurrent chunk writes; exclusive for complete/cleanup."""
    upload_root = chunk_upload_root(root_dir, upload_id)
    ensure_dir(upload_root)
    lock_path = os.path.join(upload_root, ".data.lock")
    with open(lock_path, "a+b") as lock_fh:
        lock_mode = fcntl.LOCK_SH if shared else fcntl.LOCK_EX
        lock_flags = lock_mode | (0 if blocking else fcntl.LOCK_NB)
        acquired = False
        try:
            try:
                fcntl.flock(lock_fh.fileno(), lock_flags)
                acquired = True
            except BlockingIOError:
                yield False
                return
            yield True
        finally:
            if acquired:
                fcntl.flock(lock_fh.fileno(), fcntl.LOCK_UN)


def chunk_archive_path(meta: dict):
    root_dir = meta.get("root_dir", DEFAULT_ROOT_DIR)
    return os.path.join(chunk_upload_root(root_dir, meta["upload_id"]), meta["archive_name"])


def chunk_part_path(root_dir: str, upload_id: str, chunk_index: int):
    return os.path.join(chunk_parts_dir(root_dir, upload_id), f"{chunk_index:06d}.part")


def hydrate_upload_meta(meta: dict, root_dir: str, client_ip: str = ""):
    # The directory selected by the server/request is authoritative. Never let
    # a persisted (or corrupted) meta.json redirect file operations elsewhere.
    meta["root_dir"] = root_dir
    meta.setdefault("client_ip", client_ip)
    session_name = meta.get("session_name", "")
    meta.setdefault(
        "is_damaged",
        bool(session_name) and meta.get("archive_name") == f"{session_name}_Damaged.zip",
    )
    return meta


def load_upload_meta(root_dir: str, upload_id: str, client_ip: str = ""):
    meta_path = chunk_meta_path(root_dir, upload_id)
    if not os.path.exists(meta_path):
        return None
    with open(meta_path, "r", encoding="utf-8") as fh:
        meta = json.load(fh)
    if not isinstance(meta, dict) or meta.get("upload_id") != upload_id:
        raise ValueError("upload metadata identity is invalid")
    return hydrate_upload_meta(meta, root_dir, client_ip)


def save_upload_meta(meta: dict):
    root_dir = meta.setdefault("root_dir", DEFAULT_ROOT_DIR)
    meta.setdefault("client_ip", "")
    meta["updated_at"] = time.time()
    write_json_atomic(chunk_meta_path(root_dir, meta["upload_id"]), meta)


def expected_chunk_size(meta: dict, chunk_index: int):
    total_chunks = int(meta["total_chunks"])
    chunk_size = int(meta["chunk_size"])
    total_size = int(meta["total_size"])
    if chunk_index < 0 or chunk_index >= total_chunks:
        raise ValueError("chunk_index out of range")
    offset = chunk_index * chunk_size
    remaining = total_size - offset
    if remaining <= 0:
        raise ValueError("chunk_index exceeds total_size")
    return min(chunk_size, remaining)


def refresh_upload_meta(meta: dict):
    root_dir = meta.get("root_dir", DEFAULT_ROOT_DIR)
    upload_id = meta["upload_id"]
    parts_dir = chunk_parts_dir(root_dir, upload_id)

    if meta.get("state") == "completed":
        if meta.get("archive_kind") == "session_zip":
            final_path = meta.get("final_archive_path", "")
            expected_size = int(meta.get("archive_size", meta.get("total_size", 0)))
            if not final_path or not os.path.isfile(final_path) or os.path.getsize(final_path) != expected_size:
                meta["state"] = "error"
                meta["last_error"] = "completed session ZIP is missing or has changed size"
                return meta
        total_chunks = int(meta["total_chunks"])
        # Completed uploads no longer persist a potentially huge chunk-index array.
        meta["uploaded_chunks"] = []
        meta["uploaded_chunk_count"] = total_chunks
        meta["bytes_received"] = int(meta["total_size"])
        return meta

    ensure_dir(parts_dir)

    uploaded_chunks = []
    bytes_received = 0
    for chunk_index in range(int(meta["total_chunks"])):
        part_path = chunk_part_path(root_dir, upload_id, chunk_index)
        if not os.path.exists(part_path):
            continue
        expected_size = expected_chunk_size(meta, chunk_index)
        actual_size = os.path.getsize(part_path)
        if actual_size == expected_size:
            uploaded_chunks.append(chunk_index)
            bytes_received += actual_size

    meta["uploaded_chunks"] = uploaded_chunks
    meta["uploaded_chunk_count"] = len(uploaded_chunks)
    meta["bytes_received"] = bytes_received
    if meta.get("state") != "completed":
        if len(uploaded_chunks) == int(meta["total_chunks"]):
            meta["state"] = "ready_to_complete"
        elif uploaded_chunks:
            meta["state"] = "uploading"
        else:
            meta["state"] = "initialized"
    return meta


def upload_status_payload(meta: dict, include_chunks: bool = False):
    payload = {
        "upload_id": meta["upload_id"],
        "archive_kind": meta["archive_kind"],
        "dataset_root": meta["dataset_root"],
        "archive_name": meta["archive_name"],
        "state": meta.get("state", "initialized"),
        "total_size": int(meta["total_size"]),
        "chunk_size": int(meta["chunk_size"]),
        "total_chunks": int(meta["total_chunks"]),
        "uploaded_chunk_count": int(meta.get("uploaded_chunk_count", 0)),
        "bytes_received": int(meta.get("bytes_received", 0)),
        "files_saved": int(meta.get("files_saved", 0)),
        "last_error": meta.get("last_error"),
        "created_at": meta.get("created_at"),
        "updated_at": meta.get("updated_at"),
    }
    for field in ("cache_evicted_at", "cache_evicted_bytes", "cache_eviction_count"):
        if field in meta:
            payload[field] = meta[field]
    if meta.get("archive_kind") == "session_zip":
        payload.update({
            "session_name": meta.get("session_name"),
            "is_damaged": bool(meta.get("is_damaged", False)),
            "archive_size": int(meta.get("archive_size", meta.get("total_size", 0))),
            "archive_sha256": meta.get("verified_archive_sha256", meta.get("archive_sha256")),
            "archive_relative_path": meta.get("archive_relative_path"),
            "archive_entries": int(meta.get("archive_entries", 0)),
            "entries_verified": int(meta.get("entries_verified", 0)),
        })
    if include_chunks:
        payload["uploaded_chunks"] = list(meta.get("uploaded_chunks", []))
        payload["source_files"] = list(meta.get("source_files", []))
    return payload


def parse_request_payload():
    data = request.get_json(silent=True)
    if isinstance(data, dict):
        return data
    return request.form.to_dict(flat=True)


def normalize_upload_meta(payload: dict, root_dir: str, client_ip: str):
    upload_id = validate_upload_id(payload.get("upload_id"))
    dataset_root = validate_dataset_root(payload.get("dataset_root", ""))
    archive_kind = parse_archive_kind(payload.get("archive_kind", ""))

    total_size = parse_positive_int("total_size", payload.get("total_size"))
    chunk_size = parse_positive_int("chunk_size", payload.get("chunk_size"))
    total_chunks = parse_positive_int("total_chunks", payload.get("total_chunks"))
    expected_total_chunks = (total_size + chunk_size - 1) // chunk_size
    if total_chunks != expected_total_chunks:
        raise ValueError("total_chunks does not match total_size/chunk_size")

    session_name = None
    archive_sha256 = None
    archive_entries = 0
    if archive_kind == "session_zip":
        session_name = validate_session_name(payload.get("session_name"), dataset_root)
        archive_name = payload.get("archive_name") or ""
        normal_archive_name = f"{session_name}.zip"
        damaged_archive_name = f"{session_name}_Damaged.zip"
        if archive_name not in (normal_archive_name, damaged_archive_name):
            raise ValueError(
                f"archive_name must be {normal_archive_name} or {damaged_archive_name}"
            )
        is_damaged = archive_name == damaged_archive_name
        archive_sha256 = validate_sha256(payload.get("archive_sha256"))
        archive_entries = parse_positive_int(
            "archive_entries", payload.get("archive_entries"), minimum=0 if is_damaged else 1
        )
        source_files = []
    else:
        archive_name = os.path.basename(payload.get("archive_name") or f"{upload_id}.tar")
        if not archive_name.endswith(".tar"):
            archive_name = archive_name + ".tar"
        source_files_value = payload.get("source_files")
        if source_files_value in (None, ""):
            source_files_value = payload.get("source_files_json")
        source_files = parse_source_files_value(source_files_value, dataset_root)

    now = time.time()
    meta = {
        "upload_id": upload_id,
        "client_ip": client_ip,
        "root_dir": root_dir,
        "archive_kind": archive_kind,
        "dataset_root": dataset_root,
        "archive_name": archive_name,
        "total_size": total_size,
        "chunk_size": chunk_size,
        "total_chunks": total_chunks,
        "source_files": source_files,
        "uploaded_chunks": [],
        "uploaded_chunk_count": 0,
        "bytes_received": 0,
        "files_saved": 0,
        "state": "initialized",
        "last_error": None,
        "created_at": now,
        "updated_at": now,
    }
    if archive_kind == "session_zip":
        meta.update({
            "session_name": session_name,
            "is_damaged": is_damaged,
            "archive_sha256": archive_sha256,
            "archive_entries": archive_entries,
        })
    return meta


def ensure_upload_meta(payload: dict, root_dir: str, client_ip: str):
    incoming = normalize_upload_meta(payload, root_dir, client_ip)
    upload_id = incoming["upload_id"]
    ensure_dir(chunk_parts_dir(root_dir, upload_id))

    existing = load_upload_meta(root_dir, upload_id, client_ip)
    if existing is not None:
        immutable_fields = (
            "archive_kind",
            "dataset_root",
            "archive_name",
            "total_size",
            "chunk_size",
            "total_chunks",
            "source_files",
        )
        if incoming["archive_kind"] == "session_zip":
            immutable_fields += (
                "session_name", "is_damaged", "archive_sha256", "archive_entries"
            )
        for key in immutable_fields:
            if existing.get(key) != incoming.get(key):
                raise UploadConflictError(f"upload_id conflict on {key}")
        meta = existing
    else:
        meta = incoming

    refresh_upload_meta(meta)
    meta["last_error"] = None
    save_upload_meta(meta)
    return meta


def save_chunk_atomically(file_storage, final_path: str, expected_size: int):
    ensure_dir(os.path.dirname(final_path))
    tmp_path = f"{final_path}.{os.getpid()}.{threading.get_ident()}.tmp"
    try:
        file_storage.save(tmp_path)
        actual_size = os.path.getsize(tmp_path)
        if actual_size != expected_size:
            raise ValueError(f"chunk size mismatch: expected {expected_size}, got {actual_size}")
        os.replace(tmp_path, final_path)
    except Exception:
        remove_file_if_exists(tmp_path)
        raise


def assemble_archive_from_parts(meta: dict):
    root_dir = meta.get("root_dir", DEFAULT_ROOT_DIR)
    upload_id = meta["upload_id"]
    archive_path = chunk_archive_path(meta)
    tmp_path = archive_path + ".tmp"
    try:
        with open(tmp_path, "wb") as out_fh:
            for chunk_index in range(int(meta["total_chunks"])):
                part_path = chunk_part_path(root_dir, upload_id, chunk_index)
                with open(part_path, "rb") as in_fh:
                    shutil.copyfileobj(in_fh, out_fh, COPY_BUFFER_SIZE)
            out_fh.flush()
            os.fsync(out_fh.fileno())
        os.replace(tmp_path, archive_path)
        return archive_path
    except Exception:
        remove_file_if_exists(tmp_path)
        raise


def finalize_session_zip(meta: dict, archive_path: str):
    session_name = meta.get("session_name", "")
    is_damaged = bool(meta.get("is_damaged", False))
    expected_archive_name = (
        f"{session_name}_Damaged.zip" if is_damaged else f"{session_name}.zip"
    )
    if meta.get("archive_name") != expected_archive_name:
        raise ValueError("session ZIP archive_name/is_damaged metadata is inconsistent")
    expected_entries = int(meta["archive_entries"])
    if expected_entries == 0 and not is_damaged:
        raise ValueError("archive_entries=0 is allowed only for damaged session ZIPs")

    expected_size = int(meta["total_size"])
    actual_size = os.path.getsize(archive_path)
    if actual_size != expected_size:
        raise ValueError(f"assembled ZIP size mismatch: expected {expected_size}, got {actual_size}")

    actual_sha256 = sha256_file(archive_path)
    if actual_sha256 != meta["archive_sha256"]:
        raise ValueError("assembled ZIP SHA-256 does not match upload_init")

    entries_verified = validate_session_zip(
        archive_path,
        session_name,
        expected_entries,
    )
    root_dir = os.path.abspath(meta.get("root_dir", DEFAULT_ROOT_DIR))
    dataset_dir = os.path.abspath(os.path.join(root_dir, meta["dataset_root"]))
    if os.path.commonpath([root_dir, dataset_dir]) != root_dir:
        raise ValueError("dataset destination escapes configured NAS root")
    ensure_dir(dataset_dir)
    if os.path.islink(dataset_dir):
        raise ValueError("dataset destination must not be a symbolic link")

    final_path = os.path.abspath(session_zip_final_path(
        root_dir, meta["dataset_root"], meta["archive_name"]
    ))
    if os.path.commonpath([dataset_dir, final_path]) != dataset_dir:
        raise ValueError("session ZIP destination escapes dataset directory")
    if os.path.lexists(final_path):
        if os.path.islink(final_path) or not os.path.isfile(final_path):
            raise UploadConflictError("session ZIP destination exists and is not a regular file")
        if os.path.getsize(final_path) != actual_size or sha256_file(final_path) != actual_sha256:
            raise UploadConflictError("session ZIP destination already exists with different content")
        remove_file_if_exists(archive_path)
    else:
        os.replace(archive_path, final_path)
        directory_fd = os.open(dataset_dir, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    meta.update({
        "state": "completed",
        "archive_size": actual_size,
        "verified_archive_sha256": actual_sha256,
        "archive_relative_path": f"{meta['dataset_root']}/{meta['archive_name']}",
        "entries_verified": entries_verified,
        "files_saved": entries_verified,
        "final_archive_path": final_path,
        "uploaded_chunk_count": int(meta["total_chunks"]),
        "bytes_received": actual_size,
        "last_error": None,
    })
    return final_path


def _path_is_within(path: str, root: str):
    path = os.path.abspath(path)
    root = os.path.abspath(root)
    try:
        return os.path.commonpath([path, root]) == root
    except ValueError:
        return False


def _remove_regular_file_safely(path: str, managed_root: str):
    if not _path_is_within(path, managed_root) or not os.path.lexists(path):
        return 0
    info = os.lstat(path)
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        return 0
    size = info.st_size
    os.remove(path)
    return size


def _remove_controlled_tree_safely(path: str, managed_root: str):
    if not _path_is_within(path, managed_root) or not os.path.lexists(path):
        return 0
    root_info = os.lstat(path)
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        return 0

    total_size = 0
    for current_root, dir_names, file_names in os.walk(path, topdown=True, followlinks=False):
        if not _path_is_within(current_root, path):
            return 0
        for name in dir_names + file_names:
            item_path = os.path.join(current_root, name)
            item_info = os.lstat(item_path)
            if stat.S_ISLNK(item_info.st_mode):
                return 0
            if name in dir_names:
                if not stat.S_ISDIR(item_info.st_mode):
                    return 0
            elif not stat.S_ISREG(item_info.st_mode):
                return 0
            else:
                total_size += item_info.st_size
    shutil.rmtree(path)
    return total_size


def cleanup_chunk_workspace(meta: dict):
    root_dir = meta.get("root_dir", DEFAULT_ROOT_DIR)
    upload_root = chunk_upload_root(root_dir, meta["upload_id"])
    freed = _remove_controlled_tree_safely(
        chunk_parts_dir(root_dir, meta["upload_id"]), upload_root
    )
    archive_path = chunk_archive_path(meta)
    freed += _remove_regular_file_safely(archive_path, upload_root)
    freed += _remove_regular_file_safely(archive_path + ".tmp", upload_root)
    return freed


def _workspace_latest_mtime(upload_root: str):
    latest = os.path.getmtime(upload_root)
    for current_root, dir_names, file_names in os.walk(upload_root, topdown=True, followlinks=False):
        for name in dir_names + file_names:
            path = os.path.join(current_root, name)
            try:
                info = os.lstat(path)
            except FileNotFoundError:
                continue
            if stat.S_ISLNK(info.st_mode):
                continue
            latest = max(latest, info.st_mtime)
    return latest


def _cleanup_malformed_upload_workspace(root_dir: str, upload_id: str):
    upload_root = chunk_upload_root(root_dir, upload_id)
    freed = _remove_controlled_tree_safely(
        chunk_parts_dir(root_dir, upload_id), upload_root
    )
    for name in os.listdir(upload_root):
        # Keep locks and even an unreadable meta.json as small forensic/recovery
        # records. Only controlled payload/cache files are eligible for eviction.
        if name in (".lock", ".data.lock", "meta.json"):
            continue
        path = os.path.join(upload_root, name)
        if name.endswith((".tmp", ".tar", ".zip")):
            freed += _remove_regular_file_safely(path, upload_root)
    return freed


def cleanup_upload_cache_root(root_dir: str, *, now: float = None):
    """Evict inactive chunk bytes while retaining enough metadata to restart safely."""
    now = time.time() if now is None else float(now)
    chunked_root = os.path.join(root_dir, CHUNKED_DIRNAME)
    ensure_dir(chunked_root)
    cleanup_lock_path = os.path.join(chunked_root, ".cleanup.lock")
    with open(cleanup_lock_path, "a+b") as cleanup_lock:
        try:
            fcntl.flock(cleanup_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return {"tasks_evicted": 0, "bytes_evicted": 0, "lock_busy": True}
        try:
            tasks_evicted = 0
            bytes_evicted = 0
            for upload_id in sorted(os.listdir(chunked_root)):
                if not UPLOAD_ID_RE.fullmatch(upload_id):
                    continue
                upload_root = chunk_upload_root(root_dir, upload_id)
                try:
                    root_info = os.lstat(upload_root)
                except FileNotFoundError:
                    continue
                if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
                    continue
                try:
                    prelock_latest_mtime = _workspace_latest_mtime(upload_root)
                except OSError:
                    continue

                with upload_data_lock(root_dir, upload_id, shared=False, blocking=False) as data_locked:
                    if not data_locked:
                        continue
                    with upload_meta_lock(root_dir, upload_id, blocking=False) as meta_locked:
                        if not meta_locked:
                            continue
                        try:
                            meta = load_upload_meta(root_dir, upload_id)
                        except (OSError, ValueError, json.JSONDecodeError):
                            meta = None
                        if meta is None:
                            inactive_for = now - prelock_latest_mtime
                            if inactive_for < UPLOAD_CACHE_TTL_S:
                                continue
                            freed = _cleanup_malformed_upload_workspace(root_dir, upload_id)
                            if freed:
                                tasks_evicted += 1
                                bytes_evicted += freed
                                print(f"[CACHE] evicted malformed upload cache upload_id={upload_id} bytes={freed}")
                            continue

                        # Recheck completed archives before deleting their
                        # recoverable parts; a missing/changed final ZIP turns
                        # the task back into an error task instead.
                        refresh_upload_meta(meta)
                        state = meta.get("state", "initialized")
                        if state == "completed":
                            had_chunk_list = bool(meta.get("uploaded_chunks"))
                            freed = cleanup_chunk_workspace(meta)
                            if freed or had_chunk_list:
                                meta["uploaded_chunks"] = []
                                meta["uploaded_chunk_count"] = int(meta["total_chunks"])
                                meta["bytes_received"] = int(meta["total_size"])
                                if freed:
                                    meta["cache_evicted_at"] = now
                                    meta["cache_evicted_bytes"] = int(
                                        meta.get("cache_evicted_bytes", 0)
                                    ) + freed
                                    meta["cache_eviction_count"] = int(
                                        meta.get("cache_eviction_count", 0)
                                    ) + 1
                                save_upload_meta(meta)
                            if freed:
                                tasks_evicted += 1
                                bytes_evicted += freed
                            continue

                        try:
                            updated_at = float(meta.get("updated_at", meta.get("created_at", 0)))
                        except (TypeError, ValueError):
                            updated_at = 0.0
                        if now - updated_at < UPLOAD_CACHE_TTL_S:
                            continue
                        freed = cleanup_chunk_workspace(meta)
                        meta["state"] = "initialized"
                        meta["uploaded_chunks"] = []
                        meta["uploaded_chunk_count"] = 0
                        meta["bytes_received"] = 0
                        meta["cache_evicted_at"] = now
                        meta["cache_evicted_bytes"] = int(
                            meta.get("cache_evicted_bytes", 0)
                        ) + freed
                        meta["cache_eviction_count"] = int(
                            meta.get("cache_eviction_count", 0)
                        ) + 1
                        meta["last_error"] = (
                            "inactive NAS chunk cache evicted; client may resume by re-uploading"
                        )
                        save_upload_meta(meta)
                        tasks_evicted += 1
                        bytes_evicted += freed
                        print(f"[CACHE] evicted inactive upload cache upload_id={upload_id} bytes={freed}")
            return {
                "tasks_evicted": tasks_evicted,
                "bytes_evicted": bytes_evicted,
                "lock_busy": False,
            }
        finally:
            fcntl.flock(cleanup_lock.fileno(), fcntl.LOCK_UN)


def _configured_upload_roots():
    return sorted(set([DEFAULT_ROOT_DIR, *CLIENT_ROOT_DIR_MAP.values()]))


def _upload_cache_cleanup_loop():
    while True:
        for root_dir in _configured_upload_roots():
            try:
                result = cleanup_upload_cache_root(root_dir)
                if result.get("tasks_evicted"):
                    print(f"[CACHE] root={root_dir} result={result}")
            except Exception as exc:
                print(f"[ERROR] upload cache cleanup failed root={root_dir}: {exc}")
        time.sleep(UPLOAD_CACHE_CLEANUP_INTERVAL_S)


def start_upload_cache_cleanup_once():
    global _upload_cache_cleanup_started
    global _upload_cache_cleanup_thread
    global _upload_cache_cleanup_pid
    with _upload_cache_cleanup_start_lock:
        current_pid = os.getpid()
        if (_upload_cache_cleanup_started and
                _upload_cache_cleanup_pid == current_pid and
                _upload_cache_cleanup_thread is not None and
                _upload_cache_cleanup_thread.is_alive()):
            return
        thread = threading.Thread(
            target=_upload_cache_cleanup_loop,
            daemon=True,
            name="upload-cache-cleaner",
        )
        thread.start()
        _upload_cache_cleanup_thread = thread
        _upload_cache_cleanup_pid = current_pid
        _upload_cache_cleanup_started = True


def iter_upload_meta_paths(root_dir: str):
    root = os.path.join(root_dir, CHUNKED_DIRNAME)
    if not os.path.isdir(root):
        return []
    results = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "meta.json")
        if os.path.isfile(path):
            results.append(path)
    return results


def create_app():
    start_image_workers_once()
    start_upload_cache_cleanup_once()
    app = Flask(__name__)

    @app.before_request
    def ensure_upload_cache_cleaner_for_worker():
        # In a preloaded multi-process WSGI server, threads created before fork
        # do not survive in workers. The PID/liveness check restarts one safely.
        start_upload_cache_cleanup_once()

    @app.route('/state_data', methods=['POST'])
    def handle_rtk_data():
        client_ip, root_dir, session_ts_sec, error_response = resolve_request_session_context()
        if error_response is not None:
            return error_response, 400
        if session_ts_sec is None:
            return "OK", 200

        data = request.get_json(silent=True)
        if data is None or not isinstance(data, dict) or not data:
            return jsonify({"error": "invalid json"}), 400

        added = 0
        total_written = 0
        with _lock:
            session = ensure_session_state_locked(client_ip, root_dir, session_ts_sec)
            for key, value in data.items():
                if key in session.seen_keys:
                    continue
                session.json_file.write(json.dumps({key: value}, ensure_ascii=False))
                session.json_file.write("\n")
                session.seen_keys.add(key)
                session.frames_written += 1
                session.pending_flush_count += 1
                added += 1
            flush_session_locked(session)
            total_written = session.frames_written

        if added:
            print(f"[{client_ip}] appended {added} state frame(s). session={session_ts_sec}, total={total_written}")
        return "OK", 200

    def enqueue_image_batch(kind: str):
        client_ip, root_dir, session_ts_sec, error_response = resolve_request_session_context()
        if error_response is not None:
            return error_response, 400
        if session_ts_sec is None:
            return "OK", 200

        data = request.get_json(silent=True)
        if data is None or not isinstance(data, dict) or not data:
            return jsonify({"error": "invalid json"}), 400

        try:
            _image_save_q.put_nowait((kind, client_ip, root_dir, session_ts_sec, data))
        except Full:
            return jsonify({"error": "NAS image queue full"}), 503
        return "OK", 200

    @app.route('/gridmap_data', methods=['POST'])
    def handle_gridmap_data():
        return enqueue_image_batch("gridmap")

    @app.route('/rangepic_data', methods=['POST'])
    def handle_rangepic_data():
        return enqueue_image_batch("rangepic")

    @app.route('/camera0_data', methods=['POST'])
    def handle_camera0_data():
        return enqueue_image_batch("camera_0")

    @app.route('/camera1_data', methods=['POST'])
    def handle_camera1_data():
        return enqueue_image_batch("camera_1")

    @app.route('/mirror_file', methods=['POST'])
    def mirror_file():
        upload = request.files.get("file")
        if upload is None or upload.filename == "":
            return jsonify({"error": "file is required"}), 400

        client_ip, root_dir = resolve_root_dir_for_request()
        dataset_root = request.form.get("dataset_root", "")
        relative_path = request.form.get("relative_path", "")
        source_mtime_raw = request.form.get("source_mtime_ms", "")

        try:
            ensure_dir(root_dir)
            _, final_path, normalized_relative_path = build_mirror_target(root_dir, dataset_root, relative_path)
            source_mtime_ms = parse_source_mtime_ms(source_mtime_raw)
            save_upload_atomically(upload, final_path)
            apply_source_mtime(final_path, source_mtime_ms)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            print(
                f"[ERROR] mirror_file save failed: client_ip={client_ip}, root_dir={root_dir}, "
                f"dataset_root={dataset_root}, relative_path={relative_path}, error={exc}"
            )
            return jsonify({"error": f"save failed: {exc}"}), 500

        return jsonify({
            "ok": True,
            "saved_path": final_path,
            "dataset_root": dataset_root,
            "relative_path": normalized_relative_path,
        }), 200

    @app.route('/mirror_tar', methods=['POST'])
    def mirror_tar():
        upload = request.files.get("file")
        if upload is None or upload.filename == "":
            return jsonify({"error": "file is required"}), 400

        client_ip, root_dir = resolve_root_dir_for_request()
        dataset_root = request.form.get("dataset_root", "")
        archive_kind_raw = request.form.get("archive_kind", "")
        source_files_raw = request.form.get("source_files_json", "")
        archive_path = None

        try:
            dataset_root = validate_dataset_root(dataset_root)
            archive_kind = parse_archive_kind(archive_kind_raw)
            if archive_kind == "session_zip":
                raise ValueError("session_zip must use the chunked upload endpoints")
            expected_paths = parse_source_files_value(source_files_raw, dataset_root)
            archive_path = incoming_archive_path(root_dir, dataset_root, archive_kind, upload.filename)
            save_upload_atomically(upload, archive_path)
            saved_paths = extract_tar_archive(archive_path, root_dir, dataset_root, expected_paths)
        except ValueError as exc:
            remove_file_if_exists(archive_path or "")
            print(
                f"[WARN] mirror_tar bad request: client_ip={client_ip}, root_dir={root_dir}, error={exc}, "
                f"dataset_root={dataset_root}, archive_kind={archive_kind_raw}, "
                f"filename={getattr(upload, 'filename', '')}"
            )
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            remove_file_if_exists(archive_path or "")
            print(
                f"[ERROR] mirror_tar failed: client_ip={client_ip}, root_dir={root_dir}, "
                f"dataset_root={dataset_root}, archive_kind={archive_kind_raw}, error={exc}"
            )
            return jsonify({"error": f"archive save failed: {exc}"}), 500
        finally:
            remove_file_if_exists(archive_path or "")

        return jsonify({
            "ok": True,
            "dataset_root": dataset_root,
            "archive_kind": archive_kind,
            "files_saved": len(saved_paths),
            "saved_paths": saved_paths,
        }), 200

    @app.route('/upload_init', methods=['POST'])
    def upload_init():
        client_ip, root_dir = resolve_root_dir_for_request()
        try:
            ensure_dir(os.path.join(root_dir, CHUNKED_DIRNAME))
            payload = parse_request_payload()
            upload_id = validate_upload_id(payload.get("upload_id"))
            with upload_meta_lock(root_dir, upload_id):
                meta = ensure_upload_meta(payload, root_dir, client_ip)
        except UploadConflictError as exc:
            return jsonify({"error": str(exc)}), 409
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            print(f"[ERROR] upload_init failed: {exc}")
            return jsonify({"error": f"upload_init failed: {exc}"}), 500
        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=True)}), 200

    @app.route('/upload_chunk', methods=['POST'])
    def upload_chunk():
        upload = request.files.get("file") or request.files.get("chunk")
        if upload is None or upload.filename == "":
            return jsonify({"error": "file chunk is required"}), 400

        client_ip, root_dir = resolve_root_dir_for_request()
        try:
            upload_id = validate_upload_id(request.form.get("upload_id", ""))
            chunk_index = parse_positive_int("chunk_index", request.form.get("chunk_index"), minimum=0)
            # Shared data locks preserve parallel chunk uploads while preventing
            # the cache cleaner/finalizer from removing parts being written.
            with upload_data_lock(root_dir, upload_id, shared=True):
                with upload_meta_lock(root_dir, upload_id):
                    meta = load_upload_meta(root_dir, upload_id, client_ip)
                    if meta is None:
                        raise ValueError("upload_id not initialized")
                    refresh_upload_meta(meta)
                    if meta.get("state") == "completed":
                        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=False)}), 200
                    expected_size = expected_chunk_size(meta, chunk_index)
                save_chunk_atomically(upload, chunk_part_path(root_dir, upload_id, chunk_index), expected_size)
                with upload_meta_lock(root_dir, upload_id):
                    meta = load_upload_meta(root_dir, upload_id, client_ip)
                    if meta is None:
                        raise ValueError("upload_id metadata disappeared")
                    refresh_upload_meta(meta)
                    if meta.get("state") == "completed":
                        cleanup_chunk_workspace(meta)
                    meta["last_error"] = None
                    save_upload_meta(meta)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            if 'meta' in locals() and meta is not None:
                with upload_meta_lock(root_dir, meta["upload_id"]):
                    latest = load_upload_meta(root_dir, meta["upload_id"], client_ip) or meta
                    if latest.get("state") == "completed":
                        return jsonify({
                            "ok": True,
                            **upload_status_payload(latest, include_chunks=False),
                        }), 200
                    latest["state"] = "error"
                    latest["last_error"] = str(exc)
                    save_upload_meta(latest)
            print(f"[ERROR] upload_chunk failed: {exc}")
            return jsonify({"error": f"upload_chunk failed: {exc}"}), 500

        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=False)}), 200

    @app.route('/upload_complete', methods=['POST'])
    def upload_complete():
        client_ip, root_dir = resolve_root_dir_for_request()
        payload = parse_request_payload()
        try:
            upload_id = validate_upload_id(payload.get("upload_id", ""))
            # Completion needs an exclusive data lock so no chunk writer or
            # cleanup pass can race archive assembly and final verification.
            with upload_data_lock(root_dir, upload_id, shared=False):
                with upload_meta_lock(root_dir, upload_id):
                    meta = load_upload_meta(root_dir, upload_id, client_ip)
                    if meta is None:
                        raise ValueError("upload_id not initialized")
                    refresh_upload_meta(meta)
                    # Mark this completion request as activity before potentially
                    # expensive assembly and ZIP validation.
                    save_upload_meta(meta)
                    if meta.get("state") == "completed":
                        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=True)}), 200
                    if meta.get("uploaded_chunk_count") != int(meta["total_chunks"]):
                        raise UploadConflictError("upload is incomplete")

                    archive_path = assemble_archive_from_parts(meta)
                    if meta["archive_kind"] == "session_zip":
                        finalize_session_zip(meta, archive_path)
                    else:
                        saved_paths = extract_tar_archive(
                            archive_path,
                            meta.get("root_dir", root_dir),
                            meta["dataset_root"],
                            meta["source_files"],
                        )
                        meta["state"] = "completed"
                        meta["files_saved"] = len(saved_paths)
                        meta["saved_paths"] = saved_paths
                        meta["uploaded_chunk_count"] = int(meta["total_chunks"])
                        meta["bytes_received"] = int(meta["total_size"])
                        meta["last_error"] = None
                    # Keep only aggregate completion information; the complete
                    # chunk index list can be very large and is no longer useful.
                    meta["uploaded_chunks"] = []
                    save_upload_meta(meta)
                    cleanup_chunk_workspace(meta)
        except UploadConflictError as exc:
            if 'meta' in locals() and meta is not None:
                with upload_meta_lock(root_dir, meta["upload_id"]):
                    latest = load_upload_meta(root_dir, meta["upload_id"], client_ip) or meta
                    refresh_upload_meta(latest)
                    if latest.get("state") != "completed":
                        latest["last_error"] = str(exc)
                    save_upload_meta(latest)
            return jsonify({"error": str(exc)}), 409
        except ValueError as exc:
            if 'meta' in locals() and meta is not None:
                with upload_meta_lock(root_dir, meta["upload_id"]):
                    latest = load_upload_meta(root_dir, meta["upload_id"], client_ip) or meta
                    latest["state"] = "error"
                    latest["last_error"] = str(exc)
                    save_upload_meta(latest)
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            if 'meta' in locals() and meta is not None:
                with upload_meta_lock(root_dir, meta["upload_id"]):
                    latest = load_upload_meta(root_dir, meta["upload_id"], client_ip) or meta
                    latest["state"] = "error"
                    latest["last_error"] = str(exc)
                    save_upload_meta(latest)
            print(f"[ERROR] upload_complete failed: {exc}")
            return jsonify({"error": f"upload_complete failed: {exc}"}), 500

        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=True)}), 200

    @app.route('/upload_status', methods=['GET'])
    def upload_status_list():
        client_ip, root_dir = resolve_root_dir_for_request()
        items = []
        for meta_path in iter_upload_meta_paths(root_dir):
            try:
                upload_id = validate_upload_id(os.path.basename(os.path.dirname(meta_path)))
                with upload_meta_lock(root_dir, upload_id):
                    meta = load_upload_meta(root_dir, upload_id, client_ip)
                    if meta is None:
                        continue
                    refresh_upload_meta(meta)
                    save_upload_meta(meta)
                    items.append(upload_status_payload(meta, include_chunks=False))
            except Exception as exc:
                items.append({"meta_path": meta_path, "error": str(exc)})
        items.sort(key=lambda item: item.get("updated_at", 0), reverse=True)
        return jsonify({"ok": True, "tasks": items}), 200

    @app.route('/upload_status/<upload_id>', methods=['GET'])
    def upload_status_detail(upload_id):
        client_ip, root_dir = resolve_root_dir_for_request()
        try:
            upload_id = validate_upload_id(upload_id)
            with upload_meta_lock(root_dir, upload_id):
                meta = load_upload_meta(root_dir, upload_id, client_ip)
                if meta is None:
                    return jsonify({"error": "upload_id not found"}), 404
                refresh_upload_meta(meta)
                save_upload_meta(meta)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:
            print(f"[ERROR] upload_status detail failed: {exc}")
            return jsonify({"error": f"upload_status failed: {exc}"}), 500
        return jsonify({"ok": True, **upload_status_payload(meta, include_chunks=True)}), 200

    @app.route('/save_signal', methods=['POST'])
    def save_signal():
        body = request.get_json(silent=True) or {}
        value = body.get("save_signal", body.get("value", body.get("data", None)))
        if value not in (0, 1, "0", "1"):
            return jsonify({"error": "save_signal must be 0 or 1"}), 400
        signal = int(value)

        response_session = None
        saved_path = None

        client_ip, root_dir = resolve_root_dir_for_request()
        with _lock:
            last_signal = _last_signal_by_client.get(client_ip, 0)
            active_session_ts_sec = _active_session_ts_sec_by_client.get(client_ip)

            if last_signal == 0 and signal == 1:
                host_ts_sec = body.get("host_ts_sec")
                try:
                    host_ts_sec = int(host_ts_sec)
                except (TypeError, ValueError):
                    host_ts_sec = int(time.time())

                response_session = ensure_session_state_locked(client_ip, root_dir, host_ts_sec)
                _active_session_ts_sec_by_client[client_ip] = host_ts_sec
                _recording_by_client[client_ip] = True
                print(
                    f"[{client_ip}] Recording started. session_ts_sec={host_ts_sec}, "
                    f"json_tmp={response_session.json_tmp_path}"
                )

            elif last_signal == 1 and signal == 0:
                previous_session_ts = active_session_ts_sec
                _recording_by_client[client_ip] = False
                _active_session_ts_sec_by_client[client_ip] = None
                if previous_session_ts is not None:
                    response_session = ensure_session_state_locked(client_ip, root_dir, previous_session_ts)
                    saved_path = close_session_locked(response_session)
                    print(
                        f"[{client_ip}] Recording stopped. session_ts_sec={previous_session_ts}, "
                        f"frames_written={response_session.frames_written}"
                    )

            elif active_session_ts_sec is not None:
                response_session = ensure_session_state_locked(client_ip, root_dir, active_session_ts_sec)

            _last_signal_by_client[client_ip] = signal
            recording = _recording_by_client.get(client_ip, False)

        payload = session_response_payload(response_session)
        return jsonify({
            "recording": recording,
            "gray_recording": recording,
            "gray_session_dir": payload["gray_session_dir"],
            "range_recording": recording,
            "range_session_dir": payload["range_session_dir"],
            "camera_recording": recording,
            "camera_dirs": payload["camera_dirs"],
            "session_root": payload["session_root"],
            "session_ts_sec": payload["session_ts_sec"],
            "dataset_root": payload["dataset_root"],
            "json_path": payload["json_path"],
            "saved": bool(saved_path),
            "path": saved_path,
        }), 200

    @app.route('/')
    def hello():
        return "Sany Hello"

    return app


if __name__ == '__main__':
    app = create_app()
    app.run(host='0.0.0.0', port=8082, threaded=True)
