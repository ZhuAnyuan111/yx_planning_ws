/*
项目                           修改人     修改日期       改动
无人挖机——NAS数采+大脑通信      wuql17     2026/07/03    基于20260629的python程序改C++（新增程序启动代码版本输入;新增控制、轨迹融合、vla轨迹订阅；修改数采文件夹保存逻辑；增加大脑手柄信号保存逻辑）
无人挖机——NAS数采+大脑通信      wuql17     2026/07/06    移除opencv,换gstreamer拉流，libpng处理图片
无人挖机——NAS数采+大脑通信      wuql17     2026/07/07    改代码逻辑，修复摄像头数据保存频率低的问题
无人挖机——NAS数采+大脑通信      wuql17     2026/07/09    修复摄像头功能安全误报问题
无人挖机——NAS数采+大脑通信      wuql17     2026/07/12    解决NAS数据上传问题
无人挖机——NAS数采+大脑通信      wuql17     2026/08/10    修改数据保存及上传逻辑：先质检、再打包上传NAS、存log、删除本地文件
无人挖机——NAS数采+大脑通信      wuql17     2026/08/12    将数据质检压缩和上传放到两个不同的线程里面，避免因为数据没有上传完成导致新数据不质检
无人挖机——NAS数采+大脑通信      wuql17     2026/09/02    smolvla_status_check话题的值从0、1变为0~6,表示大脑通信故障的具体问题
无人挖机——NAS数采+大脑通信      wuql17     2026/09/09    删除已完成上传的历史日期空目录及capture_upload.log，当天目录保留到次日清理





*/

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <grid_map_msgs/GridMap.h>
#include <interfaces/JointAngles.h>
#include <interfaces/SinglePoint.h>
#include <interfaces/imu_syd.h>
#include <interfaces/rtk_cgi610.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <cstdio>
#include <curl/curl.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <jpeglib.h>
#include <jsoncpp/json/json.h>
#include <png.h>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <setjmp.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace {

const std::regex kDatasetRe("^excavator_data_[0-9]{8}$");
const std::regex kSessionDirRe("^[0-9]{14}$");
const std::regex kDamagedArchiveRe("^([0-9]{14})_Damaged\\.zip$");
const char* kDefaultScanRoot = "/media/nvidia/G500/DataCapture";
const char* kDefaultBrainBaseUrl = "http://192.168.1.140:8082";
const char* kDefaultNasBaseUrl = "http://192.168.1.248:8082";
const char* kTmpRootDirname = ".sy200_upload_tmp";
const char* kStatusRootDirname = ".sy200_upload_status";
const char* kDefaultZipBinary = "/usr/bin/zip";
const char* kDefaultUnzipBinary = "/usr/bin/unzip";
const char* kDefaultRawVideoParseFormat = "4";
constexpr int kDefaultCameraInputWidth = 1920;
constexpr int kDefaultCameraInputHeight = 1080;

constexpr uint64_t kDefaultChunkSizeBytes = 8ULL * 1024ULL * 1024ULL;
constexpr uint64_t kDefaultMaxScanRootBytes = 100ULL * 1024ULL * 1024ULL * 1024ULL;
constexpr uint64_t kMinFreeBytes = 20ULL * 1024ULL * 1024ULL * 1024ULL;
constexpr int kDefaultSessionRecreateFailures = 3;
constexpr double kDefaultNasConnectTimeoutS = 5.0;
constexpr double kDefaultNasRequestTimeoutS = 300.0;

std::string getenvString(const std::string& key) {
  const char* value = std::getenv(key.c_str());
  return value ? std::string(value) : std::string();
}

std::string joinUrl(std::string base, std::string path) {
  if (path.rfind("http://", 0) == 0 || path.rfind("https://", 0) == 0) {
    return path;
  }
  if (base.empty()) {
    return path;
  }
  if (base.back() == '/' && !path.empty() && path.front() == '/') {
    return base + path.substr(1);
  }
  if (base.back() != '/' && (path.empty() || path.front() != '/')) {
    return base + "/" + path;
  }
  return base + path;
}

bool startsWith(const std::string& text, const std::string& prefix) {
  return text.rfind(prefix, 0) == 0;
}

bool pathExists(const std::string& path) {
  std::error_code ec;
  return fs::exists(path, ec);
}

bool isPathUnder(const fs::path& path, const fs::path& root) {
  std::error_code ec;
  fs::path normalized_path = fs::weakly_canonical(fs::absolute(path), ec);
  if (ec) normalized_path = fs::absolute(path).lexically_normal();
  ec.clear();
  fs::path normalized_root = fs::weakly_canonical(fs::absolute(root), ec);
  if (ec) normalized_root = fs::absolute(root).lexically_normal();
  auto path_it = normalized_path.begin();
  auto root_it = normalized_root.begin();
  for (; root_it != normalized_root.end(); ++root_it, ++path_it) {
    if (path_it == normalized_path.end() || *path_it != *root_it) {
      return false;
    }
  }
  return true;
}

fs::path normalizeConfiguredDirectory(const fs::path& path, const std::string& parameter_name) {
  if (path.empty()) {
    throw std::runtime_error(parameter_name + " path must not be empty");
  }
  std::error_code ec;
  fs::path absolute_path = fs::absolute(path, ec);
  if (ec) {
    throw std::runtime_error("cannot resolve " + parameter_name + " path: " + path.string() +
                             " (" + ec.message() + ")");
  }
  fs::path normalized = fs::weakly_canonical(absolute_path, ec);
  if (ec) normalized = absolute_path.lexically_normal();
  if (normalized.empty() || normalized == normalized.root_path()) {
    throw std::runtime_error(parameter_name + " must not be the filesystem root: " +
                             normalized.string());
  }
  return normalized;
}

fs::path requireDirectoryUnder(const fs::path& path, const fs::path& root,
                               const std::string& parameter_name) {
  const fs::path normalized = normalizeConfiguredDirectory(path, parameter_name);
  if (normalized == root || !isPathUnder(normalized, root)) {
    throw std::runtime_error(parameter_name + " must stay inside scan_root: " +
                             normalized.string() + " (scan_root=" + root.string() + ")");
  }
  return normalized;
}

std::string timeFormat(std::time_t ts, const char* fmt) {
  std::tm tm{};
  localtime_r(&ts, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), fmt, &tm);
  return std::string(buf);
}

double monotonicSeconds() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

int64_t unixMs() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch()).count();
}

void ensureDir(const fs::path& path) {
  if (!path.empty()) {
    std::error_code ec;
    fs::create_directories(path, ec);
  }
}

void ensureDirOrThrow(const fs::path& path) {
  if (path.empty()) throw std::runtime_error("directory path is empty");
  std::error_code ec;
  fs::create_directories(path, ec);
  if (ec || !fs::is_directory(path, ec) || ::access(path.c_str(), R_OK | W_OK | X_OK) != 0) {
    throw std::runtime_error("cannot create/access directory: " + path.string() +
                             (ec ? " (" + ec.message() + ")" :
                                   " (read/write/search access is required)"));
  }
}

uint64_t directoryLogicalSize(const fs::path& root, bool* ok = nullptr) {
  uint64_t total = 0;
  bool success = true;
  std::error_code ec;
  if (!fs::is_directory(root, ec)) {
    if (ok) *ok = false;
    return 0;
  }
  fs::recursive_directory_iterator it(root, fs::directory_options::none, ec);
  for (; !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    std::error_code sec;
    const fs::file_status status = it->symlink_status(sec);
    if (sec) {
      success = false;
      continue;
    }
    if (fs::is_symlink(status)) {
      if (fs::is_directory(status)) it.disable_recursion_pending();
      continue;
    }
    if (!fs::is_regular_file(status)) continue;
    const uint64_t size = static_cast<uint64_t>(fs::file_size(it->path(), sec));
    if (sec || std::numeric_limits<uint64_t>::max() - total < size) {
      success = false;
      continue;
    }
    total += size;
  }
  if (ec) success = false;
  if (ok) *ok = success;
  return total;
}

void removeFileIfExists(const fs::path& path) {
  std::error_code ec;
  if (!path.empty()) {
    fs::remove(path, ec);
  }
}

uint64_t fileSize(const fs::path& path) {
  std::error_code ec;
  auto size = fs::file_size(path, ec);
  return ec ? 0 : static_cast<uint64_t>(size);
}

std::string relativePosix(const fs::path& path, const fs::path& start) {
  std::error_code ec;
  fs::path rel = fs::relative(path, start, ec);
  if (ec) {
    rel = path.filename();
  }
  return rel.generic_string();
}

std::vector<std::pair<std::string, fs::path>> listDatasetDirs(const fs::path& scan_root) {
  std::vector<std::pair<std::string, fs::path>> out;
  std::error_code ec;
  if (!fs::is_directory(scan_root, ec)) {
    return out;
  }
  for (const auto& entry : fs::directory_iterator(scan_root, ec)) {
    if (!entry.is_directory(ec) || fs::is_symlink(entry.symlink_status(ec))) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (std::regex_match(name, kDatasetRe)) {
      out.emplace_back(name, entry.path());
    }
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  return out;
}

std::vector<std::pair<std::string, fs::path>> listSessionDirs(const fs::path& dataset_root) {
  std::vector<std::pair<std::string, fs::path>> out;
  std::error_code ec;
  if (!fs::is_directory(dataset_root, ec)) {
    return out;
  }
  for (const auto& entry : fs::directory_iterator(dataset_root, ec)) {
    if (!entry.is_directory(ec) || fs::is_symlink(entry.symlink_status(ec))) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (std::regex_match(name, kSessionDirRe)) {
      out.emplace_back(name, entry.path());
    }
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  return out;
}

std::vector<std::pair<std::string, fs::path>> listRecordingRoots(const fs::path& dataset_root) {
  auto roots = listSessionDirs(dataset_root);
  roots.insert(roots.begin(), {"", dataset_root});
  return roots;
}

double roundTo(double value, double scale) {
  if (!std::isfinite(value)) {
    return value;
  }
  return std::round(value * scale) / scale;
}

Json::Value jsonDouble(double value, double scale = 100000.0) {
  if (!std::isfinite(value)) {
    return Json::Value(Json::nullValue);
  }
  return Json::Value(roundTo(value, scale));
}

Json::Value excavatorDataJoystickJson(const Json::Value& value) {
  if (!value.isObject()) {
    return Json::Value(Json::nullValue);
  }
  Json::Value out(Json::objectValue);
  for (const std::string& key : {"swing", "arm", "bucket", "boom"}) {
    if (!value.isMember(key)) {
      return Json::Value(Json::nullValue);
    }
    double v = 0.0;
    try {
      if (value[key].isNumeric()) {
        v = value[key].asDouble();
      } else if (value[key].isString()) {
        size_t parsed = 0;
        std::string text = value[key].asString();
        v = std::stod(text, &parsed);
        if (parsed != text.size()) {
          return Json::Value(Json::nullValue);
        }
      } else {
        return Json::Value(Json::nullValue);
      }
    } catch (const std::exception&) {
      return Json::Value(Json::nullValue);
    }
    if (!std::isfinite(v)) {
      return Json::Value(Json::nullValue);
    }
    out[key] = jsonDouble(v);
  }
  return out;
}

Json::StreamWriterBuilder compactWriter() {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  builder["emitUTF8"] = true;
  return builder;
}

std::string jsonToString(const Json::Value& value) {
  static Json::StreamWriterBuilder builder = compactWriter();
  return Json::writeString(builder, value);
}

std::string formatSavedJsonNumber(double value, int decimals) {
  if (!std::isfinite(value)) {
    return "null";
  }
  const double scale = std::pow(10.0, decimals);
  double rounded = roundTo(value, scale);
  if (rounded == 0.0) {
    rounded = 0.0;
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(decimals) << rounded;
  std::string out = ss.str();
  const auto dot = out.find('.');
  if (dot != std::string::npos) {
    while (!out.empty() && out.back() == '0') {
      out.pop_back();
    }
    if (!out.empty() && out.back() == '.') {
      out.pop_back();
    }
  }
  return out == "-0" ? "0" : out;
}

std::string jsonStringLiteral(const std::string& value) {
  return jsonToString(Json::Value(value));
}

std::vector<std::string> savedJsonPreferredKeys(const Json::Value& value, const std::string& object_key) {
  if (object_key == "__save_payload__") {
    return {
        "rtk_stamp_ms",
        "rtk",
        "angular_velocity",
        "bucket_local_position",
        "bucket_rotation_local_position",
        "bucket_terrain_delta_position",
        "joints_angles",
        "joints_velocity",
        "swivel_angle",
        "Sys_SeqAction",
        "Sys_SCo_Traj",
        "fused_controller_joystick_topic",
        "smolVLA_output_angle",
        "excavator_data_joystick",
        "fault_code",
        "machine_model",
        "operation_mode",
        "location",
        "weather",
        "model_id",
        "trajectory_fusion_code_version",
        "control_code_version"};
  }
  if (object_key == "rtk") {
    return {"lat", "lon"};
  }
  if (object_key == "joints_angles") {
    return {"boom", "arm", "bucket"};
  }
  if (object_key == "excavator_data_joystick") {
    return {"swing", "arm", "bucket", "boom"};
  }
  if (value.isMember("position") || value.isMember("orientation")) {
    return {"position", "orientation"};
  }
  if (value.isMember("x") || value.isMember("y") || value.isMember("z") || value.isMember("w")) {
    return {"x", "y", "z", "w"};
  }
  if (value.isMember("data")) {
    return {"data"};
  }
  return {};
}

std::string savedJsonValueToString(const Json::Value& value, const std::string& key, const std::string& parent_key);

std::string savedJsonObjectToString(const Json::Value& value, const std::string& object_key) {
  std::vector<std::string> keys;
  std::vector<std::string> used;
  const auto preferred = savedJsonPreferredKeys(value, object_key);
  for (const auto& key : preferred) {
    if (value.isMember(key)) {
      keys.push_back(key);
      used.push_back(key);
    }
  }

  std::vector<std::string> remaining = value.getMemberNames();
  for (const auto& key : remaining) {
    if (std::find(used.begin(), used.end(), key) == used.end()) {
      keys.push_back(key);
    }
  }

  std::string out = "{";
  bool first = true;
  for (const auto& key : keys) {
    if (!first) {
      out += ",";
    }
    first = false;
    out += jsonStringLiteral(key);
    out += ":";
    out += savedJsonValueToString(value[key], key, object_key);
  }
  out += "}";
  return out;
}

std::string savedJsonValueToString(const Json::Value& value, const std::string& key, const std::string& parent_key) {
  switch (value.type()) {
    case Json::nullValue:
      return "null";
    case Json::intValue:
    case Json::uintValue:
    case Json::stringValue:
    case Json::booleanValue:
      return jsonToString(value);
    case Json::realValue: {
      const int decimals = (parent_key == "rtk" && (key == "lat" || key == "lon")) ? 8 : 5;
      return formatSavedJsonNumber(value.asDouble(), decimals);
    }
    case Json::arrayValue: {
      std::string out = "[";
      for (Json::ArrayIndex i = 0; i < value.size(); ++i) {
        if (i > 0) {
          out += ",";
        }
        out += savedJsonValueToString(value[i], "", parent_key);
      }
      out += "]";
      return out;
    }
    case Json::objectValue:
      return savedJsonObjectToString(value, key);
  }
  return "null";
}

std::string savedFrameJsonLine(const std::string& frame_key, const Json::Value& save_payload) {
  return "{" + jsonStringLiteral(frame_key) + ":" + savedJsonObjectToString(save_payload, "__save_payload__") + "}";
}

struct ImageSize {
  int width = 0;
  int height = 0;
};

struct RgbImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;

  bool empty() const {
    return width <= 0 || height <= 0 || data.size() < static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
  }
};

struct GrayImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;

  bool empty() const {
    return width <= 0 || height <= 0 || data.size() < static_cast<size_t>(width) * static_cast<size_t>(height);
  }
};

bool writeTextAtomic(const fs::path& path, const std::string& data) {
  ensureDir(path.parent_path());
  fs::path tmp = path;
  tmp += ".tmp";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) return false;
  size_t written = 0;
  while (written < data.size()) {
    const ssize_t count = ::write(fd, data.data() + written, data.size() - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      ::close(fd);
      removeFileIfExists(tmp);
      return false;
    }
    written += static_cast<size_t>(count);
  }
  const bool sync_ok = ::fsync(fd) == 0;
  const bool close_ok = ::close(fd) == 0;
  if (!sync_ok || !close_ok) {
    removeFileIfExists(tmp);
    return false;
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    removeFileIfExists(tmp);
    return false;
  }
  const int dir_fd = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd >= 0) {
    ::fsync(dir_fd);
    ::close(dir_fd);
  }
  return true;
}

bool writeBytesAtomic(const fs::path& path, const std::vector<uint8_t>& bytes) {
  ensureDir(path.parent_path());
  fs::path tmp = path;
  tmp += ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary);
    if (!out) {
      return false;
    }
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    out.flush();
    if (!out) {
      return false;
    }
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    removeFileIfExists(tmp);
    return false;
  }
  return true;
}

struct JpegErrorManager {
  jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
  char message[JMSG_LENGTH_MAX] = {0};
};

void jpegErrorExit(j_common_ptr cinfo) {
  auto* err = reinterpret_cast<JpegErrorManager*>(cinfo->err);
  (*cinfo->err->format_message)(cinfo, err->message);
  longjmp(err->setjmp_buffer, 1);
}

std::optional<std::vector<uint8_t>> encodeJpegBytes(
    const uint8_t* data,
    int width,
    int height,
    int channels,
    int quality) {
  if (!data || width <= 0 || height <= 0 || (channels != 1 && channels != 3)) {
    return std::nullopt;
  }
  jpeg_compress_struct cinfo{};
  JpegErrorManager jerr{};
  unsigned char* mem = nullptr;
  unsigned long mem_size = 0;
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpegErrorExit;
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_compress(&cinfo);
    if (mem) std::free(mem);
    return std::nullopt;
  }
  jpeg_create_compress(&cinfo);
  jpeg_mem_dest(&cinfo, &mem, &mem_size);
  cinfo.image_width = static_cast<JDIMENSION>(width);
  cinfo.image_height = static_cast<JDIMENSION>(height);
  cinfo.input_components = channels;
  cinfo.in_color_space = channels == 1 ? JCS_GRAYSCALE : JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, std::max(1, std::min(100, quality)), TRUE);
  jpeg_start_compress(&cinfo, TRUE);
  const int row_stride = width * channels;
  while (cinfo.next_scanline < cinfo.image_height) {
    JSAMPROW row = const_cast<JSAMPROW>(data + static_cast<size_t>(cinfo.next_scanline) * row_stride);
    jpeg_write_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_compress(&cinfo);
  std::vector<uint8_t> out(mem, mem + mem_size);
  jpeg_destroy_compress(&cinfo);
  std::free(mem);
  return out;
}

std::optional<std::vector<uint8_t>> encodeJpegBytes(const RgbImage& image, int quality) {
  if (image.empty()) {
    return std::nullopt;
  }
  return encodeJpegBytes(image.data.data(), image.width, image.height, 3, quality);
}

std::optional<std::vector<uint8_t>> encodeJpegBytes(const GrayImage& image, int quality) {
  if (image.empty()) {
    return std::nullopt;
  }
  return encodeJpegBytes(image.data.data(), image.width, image.height, 1, quality);
}

std::string base64Encode(const unsigned char* data, size_t len) {
  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (size_t i = 0; i < len; i += 3) {
    unsigned int n = data[i] << 16;
    if (i + 1 < len) n |= data[i + 1] << 8;
    if (i + 2 < len) n |= data[i + 2];
    out.push_back(table[(n >> 18) & 63]);
    out.push_back(table[(n >> 12) & 63]);
    out.push_back(i + 1 < len ? table[(n >> 6) & 63] : '=');
    out.push_back(i + 2 < len ? table[n & 63] : '=');
  }
  return out;
}

std::optional<std::string> encodeJpegB64(const GrayImage& image, int quality) {
  auto buf = encodeJpegBytes(image, quality);
  if (!buf) {
    return std::nullopt;
  }
  return base64Encode(buf->data(), buf->size());
}

struct PngWriteBuffer {
  std::vector<uint8_t>* out = nullptr;
};

void pngWriteCallback(png_structp png_ptr, png_bytep data, png_size_t length) {
  auto* buffer = static_cast<PngWriteBuffer*>(png_get_io_ptr(png_ptr));
  buffer->out->insert(buffer->out->end(), data, data + length);
}

void pngFlushCallback(png_structp) {}

std::optional<std::vector<uint8_t>> encodePngBytes(const GrayImage& gray) {
  if (gray.empty()) {
    return std::nullopt;
  }
  std::vector<uint8_t> out;
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr) {
    return std::nullopt;
  }
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, nullptr);
    return std::nullopt;
  }
  PngWriteBuffer buffer{&out};
  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return std::nullopt;
  }
  png_set_write_fn(png_ptr, &buffer, pngWriteCallback, pngFlushCallback);
  png_set_IHDR(
      png_ptr, info_ptr, static_cast<png_uint_32>(gray.width), static_cast<png_uint_32>(gray.height),
      8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png_ptr, info_ptr);
  for (int y = 0; y < gray.height; ++y) {
    png_write_row(png_ptr, const_cast<png_bytep>(gray.data.data() + static_cast<size_t>(y) * gray.width));
  }
  png_write_end(png_ptr, info_ptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
  return out;
}

uint16_t readU16(const uint8_t* ptr, bool big_endian) {
  if (big_endian) {
    return static_cast<uint16_t>((static_cast<uint16_t>(ptr[0]) << 8) | ptr[1]);
  }
  return static_cast<uint16_t>((static_cast<uint16_t>(ptr[1]) << 8) | ptr[0]);
}

int16_t readI16(const uint8_t* ptr, bool big_endian) {
  uint16_t raw = readU16(ptr, big_endian);
  int16_t out = 0;
  std::memcpy(&out, &raw, sizeof(out));
  return out;
}

float readF32(const uint8_t* ptr, bool big_endian) {
  uint32_t raw = 0;
  if (big_endian) {
    raw = (static_cast<uint32_t>(ptr[0]) << 24) |
          (static_cast<uint32_t>(ptr[1]) << 16) |
          (static_cast<uint32_t>(ptr[2]) << 8) |
          static_cast<uint32_t>(ptr[3]);
  } else {
    raw = (static_cast<uint32_t>(ptr[3]) << 24) |
          (static_cast<uint32_t>(ptr[2]) << 16) |
          (static_cast<uint32_t>(ptr[1]) << 8) |
          static_cast<uint32_t>(ptr[0]);
  }
  float out = 0.0f;
  std::memcpy(&out, &raw, sizeof(out));
  return out;
}

uint8_t normalizeToU8(double value, double min_value, double max_value) {
  if (!std::isfinite(value) || max_value <= min_value) {
    return 0;
  }
  double scaled = (value - min_value) * 255.0 / (max_value - min_value);
  scaled = std::max(0.0, std::min(255.0, scaled));
  return static_cast<uint8_t>(std::lround(scaled));
}

GrayImage makeGrayImage(int width, int height, uint8_t fill = 0) {
  GrayImage image;
  if (width <= 0 || height <= 0) {
    return image;
  }
  image.width = width;
  image.height = height;
  image.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), fill);
  return image;
}

GrayImage resizeNearest(const GrayImage& src, int target_w, int target_h) {
  GrayImage out = makeGrayImage(target_w, target_h, 0);
  if (src.empty() || out.empty()) {
    return GrayImage{};
  }
  for (int y = 0; y < target_h; ++y) {
    int sy = std::min(src.height - 1, static_cast<int>((static_cast<int64_t>(y) * src.height) / target_h));
    for (int x = 0; x < target_w; ++x) {
      int sx = std::min(src.width - 1, static_cast<int>((static_cast<int64_t>(x) * src.width) / target_w));
      out.data[static_cast<size_t>(y) * target_w + x] = src.data[static_cast<size_t>(sy) * src.width + sx];
    }
  }
  return out;
}

void pasteGray(const GrayImage& src, GrayImage* dst, int x0, int y0) {
  if (!dst || src.empty() || dst->empty()) {
    return;
  }
  for (int y = 0; y < src.height; ++y) {
    int dy = y0 + y;
    if (dy < 0 || dy >= dst->height) continue;
    for (int x = 0; x < src.width; ++x) {
      int dx = x0 + x;
      if (dx < 0 || dx >= dst->width) continue;
      dst->data[static_cast<size_t>(dy) * dst->width + dx] = src.data[static_cast<size_t>(y) * src.width + x];
    }
  }
}

Json::Value pointJson(double x, double y, double z) {
  Json::Value out(Json::objectValue);
  out["x"] = jsonDouble(x);
  out["y"] = jsonDouble(y);
  out["z"] = jsonDouble(z);
  return out;
}

Json::Value pointJson(const geometry_msgs::Point& p) {
  return pointJson(p.x, p.y, p.z);
}

Json::Value quaternionJson(const geometry_msgs::Quaternion& q) {
  Json::Value out(Json::objectValue);
  out["x"] = jsonDouble(q.x);
  out["y"] = jsonDouble(q.y);
  out["z"] = jsonDouble(q.z);
  out["w"] = jsonDouble(q.w);
  return out;
}

Json::Value poseJson(const geometry_msgs::Pose& p) {
  Json::Value out(Json::objectValue);
  out["position"] = pointJson(p.position);
  out["orientation"] = quaternionJson(p.orientation);
  return out;
}

Json::Value nullValue() {
  return Json::Value(Json::nullValue);
}

uint32_t packSafety(uint8_t error_code, uint8_t counter) {
  const uint32_t error_part = static_cast<uint32_t>(error_code) & 0xFFFF;
  const uint32_t counter_part = static_cast<uint32_t>(counter) & 0xFF;
  return (counter_part << 16) | error_part;
}

size_t curlWriteString(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* out = static_cast<std::string*>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

struct HttpResponse {
  bool ok = false;
  long status = 0;
  std::string body;
  std::string error;
};

HttpResponse httpPostJson(const std::string& url, const std::string& body, double timeout_s) {
  HttpResponse result;
  CURL* curl = curl_easy_init();
  if (!curl) {
    result.error = "curl_easy_init failed";
    return result;
  }
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  char errbuf[CURL_ERROR_SIZE] = {0};
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.data());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_s * 1000.0));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &result.body);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
  CURLcode code = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &result.status);
  if (code == CURLE_OK) {
    result.ok = result.status >= 200 && result.status < 300;
  } else {
    result.error = errbuf[0] ? errbuf : curl_easy_strerror(code);
  }
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return result;
}

class NasHttpSession {
 public:
  NasHttpSession() { recreate(); }
  ~NasHttpSession() {
    if (curl_) curl_easy_cleanup(curl_);
  }

  NasHttpSession(const NasHttpSession&) = delete;
  NasHttpSession& operator=(const NasHttpSession&) = delete;

  void recreate() {
    if (curl_) curl_easy_cleanup(curl_);
    curl_ = curl_easy_init();
  }

  HttpResponse postJson(const std::string& url, const std::string& body,
                        double connect_timeout_s, double request_timeout_s) {
    HttpResponse result;
    if (!curl_) {
      result.error = "curl_easy_init failed";
      return result;
    }
    curl_easy_reset(curl_);
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    char errbuf[CURL_ERROR_SIZE] = {0};
    setCommonOptions(url, connect_timeout_s, request_timeout_s, &result, errbuf);
    curl_easy_setopt(curl_, CURLOPT_POST, 1L);
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, body.data());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    const CURLcode code = curl_easy_perform(curl_);
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &result.status);
    finishResponse(code, errbuf, &result);
    curl_slist_free_all(headers);
    return result;
  }

  HttpResponse getJson(const std::string& url, double connect_timeout_s,
                       double request_timeout_s) {
    HttpResponse result;
    if (!curl_) {
      result.error = "curl_easy_init failed";
      return result;
    }
    curl_easy_reset(curl_);
    char errbuf[CURL_ERROR_SIZE] = {0};
    setCommonOptions(url, connect_timeout_s, request_timeout_s, &result, errbuf);
    curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1L);
    const CURLcode code = curl_easy_perform(curl_);
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &result.status);
    finishResponse(code, errbuf, &result);
    return result;
  }

  HttpResponse postMultipartChunk(const std::string& url,
                                  const std::string& upload_id,
                                  uint64_t chunk_index,
                                  const std::string& filename,
                                  const std::vector<char>& bytes,
                                  double connect_timeout_s,
                                  double request_timeout_s) {
    HttpResponse result;
    if (!curl_) {
      result.error = "curl_easy_init failed";
      return result;
    }
    curl_easy_reset(curl_);
    curl_mime* mime = curl_mime_init(curl_);
    if (!mime) {
      result.error = "curl_mime_init failed";
      return result;
    }
    curl_mimepart* part = curl_mime_addpart(mime);
    curl_mime_name(part, "upload_id");
    curl_mime_data(part, upload_id.c_str(), CURL_ZERO_TERMINATED);
    const std::string chunk_index_str = std::to_string(chunk_index);
    part = curl_mime_addpart(mime);
    curl_mime_name(part, "chunk_index");
    curl_mime_data(part, chunk_index_str.c_str(), CURL_ZERO_TERMINATED);
    part = curl_mime_addpart(mime);
    curl_mime_name(part, "file");
    curl_mime_filename(part, filename.c_str());
    curl_mime_type(part, "application/octet-stream");
    curl_mime_data(part, bytes.data(), bytes.size());

    char errbuf[CURL_ERROR_SIZE] = {0};
    setCommonOptions(url, connect_timeout_s, request_timeout_s, &result, errbuf);
    curl_easy_setopt(curl_, CURLOPT_MIMEPOST, mime);
    const CURLcode code = curl_easy_perform(curl_);
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &result.status);
    finishResponse(code, errbuf, &result);
    curl_mime_free(mime);
    return result;
  }

 private:
  void setCommonOptions(const std::string& url, double connect_timeout_s,
                        double request_timeout_s, HttpResponse* result,
                        char* errbuf) {
    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT_MS,
                     static_cast<long>(connect_timeout_s * 1000.0));
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT_MS,
                     static_cast<long>(request_timeout_s * 1000.0));
    curl_easy_setopt(curl_, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl_, CURLOPT_NOPROXY, "*");
    curl_easy_setopt(curl_, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, curlWriteString);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &result->body);
    curl_easy_setopt(curl_, CURLOPT_ERRORBUFFER, errbuf);
  }

  static void finishResponse(CURLcode code, const char* errbuf,
                             HttpResponse* result) {
    if (code == CURLE_OK) {
      result->ok = result->status == 200;
    } else {
      result->error = errbuf[0] ? errbuf : curl_easy_strerror(code);
    }
  }

  CURL* curl_ = nullptr;
};

bool parseJson(const std::string& text, Json::Value* out) {
  Json::CharReaderBuilder builder;
  std::string errs;
  std::istringstream in(text);
  return Json::parseFromStream(builder, in, out, &errs);
}

bool readJsonFile(const fs::path& path, Json::Value* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  Json::CharReaderBuilder builder;
  std::string errors;
  return Json::parseFromStream(builder, in, out, &errors);
}

std::string httpResponseError(const HttpResponse& response) {
  if (!response.error.empty()) return response.error;
  Json::Value body;
  if (parseJson(response.body, &body) && body.isObject() && body.isMember("error")) {
    return body["error"].isString() ? body["error"].asString() : jsonToString(body["error"]);
  }
  if (!response.body.empty()) return response.body;
  return "HTTP " + std::to_string(response.status);
}

std::string zeroPadded(uint64_t value, int width) {
  std::ostringstream out;
  out << std::setw(width) << std::setfill('0') << value;
  return out.str();
}

std::string promptText(const std::string& title, const std::string& default_value, int timeout_s) {
  if (!isatty(fileno(stdin))) {
    ROS_INFO("%s 未检测到交互输入，默认使用: %s", title.c_str(), default_value.c_str());
    return default_value;
  }
  std::cout << title << "（" << timeout_s << " 秒无输入默认 " << default_value << "）: " << std::flush;
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  timeval tv{};
  tv.tv_sec = timeout_s;
  int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    std::cout << std::endl;
    ROS_INFO("%s 超时未输入，默认使用: %s", title.c_str(), default_value.c_str());
    return default_value;
  }
  std::string line;
  std::getline(std::cin, line);
  if (line.empty()) {
    ROS_INFO("%s 输入为空，默认使用: %s", title.c_str(), default_value.c_str());
    return default_value;
  }
  return line;
}

std::string promptSelection(
    const std::string& title,
    const std::vector<std::string>& options,
    const std::string& default_value,
    int timeout_s = 5) {
  if (!isatty(fileno(stdin))) {
    ROS_INFO("%s 未检测到交互输入，默认使用: %s", title.c_str(), default_value.c_str());
    return default_value;
  }
  std::cout << title << std::endl;
  for (size_t i = 0; i < options.size(); ++i) {
    std::cout << "  " << (i + 1) << ". " << (options[i] == "其他" ? "其他（请输入）" : options[i]) << std::endl;
  }
  std::cout << "请输入序号或选项内容（" << timeout_s << " 秒无输入默认 " << default_value << "）: " << std::flush;
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  timeval tv{};
  tv.tv_sec = timeout_s;
  int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    std::cout << std::endl;
    ROS_INFO("%s 超时未选择，默认使用: %s", title.c_str(), default_value.c_str());
    return default_value;
  }
  std::string line;
  std::getline(std::cin, line);
  if (line.empty()) {
    return default_value;
  }
  try {
    int idx = std::stoi(line);
    if (idx >= 1 && idx <= static_cast<int>(options.size())) {
      if (options[idx - 1] != "其他") {
        return options[idx - 1];
      }
    }
  } catch (...) {
    for (const auto& option : options) {
      if (line == option || line == "其他（请输入）") {
        if (option != "其他") {
          return option;
        }
        break;
      }
    }
    if (std::find(options.begin(), options.end(), line) == options.end() && line != "其他（请输入）") {
      ROS_WARN("%s 输入无效，默认使用: %s", title.c_str(), default_value.c_str());
      return default_value;
    }
  }
  return promptText("请输入其他内容", default_value, timeout_s);
}

}  // namespace

class StateJsonWriter {
 public:
  void beginSession(int session_id, const fs::path& bag_dir, const fs::path& tmp_path, const fs::path& final_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    finishLocked();
    session_id_ = session_id;
    bag_dir_ = bag_dir;
    tmp_path_ = tmp_path;
    final_path_ = final_path;
    lines_written_ = 0;
    ensureDir(bag_dir_);
    file_.open(tmp_path_, std::ios::out | std::ios::app);
    if (!file_) {
      ROS_ERROR("open state json tmp failed: %s", tmp_path_.c_str());
    }
  }

  void append(int session_id, const std::string& line) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_id != session_id_ || !file_) {
      return;
    }
    file_ << line << "\n";
    ++lines_written_;
    if (lines_written_ % flush_every_ == 0) {
      file_.flush();
    }
  }

  void snapshot(int session_id, const fs::path& tmp_path, const fs::path& final_path, const fs::path& bag_dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_id != session_id_ || tmp_path != tmp_path_ || !file_) {
      return;
    }
    file_.flush();
    ensureDir(bag_dir);
    std::error_code ec;
    fs::copy_file(tmp_path, final_path, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      ROS_WARN_THROTTLE(5.0, "copy bag snapshot failed: %s", ec.message().c_str());
    }
  }

  void finishSession(int session_id, const fs::path& tmp_path, const fs::path& final_path, const fs::path& bag_dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_id != session_id_ || tmp_path != tmp_path_) {
      return;
    }
    ensureDir(bag_dir);
    finishLocked();
    std::error_code ec;
    if (fs::exists(tmp_path, ec)) {
      fs::rename(tmp_path, final_path, ec);
      if (ec) {
        fs::copy_file(tmp_path, final_path, fs::copy_options::overwrite_existing, ec);
        if (!ec) {
          fs::remove(tmp_path, ec);
        }
      }
    }
    if (ec) {
      ROS_WARN("finalize bag json failed: %s", ec.message().c_str());
    } else if (fs::is_regular_file(final_path)) {
      const int file_fd = ::open(final_path.c_str(), O_RDONLY);
      if (file_fd >= 0) {
        ::fsync(file_fd);
        ::close(file_fd);
      }
      const int dir_fd = ::open(bag_dir.c_str(), O_RDONLY | O_DIRECTORY);
      if (dir_fd >= 0) {
        ::fsync(dir_fd);
        ::close(dir_fd);
      }
    }
    session_id_ = 0;
    tmp_path_.clear();
    final_path_.clear();
    bag_dir_.clear();
  }

  Json::Value status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value out(Json::objectValue);
    out["session_id"] = session_id_;
    out["tmp_path"] = tmp_path_.string();
    out["lines_written"] = static_cast<Json::Int64>(lines_written_);
    return out;
  }

 private:
  void finishLocked() {
    if (file_) {
      file_.flush();
      file_.close();
    }
  }

  mutable std::mutex mutex_;
  int session_id_ = 0;
  fs::path bag_dir_;
  fs::path tmp_path_;
  fs::path final_path_;
  std::ofstream file_;
  size_t lines_written_ = 0;
  size_t flush_every_ = 20;
};

class LatestPostWorker {
 public:
  using Callback = std::function<void(const std::string&, bool, const HttpResponse&)>;

  explicit LatestPostWorker(Callback cb) : callback_(std::move(cb)) {}

  void start() {
    stop_ = false;
    worker_ = std::thread([this] { run(); });
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
      cv_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void submit(const std::string& kind, const std::string& url, const std::string& body, double timeout_s) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = Task{kind, url, body, timeout_s};
    cv_.notify_all();
  }

 private:
  struct Task {
    std::string kind;
    std::string url;
    std::string body;
    double timeout_s = 1.0;
  };

  void run() {
    while (ros::ok()) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || latest_.has_value(); });
        if (stop_) {
          break;
        }
        task = *latest_;
        latest_.reset();
      }
      HttpResponse resp = httpPostJson(task.url, task.body, task.timeout_s);
      if (callback_) {
        callback_(task.kind, resp.ok, resp);
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Task> latest_;
  bool stop_ = false;
  std::thread worker_;
  Callback callback_;
};

class CameraPostWorker {
 public:
  using Callback = std::function<void(bool, const HttpResponse&)>;

  explicit CameraPostWorker(Callback cb) : callback_(std::move(cb)) {}

  void start() {
    stop_ = false;
    worker_ = std::thread([this] { run(); });
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
      latest_.reset();
      cv_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void submit(
      const std::string& url,
      int64_t ts_ms,
      int width,
      int height,
      std::vector<uint8_t> jpg,
      double timeout_s) {
    if (url.empty() || jpg.empty() || width <= 0 || height <= 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = Task{url, ts_ms, width, height, std::move(jpg), timeout_s};
    cv_.notify_all();
  }

 private:
  struct Task {
    std::string url;
    int64_t ts_ms = 0;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> jpg;
    double timeout_s = 1.0;
  };

  void run() {
    while (ros::ok()) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stop_ || latest_.has_value(); });
        if (stop_) {
          break;
        }
        task = std::move(*latest_);
        latest_.reset();
      }

      Json::Value payload(Json::objectValue);
      payload["ts"] = static_cast<Json::Int64>(task.ts_ms);
      payload["w"] = task.width;
      payload["h"] = task.height;
      payload["format"] = "jpeg_base64";
      payload["data"] = base64Encode(task.jpg.data(), task.jpg.size());
      HttpResponse resp = httpPostJson(task.url, jsonToString(payload), task.timeout_s);
      if (callback_) {
        callback_(resp.ok, resp);
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Task> latest_;
  bool stop_ = false;
  std::thread worker_;
  Callback callback_;
};

struct SessionShared {
  std::mutex mutex;
  bool recording = false;
  std::string camera_0_dir;
  std::string camera_1_dir;
};

std::string makeDefaultCameraPipeline(const std::string& device, ImageSize resize) {
  if (resize.width <= 0) resize.width = 640;
  if (resize.height <= 0) resize.height = 360;
  std::ostringstream ss;
  ss << "v4l2src device=" << device
     << " ! rawvideoparse format=" << kDefaultRawVideoParseFormat
     << " width=" << kDefaultCameraInputWidth
     << " height=" << kDefaultCameraInputHeight
     << " ! videoconvert"
     << " ! videoscale"
     << " ! video/x-raw,format=RGB,width=" << resize.width
     << ",height=" << resize.height
     << " ! appsink name=sink drop=true sync=false max-buffers=1";
  return ss.str();
}

class GstCameraCapture {
 public:
  ~GstCameraCapture() {
    close();
  }

  bool open(const std::string& pipeline_desc, std::string* error_msg) {
    close();
    GError* error = nullptr;
    GstElement* parsed = gst_parse_launch(pipeline_desc.c_str(), &error);
    if (error) {
      if (error_msg) *error_msg = error->message ? error->message : "gst_parse_launch failed";
      g_error_free(error);
      if (parsed) gst_object_unref(parsed);
      return false;
    }
    if (!parsed) {
      if (error_msg) *error_msg = "gst_parse_launch returned null pipeline";
      return false;
    }
    if (!GST_IS_BIN(parsed)) {
      if (error_msg) *error_msg = "camera pipeline must be a bin";
      gst_object_unref(parsed);
      return false;
    }
    pipeline_ = parsed;
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink_ || !GST_IS_APP_SINK(sink_)) {
      if (error_msg) *error_msg = "camera pipeline must contain appsink name=sink";
      close();
      return false;
    }
    gst_app_sink_set_emit_signals(GST_APP_SINK(sink_), FALSE);
    gst_app_sink_set_drop(GST_APP_SINK(sink_), TRUE);
    gst_app_sink_set_max_buffers(GST_APP_SINK(sink_), 1);
    GstStateChangeReturn state = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      if (error_msg) *error_msg = "failed to set camera pipeline to PLAYING";
      close();
      return false;
    }
    state = gst_element_get_state(pipeline_, nullptr, nullptr, 2 * GST_SECOND);
    if (state == GST_STATE_CHANGE_FAILURE) {
      if (error_msg) *error_msg = "camera pipeline did not reach PLAYING";
      close();
      return false;
    }
    pipeline_desc_ = pipeline_desc;
    return true;
  }

  void close() {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (sink_) {
      gst_object_unref(sink_);
      sink_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    pipeline_desc_.clear();
  }

  bool isOpen() const {
    return pipeline_ != nullptr && sink_ != nullptr;
  }

  bool read(RgbImage* out, int timeout_ms) {
    if (!out || !sink_) {
      return false;
    }
    GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(sink_), static_cast<GstClockTime>(std::max(1, timeout_ms)) * GST_MSECOND);
    if (!sample) {
      return false;
    }
    bool ok = false;
    GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (caps && buffer) {
      const GstStructure* structure = gst_caps_get_structure(caps, 0);
      int width = 0;
      int height = 0;
      const gchar* format = structure ? gst_structure_get_string(structure, "format") : nullptr;
      if (structure &&
          gst_structure_get_int(structure, "width", &width) &&
          gst_structure_get_int(structure, "height", &height) &&
          format && std::string(format) == "RGB" &&
          width > 0 && height > 0) {
        GstMapInfo map{};
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
          const size_t needed = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
          if (map.size >= needed) {
            out->width = width;
            out->height = height;
            out->data.assign(map.data, map.data + needed);
            ok = true;
          }
          gst_buffer_unmap(buffer, &map);
        }
      }
    }
    gst_sample_unref(sample);
    return ok;
  }

 private:
  GstElement* pipeline_ = nullptr;
  GstElement* sink_ = nullptr;
  std::string pipeline_desc_;
};

class CameraWorker {
 public:
  CameraWorker(
      int index,
      int device_index,
      std::string device_path,
      std::string pipeline,
      std::string brain_url,
      int jpeg_quality,
      ImageSize resize,
      double http_timeout,
      double frame_interval_s,
      int queue_max,
      std::shared_ptr<std::atomic<bool>> shutdown_flag,
      std::shared_ptr<SessionShared> session,
      std::atomic<int>* http_status,
      std::atomic<int>* stream_status,
      fs::path scan_root,
      uint64_t min_free_bytes,
      double watchdog_timeout_s,
      double stream_error_confirm_s,
      double reconnect_cooldown_s,
      double startup_grace_s)
      : index_(index),
        device_index_(device_index),
        device_path_(std::move(device_path)),
        pipeline_(std::move(pipeline)),
        brain_url_(std::move(brain_url)),
        jpeg_quality_(jpeg_quality),
        resize_(resize),
        http_timeout_(http_timeout),
        frame_interval_s_(frame_interval_s),
        queue_max_(queue_max),
        shutdown_flag_(std::move(shutdown_flag)),
        session_(std::move(session)),
        http_status_(http_status),
        stream_status_(stream_status),
        post_worker_([this](bool ok, const HttpResponse&) {
          if (http_status_) {
            http_status_->store(ok ? 1 : 0);
          }
        }),
        scan_root_(std::move(scan_root)),
        min_free_bytes_(min_free_bytes),
        watchdog_timeout_s_(watchdog_timeout_s),
        stream_error_confirm_s_(stream_error_confirm_s),
        reconnect_cooldown_s_(reconnect_cooldown_s),
        startup_grace_s_(startup_grace_s) {}

  void start() {
    running_ = true;
    post_worker_.start();
    thread_ = std::thread([this] { run(); });
  }

  void stop() {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    post_worker_.stop();
  }

 private:
  bool canWriteNewData() const {
    std::error_code ec;
    auto space = fs::space(scan_root_, ec);
    return !ec && space.available >= min_free_bytes_;
  }

  std::string deviceLabel() const {
    return device_path_.empty() ? ("/dev/video" + std::to_string(device_index_)) : device_path_;
  }

  bool localDeviceAvailable() const {
    if (!pipeline_.empty()) {
      return true;
    }
    if (device_path_.empty()) {
      return pathExists(deviceLabel());
    }
    if (startsWith(device_path_, "/dev/")) {
      return pathExists(device_path_);
    }
    if (!startsWith(device_path_, "http://") && !startsWith(device_path_, "https://")) {
      return pathExists(device_path_);
    }
    return true;
  }

  std::string effectivePipeline() const {
    return pipeline_.empty() ? makeDefaultCameraPipeline(deviceLabel(), resize_) : pipeline_;
  }

  bool reopen(GstCameraCapture& cap) {
    cap.close();
    if (!localDeviceAvailable()) {
      return false;
    }
    std::string error;
    std::string pipeline_desc = effectivePipeline();
    if (!cap.open(pipeline_desc, &error)) {
      ROS_WARN_THROTTLE(5.0, "Camera %d GStreamer open failed: %s; pipeline=%s", index_, error.c_str(), pipeline_desc.c_str());
      return false;
    }
    ROS_INFO("Camera %d GStreamer pipeline opened: %s", index_, pipeline_desc.c_str());
    return true;
  }

  void run() {
    const auto started = monotonicSeconds();
    GstCameraCapture cap;
    reopen(cap);
    double stream_error_start = 0.0;
    double last_reopen = 0.0;
    double last_log = 0.0;
    while (ros::ok() && running_ && !shutdown_flag_->load()) {
      const double now = monotonicSeconds();
      RgbImage frame;
      int read_timeout_ms = static_cast<int>(std::max(20.0, watchdog_timeout_s_ * 1000.0));
      bool ok = cap.isOpen() && cap.read(&frame, read_timeout_ms) && !frame.empty();
      if (!ok) {
        const bool in_grace = (now - started) < startup_grace_s_;
        if (!in_grace) {
          if (stream_error_start <= 0.0) {
            stream_error_start = now;
          }
          if (now - stream_error_start >= stream_error_confirm_s_) {
            stream_status_->store(1);
          }
        }
        if (now - last_reopen >= reconnect_cooldown_s_) {
          reopen(cap);
          last_reopen = now;
        }
        if (now - last_log >= 5.0) {
          ROS_WARN("Camera %d GStreamer read failed or device unavailable: %s", index_, deviceLabel().c_str());
          last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      stream_error_start = 0.0;
      stream_status_->store(0);
      auto jpg = encodeJpegBytes(frame, jpeg_quality_);
      if (!jpg) {
        continue;
      }
      const int64_t frame_ts_ms = unixMs();

      std::string target_dir;
      {
        std::lock_guard<std::mutex> lock(session_->mutex);
        if (session_->recording) {
          target_dir = index_ == 0 ? session_->camera_0_dir : session_->camera_1_dir;
        }
      }
      if (!target_dir.empty() && canWriteNewData()) {
        writeBytesAtomic(fs::path(target_dir) / (std::to_string(frame_ts_ms) + ".jpg"), *jpg);
      }
      post_worker_.submit(brain_url_, frame_ts_ms, frame.width, frame.height, std::move(*jpg), http_timeout_);
      if (frame_interval_s_ > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(frame_interval_s_));
      }
    }
    cap.close();
    stream_status_->store(1);
  }

  int index_;
  int device_index_;
  std::string device_path_;
  std::string pipeline_;
  std::string brain_url_;
  int jpeg_quality_;
  ImageSize resize_;
  double http_timeout_;
  double frame_interval_s_;
  int queue_max_;
  std::shared_ptr<std::atomic<bool>> shutdown_flag_;
  std::shared_ptr<SessionShared> session_;
  std::atomic<int>* http_status_;
  std::atomic<int>* stream_status_;
  CameraPostWorker post_worker_;
  fs::path scan_root_;
  uint64_t min_free_bytes_;
  double watchdog_timeout_s_;
  double stream_error_confirm_s_;
  double reconnect_cooldown_s_;
  double startup_grace_s_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

class UnifiedDataNode;

struct SessionQualityResult {
  bool qualified = false;
  size_t total_error_count = 0;
  bool errors_truncated = false;
  size_t json_records = 0;
  size_t camera0_files = 0;
  size_t camera1_files = 0;
  size_t gray_files = 0;
  size_t range_files = 0;
  size_t archive_entries = 0;
  int64_t max_camera0_delta_ms = 0;
  int64_t max_camera1_delta_ms = 0;
  int64_t max_gray_delta_ms = 0;
  int64_t max_range_delta_ms = 0;
  std::vector<std::string> errors;
};

class SessionZipUploader {
 public:
  explicit SessionZipUploader(UnifiedDataNode* node);
  void start();
  void stop();
  void notifySessionClosed(const fs::path& session_root);
  void notifyCapacityPressure();

 private:
  class SessionClaim {
   public:
    SessionClaim(SessionZipUploader* owner, const fs::path& session_root)
        : owner_(owner), session_root_(session_root), claimed_(owner_->tryClaimSession(session_root_)) {}
    ~SessionClaim() {
      if (claimed_) owner_->releaseSession(session_root_);
    }
    SessionClaim(const SessionClaim&) = delete;
    SessionClaim& operator=(const SessionClaim&) = delete;
    explicit operator bool() const { return claimed_; }

   private:
    SessionZipUploader* owner_;
    fs::path session_root_;
    bool claimed_ = false;
  };

  SessionQualityResult validateSession(const fs::path& session_root) const;
  Json::Value qualityJson(const SessionQualityResult& quality) const;
  bool createAndVerifyZip(const fs::path& session_root, const fs::path& archive_path,
                          size_t* entry_count, std::string* error) const;
  bool runProgram(const std::vector<std::string>& args, const fs::path& cwd = {}) const;
  bool sha256File(const fs::path& path, std::string* digest) const;
  bool processSession(const fs::path& session_root);
  bool uploadTask(Json::Value* task);
  bool validateCompletedResponse(const Json::Value& task, const Json::Value& response) const;
  bool waitForCompletion(const Json::Value& task, Json::Value* completed);
  bool taskIsDamaged(const Json::Value& task) const;
  bool resumePendingTasks(bool damaged_tasks, bool* uploaded_task = nullptr);
  void discoverDamagedArchives();
  bool reclaimDamagedArchivesForCapacity();
  bool scanAndProcessSessions();
  bool sessionHasQueuedArchive(const fs::path& session_root) const;
  bool hasPendingQualitySessions() const;
  bool hasPendingNormalTasks() const;
  bool damagedUploadsMayStart();
  bool sessionIsStable(const fs::path& session_root) const;
  bool safeRemoveSession(const fs::path& session_root) const;
  bool datasetHasPendingTasks(const fs::path& dataset_root) const;
  bool cleanupCompletedDataset(const fs::path& dataset_root);
  void cleanupCompletedDatasets();
  void cleanupSessionCacheArtifacts();
  bool validTaskPaths(const Json::Value& task, fs::path* dataset_root = nullptr,
                      fs::path* session_root = nullptr, fs::path* archive_path = nullptr) const;
  bool readTask(const fs::path& path, Json::Value* task) const;
  std::vector<fs::path> listTaskFiles() const;
  bool taskPathExists(const fs::path& path) const;
  bool tryClaimSession(const fs::path& session_root);
  void releaseSession(const fs::path& session_root);
  void scheduleQualityScanAfter(double seconds);
  bool waitForStop(double seconds);
  void cleanupLegacyArtifacts();
  void logEvent(const fs::path& dataset_root, const std::string& event,
                const Json::Value& details) const;
  bool saveTask(Json::Value* task);
  void removeTask(const Json::Value& task);
  void notifyUploadWork();
  void qualityLoop();
  void uploadLoop();

  UnifiedDataNode* node_;
  fs::path scan_root_;
  std::string upload_init_url_;
  std::string upload_chunk_url_;
  std::string upload_complete_url_;
  std::string upload_status_url_;
  double upload_interval_s_ = 90.0;
  double stop_flush_delay_s_ = 10.0;
  uint64_t chunk_size_ = kDefaultChunkSizeBytes;
  int alignment_tolerance_ms_ = 500;
  size_t quality_error_detail_limit_ = 200;
  std::string zip_binary_ = kDefaultZipBinary;
  std::string unzip_binary_ = kDefaultUnzipBinary;
  int zip_compression_level_ = 1;
  int upload_parallel_chunks_ = 4;
  int upload_retry_count_ = 3;
  double complete_wait_timeout_s_ = 1800.0;
  double upload_status_interval_s_ = 5.0;
  double cache_cleanup_interval_s_ = 3600.0;
  double zip_tmp_ttl_s_ = 86400.0;
  double status_orphan_ttl_s_ = 604800.0;
  fs::path zip_tmp_root_;
  fs::path status_root_;
  double connect_timeout_s_ = kDefaultNasConnectTimeoutS;
  double request_timeout_s_ = kDefaultNasRequestTimeoutS;
  int session_recreate_failures_ = kDefaultSessionRecreateFailures;
  std::unique_ptr<NasHttpSession> http_session_;
  std::mutex mutex_;
  std::condition_variable quality_cv_;
  std::condition_variable upload_cv_;
  bool stop_ = false;
  bool upload_work_pending_ = false;
  bool startup_recovery_complete_ = false;
  bool initial_quality_scan_complete_ = false;
  bool quality_scan_active_ = false;
  uint64_t close_generation_ = 0;
  std::optional<double> next_forced_quality_scan_at_;
  std::thread quality_thread_;
  std::thread upload_thread_;
  mutable std::mutex status_mutex_;
  mutable std::mutex claim_mutex_;
  std::map<std::string, std::pair<std::thread::id, size_t>> active_session_claims_;
  mutable std::mutex log_mutex_;
};

struct DataCache {
  std::optional<std_msgs::Float32> angular_vel;
  std::optional<interfaces::SinglePoint> bucket_local;
  std::optional<interfaces::SinglePoint> bucket_rot;
  std::optional<geometry_msgs::PointStamped> bucket_terrain_delta;
  std::optional<geometry_msgs::Quaternion> joystick;
  std::optional<interfaces::imu_syd> imu_arm;
  std::optional<interfaces::imu_syd> imu_boom;
  std::optional<interfaces::imu_syd> imu_bucket;
  std::optional<interfaces::imu_syd> imu_vehicle;
  std::optional<interfaces::JointAngles> joints_angles;
  std::optional<geometry_msgs::Quaternion> joints_velocity;
  std::optional<std_msgs::Float32> swivel_angle;
  std::optional<std_msgs::Float64> sys_seq_action;
  std::optional<geometry_msgs::Pose> sys_sco_traj;
  std::optional<geometry_msgs::Quaternion> fused_controller_joystick;
  std::optional<geometry_msgs::Pose> smolvla_output_angle;
  Json::Value excavator_data_joystick = Json::Value(Json::nullValue);
  std::string fault_code;
};

class UnifiedDataNode {
 public:
  UnifiedDataNode() : nh_(), pnh_("~"), brain_post_worker_([this](const std::string& kind, bool ok, const HttpResponse& resp) {
    handleBrainStatus(kind, ok, resp);
  }) {
    curl_global_init(CURL_GLOBAL_ALL);
    gst_init(nullptr, nullptr);
    brain_base_url_ = getParam<std::string>("brain_base_url", kDefaultBrainBaseUrl);
    brain_state_url_ = joinUrl(brain_base_url_, getParam<std::string>("brain_state_path", "/state_data"));
    brain_gridmap_url_ = joinUrl(brain_base_url_, getParam<std::string>("brain_gridmap_path", "/gridmap_data"));
    brain_range_url_ = joinUrl(brain_base_url_, getParam<std::string>("brain_range_path", "/range_data"));
    brain_cam0_url_ = joinUrl(brain_base_url_, getParam<std::string>("brain_camera0_path", "/camera0_data"));
    brain_cam1_url_ = joinUrl(brain_base_url_, getParam<std::string>("brain_camera1_path", "/camera1_data"));
    http_timeout_ = getParam<double>("http_timeout", 0.8);
    http_session_recreate_failures_ = getParam<int>("http_session_recreate_failures", kDefaultSessionRecreateFailures);
    jpeg_quality_ = getParam<int>("jpeg_quality", 90);
    scan_root_ = normalizeConfiguredDirectory(resolveScanRoot(), "scan_root");
    min_free_bytes_ = static_cast<uint64_t>(getParam<double>("min_free_bytes", static_cast<double>(kMinFreeBytes)));
    max_scan_root_bytes_ = static_cast<uint64_t>(std::max<double>(1.0, getParam<double>(
        "max_scan_root_bytes", static_cast<double>(kDefaultMaxScanRootBytes))));
    local_image_queue_max_ = getParam<int>("local_image_queue_max", 1000);
    ensureDirOrThrow(scan_root_);

    std::string default_machine_model = getParam<std::string>("default_machine_model", "SY750-1");
    std::string default_operation_mode = getParam<std::string>("default_operation_mode", "人工");
    machine_model_ = promptSelection("请选择机型", {"SY750-1", "SY870-1", "SY1350-1", "其他"}, default_machine_model);
    operation_mode_ = promptSelection("请选择操作模式", {"人工", "遥控", "无人", "其他"}, default_operation_mode);
    if (operation_mode_ == "无人") {
      std::string startup_default = timeFormat(std::time(nullptr), "%Y%m%d%H%M%S");
      model_id_ = promptText("请输入模型编号", startup_default, 30);
      trajectory_fusion_code_version_ = promptText("请输入轨迹融合代码版本", startup_default, 30);
      control_code_version_ = promptText("请输入控制代码版本", startup_default, 30);
    }
    location_ = promptSelection("请选择地点", {"林芝", "湖州", "昆山", "其他"}, "林芝");
    weather_ = promptSelection("请选择天气", {"晴天", "阴天", "雨天", "雪天", "雾天", "其他"}, "晴天");

    delta_vla_height_ = loadDeltaVlaHeight();
    capture_switch_debounce_s_ = getParam<double>("capture_switch_debounce_s", 1.0);
    state_flush_every_ = getParam<int>("state_flush_every", 20);
    state_flush_interval_s_ = getParam<double>("state_flush_interval_s", 0.5);
    state_fsync_interval_s_ = getParam<double>("state_fsync_interval_s", 5.0);
    state_json_queue_max_ = getParam<int>("state_json_queue_max", 20000);

    topic_monitors_ = {
        {"/rtk_can_msg", false},
        {"/angular_velocity", false},
        {"/bucket_local_position", false},
        {"/bucket_rotation_local_position", false},
        {"/bucket_terrain_delta_position", false},
        // {"/evcavator_joystick_topic", false},
        {"/imu_topic_arm", false},
        {"/imu_topic_boom", false},
        {"/imu_topic_bucket", false},
        {"/imu_topic_vehicle", false},
        {"/joints_angles", false},
        {"/joints_velocity", false},
        {"/swivel_angle", false},
        {"/Sys_SeqAction", false},
        {"/Sys_SCo_Traj", false},
        {"/fused_controller_joystick_topic", false},
        {"/excavation_gridmap", false},
        {"/gray_range_image", false},
    };

    pred_action_pub_ = nh_.advertise<geometry_msgs::Pose>("/smolVLA_output_angle", 10);
    smolvla_status_pub_ = nh_.advertise<std_msgs::Int8>("/smolVLA_status_check", 10);
    camera_safety_pub_ = nh_.advertise<std_msgs::UInt32>("/CameraSafety_topic", 10);

    session_shared_ = std::make_shared<SessionShared>();
    shutdown_flag_ = std::make_shared<std::atomic<bool>>(false);
    session_uploader_ = std::make_unique<SessionZipUploader>(this);

    recoverOrphanStateJsonTmpFiles();
    brain_post_worker_.start();
    session_uploader_->start();
    startCameraWorkers();
    setupSubscribers();

    status_timer_ = nh_.createTimer(ros::Duration(0.5), [this](const ros::TimerEvent&) { computeAndPublishStatus(); });
    heartbeat_timer_ = nh_.createTimer(ros::Duration(0.1), &UnifiedDataNode::heartbeatTimerCb, this);
    debounce_timer_ = nh_.createTimer(ros::Duration(0.1), &UnifiedDataNode::captureSwitchDebounceTimerCb, this);
    diagnostic_timer_ = nh_.createTimer(ros::Duration(1.0), &UnifiedDataNode::diagnosticTimerCb, this);
    state_status_timer_ = nh_.createTimer(ros::Duration(30.0), &UnifiedDataNode::stateJsonStatusTimerCb, this);
    ROS_INFO("data_to_NAS_and_ubuntu2204 C++ node started. scan_root=%s", scan_root_.c_str());
  }

  ~UnifiedDataNode() {
    shutdown();
    curl_global_cleanup();
  }

  const fs::path& scanRoot() const { return scan_root_; }
  uint64_t minFreeBytes() const { return min_free_bytes_; }
  uint64_t maxScanRootBytes() const { return max_scan_root_bytes_; }

  template <typename T>
  T getParam(const std::string& name, const T& default_value) const {
    T value;
    if (pnh_.getParam(name, value)) {
      return value;
    }
    return default_value;
  }

  void prepareUploadArtifacts() {
    int session_id = 0;
    fs::path tmp_path;
    fs::path final_path;
    fs::path bag_dir;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (!recording_ || json_tmp_path_.empty()) {
        return;
      }
      session_id = state_session_id_;
      tmp_path = json_tmp_path_;
      final_path = json_final_path_;
      bag_dir = bag_dir_;
    }
    if (!canWriteNewData()) {
      logLowSpace("prepareUploadArtifacts");
      return;
    }
    state_json_writer_.snapshot(session_id, tmp_path, final_path, bag_dir);
  }

  void shutdown() {
    if (shutdown_done_.exchange(true)) {
      return;
    }
    ROS_INFO("data_to_NAS_and_ubuntu2204 C++ shutting down...");
    shutdown_flag_->store(true);
    for (auto& worker : camera_workers_) {
      if (worker) worker->stop();
    }
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      recording_ = false;
      finalizeSessionLocked();
    }
    if (session_uploader_) session_uploader_->stop();
    brain_post_worker_.stop();
    ROS_INFO("data_to_NAS_and_ubuntu2204 C++ shutdown complete.");
  }

 private:
  fs::path resolveScanRoot() {
    const std::string configured = getParam<std::string>("scan_root", kDefaultScanRoot);
    return fs::path(configured.empty() ? kDefaultScanRoot : configured);
  }

  double loadDeltaVlaHeight() {
    std::string default_path = (fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path().parent_path() / "config" / "param_sy870.yaml").string();
    std::string yaml_path = getParam<std::string>("yaml_config_path", default_path);
    try {
      YAML::Node cfg = YAML::LoadFile(yaml_path);
      if (cfg["delta_vla_height"]) {
        double value = cfg["delta_vla_height"].as<double>();
        ROS_INFO("[config] delta_vla_height = %.5f (%s)", value, yaml_path.c_str());
        return value;
      }
    } catch (const std::exception& e) {
      ROS_WARN("[config] 读取 delta_vla_height 失败: %s，使用默认 0.0", e.what());
    }
    return 0.0;
  }

  void setupSubscribers() {
    sub_angular_vel_ = nh_.subscribe("/angular_velocity", 10, &UnifiedDataNode::angularVelCallback, this);
    sub_swivel_angle_ = nh_.subscribe("/swivel_angle", 10, &UnifiedDataNode::swivelAngleCallback, this);
    sub_sys_seq_action_ = nh_.subscribe("/Sys_SeqAction", 10, &UnifiedDataNode::sysSeqActionCallback, this);
    sub_sys_sco_traj_ = nh_.subscribe("/Sys_SCo_Traj", 10, &UnifiedDataNode::sysScoTrajCallback, this);
    sub_fused_controller_joystick_ = nh_.subscribe("/fused_controller_joystick_topic", 10, &UnifiedDataNode::fusedControllerJoystickCallback, this);
    sub_bucket_local_ = nh_.subscribe("/bucket_local_position", 10, &UnifiedDataNode::bucketLocalCallback, this);
    sub_bucket_rot_ = nh_.subscribe("/bucket_rotation_local_position", 10, &UnifiedDataNode::bucketRotCallback, this);
    sub_bucket_terrain_delta_ = nh_.subscribe("/bucket_terrain_delta_position", 10, &UnifiedDataNode::bucketTerrainDeltaCallback, this);
    sub_joystick_ = nh_.subscribe("/evcavator_joystick_topic", 10, &UnifiedDataNode::joystickCallback, this);
    sub_imu_arm_ = nh_.subscribe("/imu_topic_arm", 10, &UnifiedDataNode::imuArmCallback, this);
    sub_imu_boom_ = nh_.subscribe("/imu_topic_boom", 10, &UnifiedDataNode::imuBoomCallback, this);
    sub_imu_bucket_ = nh_.subscribe("/imu_topic_bucket", 10, &UnifiedDataNode::imuBucketCallback, this);
    sub_imu_vehicle_ = nh_.subscribe("/imu_topic_vehicle", 10, &UnifiedDataNode::imuVehicleCallback, this);
    sub_joints_angles_ = nh_.subscribe("/joints_angles", 10, &UnifiedDataNode::jointsAnglesCallback, this);
    sub_joints_velocity_ = nh_.subscribe("/joints_velocity", 10, &UnifiedDataNode::jointsVelocityCallback, this);
    sub_rtk_ = nh_.subscribe("/rtk_can_msg", 50, &UnifiedDataNode::rtkCallback, this);
    sub_gridmap_ = nh_.subscribe("/excavation_gridmap", 3, &UnifiedDataNode::gridmapCallback, this);
    sub_gray_range_ = nh_.subscribe("/gray_range_image", 3, &UnifiedDataNode::grayRangeImageCallback, this);
    sub_fault_code_ = nh_.subscribe("/fault_code", 10, &UnifiedDataNode::faultCodeCallback, this);
    sub_save_signal_ = nh_.subscribe("/data_capture_switch_manual", 10, &UnifiedDataNode::saveSignalCallback, this);
  }

  void startCameraWorkers() {
    std::vector<int> resize_param;
    pnh_.param("resize", resize_param, std::vector<int>{640, 360});
    ImageSize resize{640, 360};
    if (resize_param.size() == 2) {
      resize = ImageSize{resize_param[0], resize_param[1]};
    }
    double frame_interval_s = std::max(0.0, getParam<double>("frame_interval_s", 0.066));
    int queue_max = std::max(1, getParam<int>("camera_queue_max", 200));
    double watchdog_timeout_s = getParam<double>("camera_watchdog_timeout_s", 0.6);
    double stream_error_confirm_s = getParam<double>("camera_stream_error_confirm_s", 2.5);
    double reconnect_cooldown_s = getParam<double>("camera_reconnect_cooldown_s", 2.0);
    double startup_grace_s = getParam<double>("camera_startup_grace_s", 5.0);
    int cam0_device = getParam<int>("camera0_device", 0);
    int cam1_device = getParam<int>("camera1_device", 1);
    std::string cam0_device_path = getParam<std::string>("camera0_device_path", "");
    std::string cam1_device_path = getParam<std::string>("camera1_device_path", "");
    std::string cam0_pipeline = getParam<std::string>("camera0_pipeline", "");
    std::string cam1_pipeline = getParam<std::string>("camera1_pipeline", "");
    camera_workers_.push_back(std::make_unique<CameraWorker>(
        0, cam0_device, cam0_device_path, cam0_pipeline, brain_cam0_url_, jpeg_quality_, resize, http_timeout_, frame_interval_s, queue_max,
        shutdown_flag_, session_shared_, &cam0_http_status_, &cam0_stream_status_, scan_root_, min_free_bytes_, watchdog_timeout_s,
        stream_error_confirm_s, reconnect_cooldown_s, startup_grace_s));
    camera_workers_.push_back(std::make_unique<CameraWorker>(
        1, cam1_device, cam1_device_path, cam1_pipeline, brain_cam1_url_, jpeg_quality_, resize, http_timeout_, frame_interval_s, queue_max,
        shutdown_flag_, session_shared_, &cam1_http_status_, &cam1_stream_status_, scan_root_, min_free_bytes_, watchdog_timeout_s,
        stream_error_confirm_s, reconnect_cooldown_s, startup_grace_s));
    camera_started_at_[0] = monotonicSeconds();
    camera_started_at_[1] = camera_started_at_[0];
    for (auto& worker : camera_workers_) {
      worker->start();
    }
    ROS_INFO("摄像头线程已启动，实时发送到大脑域控并按保存信号写入 G500。");
  }

  void markTopicReceived(const std::string& topic) {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    auto it = topic_monitors_.find(topic);
    if (it != topic_monitors_.end()) {
      it->second = true;
    }
  }

  void resetDataDiagnostics() {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    for (auto& kv : topic_monitors_) {
      kv.second = false;
    }
    topic_diagnostics_active_ = true;
    topic_diagnostics_started_at_ = monotonicSeconds();
    last_topic_warn_at_.clear();
    last_rtk_monotonic_ts_ = 0.0;
  }

  void stopDataDiagnostics() {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    topic_diagnostics_active_ = false;
  }

  bool warnDue(double last_warn, double interval = 5.0) const {
    return monotonicSeconds() - last_warn >= interval;
  }

  bool canWriteNewData() const {
    std::error_code ec;
    auto space = fs::space(scan_root_, ec);
    return !ec && space.available >= min_free_bytes_;
  }

  bool captureCapacityAllowsStart(uint64_t* used_bytes = nullptr,
                                  uint64_t* available_bytes = nullptr) const {
    bool size_ok = false;
    const uint64_t used = directoryLogicalSize(scan_root_, &size_ok);
    std::error_code ec;
    const auto space = fs::space(scan_root_, ec);
    if (used_bytes) *used_bytes = used;
    if (available_bytes) *available_bytes = ec ? 0 : space.available;
    return size_ok && !ec && used < max_scan_root_bytes_ && space.available >= min_free_bytes_;
  }

  void logLowSpace(const std::string& source) {
    std::lock_guard<std::mutex> lock(low_space_log_mutex_);
    double now = monotonicSeconds();
    if (now - last_low_space_log_ts_ >= 5.0) {
      last_low_space_log_ts_ = now;
      ROS_WARN("%s stop local writes: G500 free space below threshold.", source.c_str());
    }
  }

  void buildSessionPaths(std::time_t session_ts_sec) {
    std::string compact = timeFormat(session_ts_sec, "%Y%m%d%H%M%S");
    std::string human = timeFormat(session_ts_sec, "%Y-%m-%d_%H-%M-%S");
    std::string bag_name = timeFormat(session_ts_sec, "%Y%m%d-%H%M%S");
    dataset_name_ = "excavator_data_" + timeFormat(session_ts_sec, "%Y%m%d");
    dataset_root_ = scan_root_ / dataset_name_;
    session_root_ = dataset_root_ / compact;
    bag_dir_ = session_root_ / "bag";
    gray_dir_ = session_root_ / "grayPicture" / (human + "_png");
    range_dir_ = session_root_ / "rangePicture" / (human + "_range_images");
    json_final_path_ = bag_dir_ / (bag_name + ".json");
    json_tmp_path_ = fs::path(json_final_path_.string() + ".tmp");
    fs::path cam0 = session_root_ / "camera_data" / (compact + "_camera_0");
    fs::path cam1 = session_root_ / "camera_data" / (compact + "_camera_1");
    ensureDir(bag_dir_);
    ensureDir(gray_dir_);
    ensureDir(range_dir_);
    ensureDir(cam0);
    ensureDir(cam1);
    {
      std::lock_guard<std::mutex> lock(session_shared_->mutex);
      session_shared_->camera_0_dir = cam0.string();
      session_shared_->camera_1_dir = cam1.string();
    }
  }

  void appendStateLine(const std::string& line) {
    int session_id = 0;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (!recording_) {
        return;
      }
      session_id = state_session_id_;
    }
    state_json_writer_.append(session_id, line);
  }

  void finalizeSessionLocked() {
    if (state_session_id_ && !json_tmp_path_.empty()) {
      state_json_writer_.finishSession(state_session_id_, json_tmp_path_, json_final_path_, bag_dir_);
    }
  }

  void saveSignalCallback(const std_msgs::Int8::ConstPtr& msg) {
    int signal_value = msg->data == 0 ? 0 : 1;
    double now = monotonicSeconds();
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (signal_value != raw_save_signal_) {
      raw_save_signal_ = signal_value;
      raw_save_signal_changed_at_ = now;
      ROS_INFO("capture switch raw changed. raw=%d applied=%d debounce=%.2fs", raw_save_signal_, last_save_signal_, capture_switch_debounce_s_);
    }
  }

  void captureSwitchDebounceTimerCb(const ros::TimerEvent&) {
    int previous = 0;
    int signal_value = 0;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (raw_save_signal_ == last_save_signal_) {
        return;
      }
      if (monotonicSeconds() - raw_save_signal_changed_at_ < capture_switch_debounce_s_) {
        return;
      }
      previous = last_save_signal_;
      signal_value = raw_save_signal_;
    }
    applySaveSignalTransition(previous, signal_value);
  }

  void applySaveSignalTransition(int previous, int signal_value) {
    bool applied = false;
    std::lock_guard<std::mutex> transition_lock(save_signal_transition_mutex_);
    if (previous == 0 && signal_value == 1) {
      uint64_t used_bytes = 0;
      uint64_t available_bytes = 0;
      const bool capacity_allows = captureCapacityAllowsStart(&used_bytes, &available_bytes);
      std::time_t host_ts_sec = std::time(nullptr);
      std::string dataset_name;
      fs::path json_tmp_path;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (last_save_signal_ != previous || raw_save_signal_ != signal_value) {
          return;
        }
        // Consume this debounced high level even when admission is denied.  A
        // fresh 0 -> 1 edge is required after storage has been reclaimed.
        last_save_signal_ = signal_value;
        if (!capacity_allows) {
          ROS_WARN("capture start rejected. scan_root_bytes=%llu limit=%llu available=%llu min_free=%llu",
                   static_cast<unsigned long long>(used_bytes),
                   static_cast<unsigned long long>(max_scan_root_bytes_),
                   static_cast<unsigned long long>(available_bytes),
                   static_cast<unsigned long long>(min_free_bytes_));
          if (session_uploader_) session_uploader_->notifyCapacityPressure();
          return;
        }
        state_session_id_ += 1;
        session_ts_sec_ = host_ts_sec;
        buildSessionPaths(host_ts_sec);
        state_json_writer_.beginSession(state_session_id_, bag_dir_, json_tmp_path_, json_final_path_);
        recording_ = true;
        {
          std::lock_guard<std::mutex> shared_lock(session_shared_->mutex);
          session_shared_->recording = true;
        }
        dataset_name = dataset_name_;
        json_tmp_path = json_tmp_path_;
        resetDataDiagnostics();
        applied = true;
      }
      ROS_INFO("recording started. dataset=%s json_tmp=%s signal=%d", dataset_name.c_str(), json_tmp_path.c_str(), signal_value);
    } else if (previous == 1 && signal_value == 0) {
      int session_id = 0;
      std::string dataset_name;
      fs::path tmp_path;
      fs::path final_path;
      fs::path bag_dir;
      fs::path closed_session_root;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (last_save_signal_ != previous || raw_save_signal_ != signal_value) {
          return;
        }
        last_save_signal_ = signal_value;
        if (!recording_) {
          return;
        }
        session_id = state_session_id_;
        dataset_name = dataset_name_;
        tmp_path = json_tmp_path_;
        final_path = json_final_path_;
        bag_dir = bag_dir_;
        closed_session_root = session_root_;
        recording_ = false;
        {
          std::lock_guard<std::mutex> shared_lock(session_shared_->mutex);
          session_shared_->recording = false;
          session_shared_->camera_0_dir.clear();
          session_shared_->camera_1_dir.clear();
        }
        stopDataDiagnostics();
        applied = true;
      }
      if (session_id && !tmp_path.empty()) {
        state_json_writer_.finishSession(session_id, tmp_path, final_path, bag_dir);
      }
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (state_session_id_ == session_id) {
          session_ts_sec_ = 0;
          dataset_name_.clear();
          dataset_root_.clear();
          session_root_.clear();
          bag_dir_.clear();
          gray_dir_.clear();
          range_dir_.clear();
          json_tmp_path_.clear();
          json_final_path_.clear();
        }
      }
      ROS_INFO("recording stopped. dataset=%s json_final=%s signal=%d", dataset_name.c_str(), final_path.c_str(), signal_value);
      if (session_uploader_ && !closed_session_root.empty()) {
        session_uploader_->notifySessionClosed(closed_session_root);
      }
    }
    (void)applied;
  }

  std::pair<fs::path, fs::path> currentImageDirs() {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!recording_) {
      return {fs::path(), fs::path()};
    }
    return {gray_dir_, range_dir_};
  }

  void angularVelCallback(const std_msgs::Float32::ConstPtr& msg) { markTopicReceived("/angular_velocity"); std::lock_guard<std::mutex> lock(data_mutex_); data_.angular_vel = *msg; }
  void bucketLocalCallback(const interfaces::SinglePoint::ConstPtr& msg) { markTopicReceived("/bucket_local_position"); std::lock_guard<std::mutex> lock(data_mutex_); data_.bucket_local = *msg; }
  void bucketRotCallback(const interfaces::SinglePoint::ConstPtr& msg) { markTopicReceived("/bucket_rotation_local_position"); std::lock_guard<std::mutex> lock(data_mutex_); data_.bucket_rot = *msg; }
  void bucketTerrainDeltaCallback(const geometry_msgs::PointStamped::ConstPtr& msg) { markTopicReceived("/bucket_terrain_delta_position"); std::lock_guard<std::mutex> lock(data_mutex_); data_.bucket_terrain_delta = *msg; }
  void joystickCallback(const geometry_msgs::Quaternion::ConstPtr& msg) { markTopicReceived("/evcavator_joystick_topic"); std::lock_guard<std::mutex> lock(data_mutex_); data_.joystick = *msg; }
  void imuArmCallback(const interfaces::imu_syd::ConstPtr& msg) { markTopicReceived("/imu_topic_arm"); std::lock_guard<std::mutex> lock(data_mutex_); data_.imu_arm = *msg; }
  void imuBoomCallback(const interfaces::imu_syd::ConstPtr& msg) { markTopicReceived("/imu_topic_boom"); std::lock_guard<std::mutex> lock(data_mutex_); data_.imu_boom = *msg; }
  void imuBucketCallback(const interfaces::imu_syd::ConstPtr& msg) { markTopicReceived("/imu_topic_bucket"); std::lock_guard<std::mutex> lock(data_mutex_); data_.imu_bucket = *msg; }
  void imuVehicleCallback(const interfaces::imu_syd::ConstPtr& msg) { markTopicReceived("/imu_topic_vehicle"); std::lock_guard<std::mutex> lock(data_mutex_); data_.imu_vehicle = *msg; }
  void jointsAnglesCallback(const interfaces::JointAngles::ConstPtr& msg) { markTopicReceived("/joints_angles"); std::lock_guard<std::mutex> lock(data_mutex_); data_.joints_angles = *msg; }
  void jointsVelocityCallback(const geometry_msgs::Quaternion::ConstPtr& msg) { markTopicReceived("/joints_velocity"); std::lock_guard<std::mutex> lock(data_mutex_); data_.joints_velocity = *msg; }
  void swivelAngleCallback(const std_msgs::Float32::ConstPtr& msg) { markTopicReceived("/swivel_angle"); std::lock_guard<std::mutex> lock(data_mutex_); data_.swivel_angle = *msg; }
  void sysSeqActionCallback(const std_msgs::Float64::ConstPtr& msg) { markTopicReceived("/Sys_SeqAction"); std::lock_guard<std::mutex> lock(data_mutex_); data_.sys_seq_action = *msg; }
  void sysScoTrajCallback(const geometry_msgs::Pose::ConstPtr& msg) { markTopicReceived("/Sys_SCo_Traj"); std::lock_guard<std::mutex> lock(data_mutex_); data_.sys_sco_traj = *msg; }
  void fusedControllerJoystickCallback(const geometry_msgs::Quaternion::ConstPtr& msg) { markTopicReceived("/fused_controller_joystick_topic"); std::lock_guard<std::mutex> lock(data_mutex_); data_.fused_controller_joystick = *msg; }

  void faultCodeCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_.fault_code = msg->data;
  }

  Json::Value buildValidatedGroup(const std::string& field_name, const std::vector<std::pair<std::string, double>>& pairs) {
    Json::Value out(Json::objectValue);
    for (const auto& kv : pairs) {
      out[kv.first] = jsonDouble(kv.second);
    }
    return out;
  }

  Json::Value rtkJson(const interfaces::rtk_cgi610& msg) {
    Json::Value out(Json::objectValue);
    out["lat"] = jsonDouble(msg.lat, 100000000.0);
    out["lon"] = jsonDouble(msg.lon, 100000000.0);
    return out;
  }

  void rtkCallback(const interfaces::rtk_cgi610::ConstPtr& msg) {
    markTopicReceived("/rtk_can_msg");
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      last_rtk_monotonic_ts_ = monotonicSeconds();
    }
    DataCache data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      data = data_;
    }

    double stamp_sec = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    int64_t stamp_ms = static_cast<int64_t>(stamp_sec * 1000.0);
    std::string frame_key = "frame" + std::to_string(frame_counter_++) + ", " + fixed3(stamp_sec);

    Json::Value brain_payload(Json::objectValue);
    brain_payload["joints_angles"] = data.joints_angles ? jointsAnglesJson(*data.joints_angles) : nullValue();
    brain_payload["swivel_angle"] = data.swivel_angle ? float32Json(*data.swivel_angle) : nullValue();
    brain_payload["bucket_terrain_delta_position"] = data.bucket_terrain_delta ? pointStampedJson(*data.bucket_terrain_delta, delta_vla_height_, true) : nullValue();
    brain_payload["joints_velocity"] = data.joints_velocity ? quaternionJson(*data.joints_velocity) : nullValue();
    brain_payload["rtk_stamp_ms"] = Json::Int64(stamp_ms);
    brain_payload["rtk"] = rtkJson(*msg);
    Json::Value brain_root(Json::objectValue);
    brain_root[frame_key] = brain_payload;
    brain_post_worker_.submit("state", brain_state_url_, jsonToString(brain_root), http_timeout_);

    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (!recording_) {
        return;
      }
    }

    Json::Value save_payload(Json::objectValue);
    save_payload["rtk_stamp_ms"] = Json::Int64(stamp_ms);
    save_payload["rtk"] = rtkJson(*msg);
    save_payload["angular_velocity"] = buildValidatedGroup("angular_velocity", {{"data", data.angular_vel ? data.angular_vel->data : 0.0}});
    save_payload["bucket_local_position"] = buildValidatedGroup("bucket_local_position", {
        {"x", data.bucket_local ? data.bucket_local->point.x : 0.0},
        {"y", data.bucket_local ? data.bucket_local->point.y : 0.0},
        {"z", data.bucket_local ? data.bucket_local->point.z : 0.0}});
    save_payload["bucket_rotation_local_position"] = buildValidatedGroup("bucket_rotation_local_position", {
        {"x", data.bucket_rot ? data.bucket_rot->point.x : 0.0},
        {"y", data.bucket_rot ? data.bucket_rot->point.y : 0.0},
        {"z", data.bucket_rot ? data.bucket_rot->point.z : 0.0}});
    save_payload["bucket_terrain_delta_position"] = buildValidatedGroup("bucket_terrain_delta_position", {
        {"x", data.bucket_terrain_delta ? data.bucket_terrain_delta->point.x : 0.0},
        {"y", data.bucket_terrain_delta ? data.bucket_terrain_delta->point.y : 0.0},
        {"z", data.bucket_terrain_delta ? data.bucket_terrain_delta->point.z : 0.0}});
    // save_payload["evcavator_joystick_topic"] = data.joystick ? quaternionJson(*data.joystick) : buildValidatedGroup("evcavator_joystick_topic", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 0.0}});
    save_payload["joints_angles"] = data.joints_angles ? jointsAnglesSaveJson(*data.joints_angles) : buildValidatedGroup("joints_angles", {{"boom", 0.0}, {"arm", 0.0}, {"bucket", 0.0}});
    save_payload["joints_velocity"] = data.joints_velocity ? quaternionJson(*data.joints_velocity) : buildValidatedGroup("joints_velocity", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 0.0}});
    save_payload["swivel_angle"] = buildValidatedGroup("swivel_angle", {{"data", data.swivel_angle ? data.swivel_angle->data : 0.0}});
    Json::Value seq(Json::objectValue);
    seq["data"] = data.sys_seq_action ? jsonDouble(data.sys_seq_action->data) : nullValue();
    save_payload["Sys_SeqAction"] = seq;
    save_payload["Sys_SCo_Traj"] = data.sys_sco_traj ? poseJson(*data.sys_sco_traj) : nullValue();
    save_payload["fused_controller_joystick_topic"] = data.fused_controller_joystick ? quaternionJson(*data.fused_controller_joystick) : nullValue();
    save_payload["smolVLA_output_angle"] = data.smolvla_output_angle ? poseJson(*data.smolvla_output_angle) : nullValue();
    save_payload["excavator_data_joystick"] = data.excavator_data_joystick;
    save_payload["fault_code"] = data.fault_code;
    save_payload["machine_model"] = machine_model_;
    save_payload["operation_mode"] = operation_mode_;
    save_payload["location"] = location_;
    save_payload["weather"] = weather_;
    if (operation_mode_ == "无人") {
      save_payload["model_id"] = model_id_;
      save_payload["trajectory_fusion_code_version"] = trajectory_fusion_code_version_;
      save_payload["control_code_version"] = control_code_version_;
    }
    appendStateLine(savedFrameJsonLine(frame_key, save_payload));
  }

  static std::string fixed3(double value) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << value;
    return ss.str();
  }

  Json::Value jointsAnglesJson(const interfaces::JointAngles& msg) {
    Json::Value out(Json::objectValue);
    out["boom"] = jsonDouble(msg.joint_boom);
    out["arm"] = jsonDouble(msg.joint_arm);
    out["bucket"] = jsonDouble(msg.joint_bucket);
    return out;
  }

  Json::Value jointsAnglesSaveJson(const interfaces::JointAngles& msg) {
    return jointsAnglesJson(msg);
  }

  Json::Value float32Json(const std_msgs::Float32& msg) {
    Json::Value out(Json::objectValue);
    out["data"] = jsonDouble(msg.data);
    return out;
  }

  Json::Value pointStampedJson(const geometry_msgs::PointStamped& msg, double z_offset, bool include_stamp) {
    Json::Value out = pointJson(msg.point.x, msg.point.y, msg.point.z + z_offset);
    if (include_stamp) {
      double stamp_sec = msg.header.stamp.isZero() ? ros::Time::now().toSec() : msg.header.stamp.toSec();
      out["stamp_ms"] = Json::Int64(static_cast<int64_t>(stamp_sec * 1000.0));
    }
    return out;
  }

  std::optional<GrayImage> imageToGrayU8(const sensor_msgs::Image& msg) {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      return std::nullopt;
    }
    size_t bytes_per_px = 1;
    bool is_u16 = false;
    bool is_i16 = false;
    bool is_f32 = false;
    if (msg.encoding == "mono16" || msg.encoding == "16UC1" || msg.encoding == "16SC1") {
      bytes_per_px = 2;
      is_u16 = msg.encoding != "16SC1";
      is_i16 = msg.encoding == "16SC1";
    } else if (msg.encoding == "32FC1") {
      bytes_per_px = 4;
      is_f32 = true;
    }
    size_t row_bytes = msg.step > 0 ? msg.step : msg.width * bytes_per_px;
    const size_t min_row_bytes = static_cast<size_t>(msg.width) * bytes_per_px;
    if (row_bytes < min_row_bytes || (msg.height > 0 && row_bytes > msg.data.size() / msg.height)) {
      return std::nullopt;
    }
    GrayImage out = makeGrayImage(static_cast<int>(msg.width), static_cast<int>(msg.height), 0);
    if (out.empty()) {
      return std::nullopt;
    }
    if (bytes_per_px == 1) {
      for (uint32_t y = 0; y < msg.height; ++y) {
        const uint8_t* row = msg.data.data() + static_cast<size_t>(y) * row_bytes;
        std::copy(row, row + msg.width, out.data.begin() + static_cast<size_t>(y) * msg.width);
      }
      return out;
    }

    std::vector<double> values(static_cast<size_t>(msg.width) * msg.height, 0.0);
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    const bool big_endian = msg.is_bigendian != 0;
    for (uint32_t y = 0; y < msg.height; ++y) {
      const uint8_t* row = msg.data.data() + static_cast<size_t>(y) * row_bytes;
      for (uint32_t x = 0; x < msg.width; ++x) {
        const uint8_t* px = row + static_cast<size_t>(x) * bytes_per_px;
        double value = 0.0;
        if (is_f32) {
          value = readF32(px, big_endian);
        } else if (is_i16) {
          value = readI16(px, big_endian);
        } else if (is_u16) {
          value = readU16(px, big_endian);
        }
        if (!std::isfinite(value)) {
          value = 0.0;
        }
        values[static_cast<size_t>(y) * msg.width + x] = value;
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
      }
    }
    if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
      return out;
    }
    for (size_t i = 0; i < values.size(); ++i) {
      out.data[i] = normalizeToU8(values[i], min_value, max_value);
    }
    return out;
  }

  std::optional<GrayImage> resizeRangeCanvas(const GrayImage& gray) {
    if (gray.empty()) {
      return std::nullopt;
    }
    const int target_h = 360;
    const int target_w = 640;
    const int target_inner_w = 512;
    double scale_w = static_cast<double>(target_inner_w) / static_cast<double>(gray.width);
    int new_w = target_inner_w;
    int new_h = static_cast<int>(std::round(gray.height * scale_w));
    if (new_h > target_h) {
      double scale_h = static_cast<double>(target_h) / static_cast<double>(gray.height);
      new_h = target_h;
      new_w = static_cast<int>(std::round(gray.width * scale_h));
    }
    new_w = std::max(1, std::min(new_w, target_w));
    new_h = std::max(1, std::min(new_h, target_h));
    GrayImage resized = resizeNearest(gray, new_w, new_h);
    GrayImage canvas = makeGrayImage(target_w, target_h, 0);
    int x0 = (target_w - new_w) / 2;
    int y0 = (target_h - new_h) / 2;
    pasteGray(resized, &canvas, x0, y0);
    return canvas;
  }

  std::optional<GrayImage> gridmapToGrayU8(const grid_map_msgs::GridMap& msg) {
    double res = msg.info.resolution;
    int cols = static_cast<int>(std::round(msg.info.length_x / res));
    int rows = static_cast<int>(std::round(msg.info.length_y / res));
    if (rows <= 0 || cols <= 0) {
      return std::nullopt;
    }
    int layer_idx = -1;
    for (size_t i = 0; i < msg.layers.size(); ++i) {
      if (msg.layers[i] == "grayscale") {
        layer_idx = static_cast<int>(i);
        break;
      }
    }
    if (layer_idx < 0 || layer_idx >= static_cast<int>(msg.data.size())) {
      return std::nullopt;
    }
    const auto& values = msg.data[layer_idx].data;
    int length = static_cast<int>(values.size());
    if (rows * cols != length) {
      int square = static_cast<int>(std::round(std::sqrt(length)));
      if (square * square == length) {
        rows = cols = square;
      } else if (cols > 0 && length % cols == 0) {
        rows = length / cols;
      } else {
        rows = 1;
        cols = length;
      }
    }
    GrayImage gray = makeGrayImage(cols, rows, 0);
    if (gray.empty()) {
      return std::nullopt;
    }
    for (int c = 0; c < cols; ++c) {
      for (int r = 0; r < rows; ++r) {
        int idx = c * rows + r;
        float v = idx < length ? values[idx] : 0.0f;
        if (!std::isfinite(v)) v = 0.0f;
        v = std::max(0.0f, std::min(255.0f, v));
        gray.data[static_cast<size_t>(r) * cols + c] = static_cast<uint8_t>(v);
      }
    }
    return gray;
  }

  std::optional<GrayImage> gridmapBrainCanvas(const GrayImage& gray) {
    if (gray.empty()) return std::nullopt;
    GrayImage gray360 = resizeNearest(gray, 360, 360);
    GrayImage canvas = makeGrayImage(640, 360, 0);
    pasteGray(gray360, &canvas, (640 - 360) / 2, 0);
    return canvas;
  }

  void gridmapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    markTopicReceived("/excavation_gridmap");
    auto gray = gridmapToGrayU8(*msg);
    if (!gray) {
      ROS_WARN_THROTTLE(5.0, "[gridmap] 解析 grayscale 层失败");
      return;
    }
    auto canvas = gridmapBrainCanvas(*gray);
    if (canvas) {
      auto b64 = encodeJpegB64(*canvas, jpeg_quality_);
      if (b64) {
        Json::Value payload(Json::objectValue);
        payload["ts"] = Json::Int64(static_cast<int64_t>(msg->info.header.stamp.toSec() * 1000.0));
        payload["w"] = 640;
        payload["h"] = 360;
        payload["format"] = "jpeg_base64";
        payload["data"] = *b64;
        brain_post_worker_.submit("gridmap", brain_gridmap_url_, jsonToString(payload), http_timeout_);
      }
    }
    auto dirs = currentImageDirs();
    if (dirs.first.empty()) return;
    if (!canWriteNewData()) {
      logLowSpace("gridmap");
      return;
    }
    auto png = encodePngBytes(*gray);
    if (png) {
      int64_t ts_ms = static_cast<int64_t>(msg->info.header.stamp.toSec() * 1000.0);
      writeBytesAtomic(dirs.first / (std::to_string(ts_ms) + ".png"), *png);
    }
  }

  void grayRangeImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    markTopicReceived("/gray_range_image");
    auto gray = imageToGrayU8(*msg);
    if (!gray) {
      ROS_WARN_THROTTLE(5.0, "[range] 解析 /gray_range_image 失败");
      return;
    }
    auto canvas = resizeRangeCanvas(*gray);
    if (!canvas) {
      ROS_WARN_THROTTLE(5.0, "[range] resize/pad 失败");
      return;
    }
    auto b64 = encodeJpegB64(*canvas, jpeg_quality_);
    int64_t ts_ms = static_cast<int64_t>((msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec()) * 1000.0);
    if (b64) {
      Json::Value payload(Json::objectValue);
      payload["ts"] = Json::Int64(ts_ms);
      payload["w"] = 640;
      payload["h"] = 360;
      payload["format"] = "jpeg_base64";
      payload["data"] = *b64;
      brain_post_worker_.submit("range", brain_range_url_, jsonToString(payload), http_timeout_);
    }
    auto dirs = currentImageDirs();
    if (dirs.second.empty()) return;
    if (!canWriteNewData()) {
      logLowSpace("grayRangeImage");
      return;
    }
    auto png = encodePngBytes(*canvas);
    if (png) {
      writeBytesAtomic(dirs.second / ("range_" + std::to_string(ts_ms) + ".png"), *png);
    }
  }

  void handleBrainStatus(const std::string& kind, bool ok, const HttpResponse& response) {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (kind == "state") status_state_ok_ = ok;
      else if (kind == "range") status_range_ok_ = ok;
      else if (kind == "gridmap") status_gridmap_ok_ = ok;
    }
    if (kind == "state" && ok) {
      Json::Value body;
      if (parseJson(response.body, &body) && body.isObject() && body.isMember("excavator_data_joystick")) {
        Json::Value joystick = excavatorDataJoystickJson(body["excavator_data_joystick"]);
        std::lock_guard<std::mutex> lock(data_mutex_);
        data_.excavator_data_joystick = joystick;
      }
    }
    if (kind == "gridmap" && ok) {
      Json::Value body;
      if (parseJson(response.body, &body) && body.isObject() && body.isMember("action")) {
        publishPredActionAsPose(body["action"]);
      } else {
        ROS_WARN_THROTTLE(5.0, "[graymap] 返回 JSON 中未找到 'action' 字段");
      }
    }
    computeAndPublishStatus();
  }

  void publishPredActionAsPose(const Json::Value& action_obj) {
    std::vector<double> arr;
    if (action_obj.isArray()) {
      for (const auto& v : action_obj) arr.push_back(v.asDouble());
    } else if (action_obj.isObject()) {
      const Json::Value* source = nullptr;
      if (action_obj.isMember("action") && action_obj["action"].isArray()) source = &action_obj["action"];
      else if (action_obj.isMember("data") && action_obj["data"].isArray()) source = &action_obj["data"];
      if (source) {
        for (const auto& v : *source) arr.push_back(v.asDouble());
      } else {
        std::vector<int> keys;
        for (const auto& key : action_obj.getMemberNames()) {
          try { keys.push_back(std::stoi(key)); } catch (...) {}
        }
        std::sort(keys.begin(), keys.end());
        for (int key : keys) arr.push_back(action_obj[std::to_string(key)].asDouble());
      }
    }
    if (arr.size() < 4) {
      ROS_WARN_THROTTLE(5.0, "[action] 无法从返回值中解析出 4 个元素");
      return;
    }
    geometry_msgs::Pose pose;
    pose.orientation.x = arr[3];
    pose.orientation.y = arr[0];
    pose.orientation.z = arr[1];
    pose.orientation.w = arr[2];
    pred_action_pub_.publish(pose);
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_.smolvla_output_angle = pose;
  }

  void computeAndPublishStatus() {
    bool state_ok, grid_ok, range_ok;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      state_ok = status_state_ok_;
      grid_ok = status_gridmap_ok_;
      range_ok = status_range_ok_;
    }
    const bool cam0_ok = cam0_http_status_.load() != 0;
    const bool cam1_ok = cam1_http_status_.load() != 0;

    std_msgs::Int8 msg;
    if (!state_ok && !grid_ok && !range_ok && !cam0_ok && !cam1_ok) {
      msg.data = 0;
    } else if (state_ok && grid_ok && range_ok && cam0_ok && cam1_ok) {
      msg.data = 1;
    } else if (!state_ok) {
      msg.data = 2;
    } else if (!grid_ok) {
      msg.data = 3;
    } else if (!range_ok) {
      msg.data = 4;
    } else if (!cam0_ok) {
      msg.data = 5;
    } else {
      msg.data = 6;
    }
    smolvla_status_pub_.publish(msg);
  }

  void heartbeatTimerCb(const ros::TimerEvent&) {
    int st0 = cam0_stream_status_.load();
    int st1 = cam1_stream_status_.load();
    uint8_t error_code = 0;
    if (st0) error_code |= 1 << 0;
    if (st1) error_code |= 1 << 1;
    std_msgs::UInt32 msg;
    msg.data = packSafety(error_code, heartbeat_counter_);
    camera_safety_pub_.publish(msg);
    // ROS_INFO_THROTTLE(1.0,
    //                   "CameraSafety cam0_error_code=%u cam1_error_code=%u error_code=0x%02X heart=%u packed_data=0x%08X",
    //                   st0 ? 1u : 0u, st1 ? 1u : 0u, static_cast<unsigned int>(error_code),
    //                   static_cast<unsigned int>(heartbeat_counter_), static_cast<unsigned int>(msg.data));
    heartbeat_counter_ = static_cast<uint8_t>((heartbeat_counter_ + 1) % 256);
  }

  void diagnosticTimerCb(const ros::TimerEvent&) {
    bool recording = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      recording = recording_;
    }
    if (!recording) return;
    std::vector<std::string> missing;
    double rtk_age = -1.0;
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      double now = monotonicSeconds();
      if (!topic_diagnostics_active_ || now - topic_diagnostics_started_at_ < 3.0) return;
      for (const auto& kv : topic_monitors_) {
        if (!kv.second && warnDue(last_topic_warn_at_[kv.first])) {
          last_topic_warn_at_[kv.first] = now;
          missing.push_back(kv.first);
        }
      }
      if (last_rtk_monotonic_ts_ > 0.0) {
        rtk_age = now - last_rtk_monotonic_ts_;
      }
    }
    for (const auto& topic : missing) {
      ROS_WARN("\033[1;31m[DATA WARNING] topic no data: %s\033[0m", topic.c_str());
    }
    if (rtk_age >= 5.0 && warnDue(last_rtk_no_data_warn_at_)) {
      last_rtk_no_data_warn_at_ = monotonicSeconds();
      ROS_WARN("\033[1;31m[DATA WARNING] /rtk_can_msg no new data for %.1fs; JSON state will stop growing\033[0m", rtk_age);
    }
  }

  void stateJsonStatusTimerCb(const ros::TimerEvent&) {
    bool recording = false;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      recording = recording_;
    }
    if (!recording) return;
    Json::Value status = state_json_writer_.status();
    ROS_INFO("state JSON status. tmp=%s lines=%lld", status["tmp_path"].asCString(), status["lines_written"].asInt64());
  }

  void recoverOrphanStateJsonTmpFiles() {
    int count = 0;
    for (const auto& dataset : listDatasetDirs(scan_root_)) {
      for (const auto& root : listRecordingRoots(dataset.second)) {
        fs::path bag_dir = root.second / "bag";
        std::error_code ec;
        if (!fs::is_directory(bag_dir, ec)) continue;
        for (const auto& entry : fs::directory_iterator(bag_dir, ec)) {
          if (!entry.is_regular_file(ec)) continue;
          if (entry.path().extension() == ".tmp" && entry.path().string().find(".json.tmp") != std::string::npos) {
            fs::path final_path = entry.path();
            std::string s = final_path.string();
            s.resize(s.size() - 4);
            final_path = s;
            std::error_code rec;
            fs::rename(entry.path(), final_path, rec);
            if (!rec) ++count;
          }
        }
      }
    }
    if (count) ROS_WARN("recovered orphan state JSON tmp files: %d", count);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string brain_base_url_, brain_state_url_, brain_gridmap_url_, brain_range_url_, brain_cam0_url_, brain_cam1_url_;
  double http_timeout_ = 0.8;
  int http_session_recreate_failures_ = kDefaultSessionRecreateFailures;
  int jpeg_quality_ = 90;
  fs::path scan_root_;
  uint64_t min_free_bytes_ = kMinFreeBytes;
  uint64_t max_scan_root_bytes_ = kDefaultMaxScanRootBytes;
  int local_image_queue_max_ = 1000;
  std::string machine_model_, operation_mode_, location_, weather_;
  std::string model_id_, trajectory_fusion_code_version_, control_code_version_;
  double delta_vla_height_ = 0.0;
  int state_flush_every_ = 20;
  double state_flush_interval_s_ = 0.5;
  double state_fsync_interval_s_ = 5.0;
  int state_json_queue_max_ = 20000;

  mutable std::mutex data_mutex_;
  DataCache data_;
  std::mutex session_mutex_;
  std::mutex save_signal_transition_mutex_;
  std::mutex diagnostic_mutex_;
  std::mutex status_mutex_;
  std::mutex low_space_log_mutex_;
  std::map<std::string, bool> topic_monitors_;
  std::map<std::string, double> last_topic_warn_at_;
  bool topic_diagnostics_active_ = false;
  double topic_diagnostics_started_at_ = 0.0;
  double last_rtk_monotonic_ts_ = 0.0;
  double last_rtk_no_data_warn_at_ = 0.0;
  double last_low_space_log_ts_ = 0.0;

  int frame_counter_ = 0;
  bool recording_ = false;
  int last_save_signal_ = 0;
  int raw_save_signal_ = 0;
  double raw_save_signal_changed_at_ = monotonicSeconds();
  double capture_switch_debounce_s_ = 1.0;
  int state_session_id_ = 0;
  std::time_t session_ts_sec_ = 0;
  std::string dataset_name_;
  fs::path dataset_root_, session_root_, bag_dir_, gray_dir_, range_dir_, json_tmp_path_, json_final_path_;

  bool status_state_ok_ = false;
  bool status_gridmap_ok_ = false;
  bool status_range_ok_ = false;
  uint8_t heartbeat_counter_ = 0;
  std::atomic<int> cam0_http_status_{0}, cam1_http_status_{0}, cam0_stream_status_{0}, cam1_stream_status_{0};
  double camera_started_at_[2] = {0.0, 0.0};

  ros::Publisher pred_action_pub_, smolvla_status_pub_, camera_safety_pub_;
  ros::Subscriber sub_angular_vel_, sub_bucket_local_, sub_bucket_rot_, sub_bucket_terrain_delta_, sub_joystick_;
  ros::Subscriber sub_imu_arm_, sub_imu_boom_, sub_imu_bucket_, sub_imu_vehicle_;
  ros::Subscriber sub_joints_angles_, sub_joints_velocity_, sub_swivel_angle_, sub_sys_seq_action_;
  ros::Subscriber sub_sys_sco_traj_, sub_fused_controller_joystick_, sub_rtk_, sub_gridmap_, sub_gray_range_, sub_fault_code_, sub_save_signal_;
  ros::Timer status_timer_, heartbeat_timer_, debounce_timer_, diagnostic_timer_, state_status_timer_;

  StateJsonWriter state_json_writer_;
  LatestPostWorker brain_post_worker_;
  std::unique_ptr<SessionZipUploader> session_uploader_;
  std::shared_ptr<SessionShared> session_shared_;
  std::shared_ptr<std::atomic<bool>> shutdown_flag_;
  std::vector<std::unique_ptr<CameraWorker>> camera_workers_;
  std::atomic<bool> shutdown_done_{false};

  friend class SessionZipUploader;
};

SessionZipUploader::SessionZipUploader(UnifiedDataNode* node)
    : node_(node), scan_root_(node->scanRoot()) {
  const std::string env_base = getenvString("SY200_NAS_BASE_URL");
  const std::string nas_base = node_->getParam<std::string>(
      "nas_webhook_url", env_base.empty() ? kDefaultNasBaseUrl : env_base);
  upload_init_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_init_path", "/upload_init"));
  upload_chunk_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_chunk_path", "/upload_chunk"));
  upload_complete_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_complete_path", "/upload_complete"));
  upload_status_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_status_path", "/upload_status"));
  upload_interval_s_ = std::max(1.0, node_->getParam<double>("upload_interval_s", 90.0));
  stop_flush_delay_s_ = std::max(0.0, node_->getParam<double>("stop_flush_delay_s", 10.0));
  chunk_size_ = static_cast<uint64_t>(std::max<double>(1.0, node_->getParam<double>(
      "chunk_size_bytes", static_cast<double>(kDefaultChunkSizeBytes))));
  alignment_tolerance_ms_ = std::max(0, node_->getParam<int>("alignment_tolerance_ms", 500));
  quality_error_detail_limit_ = static_cast<size_t>(std::max(
      1, node_->getParam<int>("quality_error_detail_limit", 200)));
  zip_binary_ = node_->getParam<std::string>("zip_binary", kDefaultZipBinary);
  unzip_binary_ = node_->getParam<std::string>("unzip_binary", kDefaultUnzipBinary);
  zip_compression_level_ = std::max(0, std::min(9, node_->getParam<int>("zip_compression_level", 1)));
  upload_parallel_chunks_ = std::max(1, node_->getParam<int>("upload_parallel_chunks", 4));
  upload_retry_count_ = std::max(0, node_->getParam<int>("upload_retry_count", 3));
  complete_wait_timeout_s_ = std::max(1.0, node_->getParam<double>("complete_wait_timeout_s", 1800.0));
  upload_status_interval_s_ = std::max(0.2, node_->getParam<double>("upload_status_interval_s", 5.0));
  cache_cleanup_interval_s_ = std::max(
      60.0, node_->getParam<double>("session_cache_cleanup_interval_s", 3600.0));
  zip_tmp_ttl_s_ = std::max(
      60.0, node_->getParam<double>("session_zip_tmp_ttl_s", 86400.0));
  status_orphan_ttl_s_ = std::max(
      60.0, node_->getParam<double>("session_status_orphan_ttl_s", 604800.0));
  zip_tmp_root_ = requireDirectoryUnder(node_->getParam<std::string>(
      "session_zip_tmp_root", (scan_root_ / ".session_zip_tmp").string()),
      scan_root_, "session_zip_tmp_root");
  status_root_ = requireDirectoryUnder(node_->getParam<std::string>(
      "session_upload_status_root", (scan_root_ / ".session_upload_status").string()),
      scan_root_, "session_upload_status_root");
  connect_timeout_s_ = node_->getParam<double>("connect_timeout_s", kDefaultNasConnectTimeoutS);
  request_timeout_s_ = node_->getParam<double>("request_timeout_s", kDefaultNasRequestTimeoutS);
  session_recreate_failures_ = std::max(1, node_->getParam<int>(
      "nas_session_recreate_failures", kDefaultSessionRecreateFailures));
  http_session_ = std::make_unique<NasHttpSession>();
}

void SessionZipUploader::start() {
  ensureDirOrThrow(zip_tmp_root_);
  ensureDirOrThrow(status_root_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = false;
    upload_work_pending_ = false;
    startup_recovery_complete_ = false;
    initial_quality_scan_complete_ = false;
    quality_scan_active_ = false;
    close_generation_ = 0;
    next_forced_quality_scan_at_.reset();
  }
  quality_thread_ = std::thread([this] { qualityLoop(); });
  upload_thread_ = std::thread([this] { uploadLoop(); });
  ROS_INFO("session ZIP quality and upload workers started. scan_root=%s parallel_chunks=%d "
           "chunk_size=%llu nas=%s",
           scan_root_.c_str(), upload_parallel_chunks_,
           static_cast<unsigned long long>(chunk_size_), upload_init_url_.c_str());
}

void SessionZipUploader::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    quality_cv_.notify_all();
    upload_cv_.notify_all();
  }
  if (quality_thread_.joinable()) quality_thread_.join();
  if (upload_thread_.joinable()) upload_thread_.join();
}

void SessionZipUploader::notifySessionClosed(const fs::path&) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++close_generation_;
  const double ready_at = monotonicSeconds() + stop_flush_delay_s_;
  if (!next_forced_quality_scan_at_ || ready_at < *next_forced_quality_scan_at_) {
    next_forced_quality_scan_at_ = ready_at;
  }
  quality_cv_.notify_one();
}

void SessionZipUploader::notifyCapacityPressure() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double ready_at = monotonicSeconds();
  if (!next_forced_quality_scan_at_ || ready_at < *next_forced_quality_scan_at_) {
    next_forced_quality_scan_at_ = ready_at;
  }
  quality_cv_.notify_one();
}

bool SessionZipUploader::runProgram(const std::vector<std::string>& args, const fs::path& cwd) const {
  if (args.empty()) return false;
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);
  const pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    if (!cwd.empty() && chdir(cwd.c_str()) != 0) _exit(126);
    execv(args.front().c_str(), argv.data());
    _exit(127);
  }
  int status = 0;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

bool SessionZipUploader::sha256File(const fs::path& path, std::string* digest) const {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (!ctx) return false;
  bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1;
  std::vector<char> buffer(1024 * 1024);
  while (ok && in) {
    in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize got = in.gcount();
    if (got > 0) ok = EVP_DigestUpdate(ctx, buffer.data(), static_cast<size_t>(got)) == 1;
  }
  unsigned char bytes[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (ok) ok = EVP_DigestFinal_ex(ctx, bytes, &length) == 1;
  EVP_MD_CTX_free(ctx);
  if (!ok) return false;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < length; ++i) out << std::setw(2) << static_cast<unsigned int>(bytes[i]);
  *digest = out.str();
  return true;
}

bool SessionZipUploader::sessionIsStable(const fs::path& session_root) const {
  std::error_code ec;
  if (!fs::is_directory(session_root, ec) || fs::is_symlink(fs::symlink_status(session_root, ec))) return false;
  auto newest = fs::last_write_time(session_root, ec);
  if (ec) return false;
  for (auto it = fs::recursive_directory_iterator(session_root, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    const auto stamp = fs::last_write_time(it->path(), ec);
    if (ec) return false;
    if (stamp > newest) newest = stamp;
  }
  return std::chrono::duration<double>(fs::file_time_type::clock::now() - newest).count() >= stop_flush_delay_s_;
}

SessionQualityResult SessionZipUploader::validateSession(const fs::path& session_root) const {
  SessionQualityResult result;
  const auto add_error = [&](std::string error) {
    ++result.total_error_count;
    if (result.errors.size() < quality_error_detail_limit_) {
      result.errors.push_back(std::move(error));
    } else {
      result.errors_truncated = true;
    }
  };
  const fs::path bag = session_root / "bag";
  const fs::path camera = session_root / "camera_data";
  const fs::path gray = session_root / "grayPicture";
  const fs::path range = session_root / "rangePicture";
  for (const auto& item : std::vector<std::pair<std::string, fs::path>>{
           {"bag", bag}, {"camera_data", camera}, {"grayPicture", gray}, {"rangePicture", range}}) {
    std::error_code ec;
    if (!fs::is_directory(item.second, ec)) add_error(item.first + " directory missing");
  }

  std::vector<int64_t> json_stamps, camera0_stamps, camera1_stamps, gray_stamps, range_stamps;
  std::vector<fs::path> json_files;
  std::error_code ec;
  if (fs::is_directory(session_root, ec)) {
    for (auto it = fs::recursive_directory_iterator(session_root, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
      const auto status = it->symlink_status(ec);
      if (ec) break;
      if (fs::is_symlink(status)) {
        add_error("symbolic link is not allowed: " + relativePosix(it->path(), session_root));
        if (fs::is_directory(status)) it.disable_recursion_pending();
        continue;
      }
      if (!fs::is_regular_file(status)) continue;
      ++result.archive_entries;
      const uint64_t size = fileSize(it->path());
      if (size == 0) add_error("zero-byte file: " + relativePosix(it->path(), session_root));
      std::string ext = it->path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (isPathUnder(it->path(), bag) && ext == ".json") json_files.push_back(it->path());
    }
  }
  if (ec) add_error("failed to enumerate session: " + ec.message());

  const std::regex frame_stamp_re(",\\s*([0-9]+(?:\\.[0-9]+)?)$");
  for (const auto& json_path : json_files) {
    std::ifstream in(json_path);
    std::string line;
    size_t line_number = 0;
    while (std::getline(in, line)) {
      ++line_number;
      if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
      Json::Value root;
      if (!parseJson(line, &root) || !root.isObject() || root.empty()) {
        add_error("invalid JSON line: " + json_path.filename().string() + ":" + std::to_string(line_number));
        continue;
      }
      for (const auto& key : root.getMemberNames()) {
        int64_t stamp_ms = 0;
        const Json::Value& payload = root[key];
        if (payload.isObject() && payload.isMember("rtk_stamp_ms") && payload["rtk_stamp_ms"].isNumeric()) {
          stamp_ms = payload["rtk_stamp_ms"].asInt64();
        } else {
          std::smatch match;
          if (std::regex_search(key, match, frame_stamp_re)) {
            try { stamp_ms = static_cast<int64_t>(std::llround(std::stod(match[1].str()) * 1000.0)); }
            catch (...) { stamp_ms = 0; }
          }
        }
        if (stamp_ms <= 0) add_error("JSON timestamp missing: " + key);
        else json_stamps.push_back(stamp_ms);
      }
    }
  }
  result.json_records = json_stamps.size();
  if (json_files.empty()) add_error("bag JSON file missing");
  if (json_stamps.empty()) add_error("bag JSON contains no valid timestamps");

  const std::regex camera_re("^([0-9]{10,16})\\.(?:jpg|jpeg)$", std::regex::icase);
  const std::regex gray_re("^([0-9]{10,16})\\.png$", std::regex::icase);
  const std::regex range_re("^range_([0-9]{10,16})\\.png$", std::regex::icase);
  const auto collect = [&](const fs::path& root, const std::regex& name_re,
                           const std::string& label, std::vector<int64_t>* first,
                           std::vector<int64_t>* second) {
    std::error_code rec;
    if (!fs::is_directory(root, rec)) return;
    for (auto it = fs::recursive_directory_iterator(root, rec); !rec && it != fs::recursive_directory_iterator(); it.increment(rec)) {
      if (!it->is_regular_file(rec)) continue;
      std::smatch match;
      const std::string name = it->path().filename().string();
      if (!std::regex_match(name, match, name_re)) {
        add_error(label + " timestamp cannot be parsed from filename: " +
                  relativePosix(it->path(), session_root));
        continue;
      }
      if (fileSize(it->path()) == 0) continue;
      int64_t stamp = 0;
      try { stamp = std::stoll(match[1].str()); } catch (...) { stamp = 0; }
      if (stamp <= 0) {
        add_error(label + " timestamp is invalid: " +
                  relativePosix(it->path(), session_root));
        continue;
      }
      if (!second) {
        first->push_back(stamp);
      } else {
        const std::string parent = it->path().parent_path().filename().string();
        if (parent.size() >= 9 && parent.rfind("_camera_0") == parent.size() - 9) first->push_back(stamp);
        else if (parent.size() >= 9 && parent.rfind("_camera_1") == parent.size() - 9) second->push_back(stamp);
        else add_error("camera image is outside camera_0/camera_1 directory: " +
                       relativePosix(it->path(), session_root));
      }
    }
    if (rec) add_error("failed to enumerate " + label + ": " + rec.message());
  };
  collect(camera, camera_re, "camera_data", &camera0_stamps, &camera1_stamps);
  collect(gray, gray_re, "grayPicture", &gray_stamps, nullptr);
  collect(range, range_re, "rangePicture", &range_stamps, nullptr);
  result.camera0_files = camera0_stamps.size();
  result.camera1_files = camera1_stamps.size();
  result.gray_files = gray_stamps.size();
  result.range_files = range_stamps.size();
  if (camera0_stamps.empty()) add_error("camera_0 has no valid images");
  if (camera1_stamps.empty()) add_error("camera_1 has no valid images");
  if (gray_stamps.empty()) add_error("grayPicture has no valid images");
  if (range_stamps.empty()) add_error("rangePicture has no valid images");

  std::sort(json_stamps.begin(), json_stamps.end());
  const auto check_alignment = [&](std::vector<int64_t>* images, const std::string& label, int64_t* max_delta) {
    if (images->empty() || json_stamps.empty()) return;
    std::sort(images->begin(), images->end());
    bool reported = false;
    for (const int64_t stamp : json_stamps) {
      auto it = std::lower_bound(images->begin(), images->end(), stamp);
      int64_t delta = std::numeric_limits<int64_t>::max();
      if (it != images->end()) delta = std::min<int64_t>(delta, static_cast<int64_t>(std::llabs(*it - stamp)));
      if (it != images->begin()) delta = std::min<int64_t>(delta, static_cast<int64_t>(std::llabs(*std::prev(it) - stamp)));
      *max_delta = std::max(*max_delta, delta);
      if (delta > alignment_tolerance_ms_ && !reported) {
        add_error(label + " alignment exceeds tolerance at JSON timestamp " +
                  std::to_string(stamp) + ", nearest_delta_ms=" + std::to_string(delta));
        reported = true;
      }
    }
  };
  check_alignment(&camera0_stamps, "camera_0", &result.max_camera0_delta_ms);
  check_alignment(&camera1_stamps, "camera_1", &result.max_camera1_delta_ms);
  check_alignment(&gray_stamps, "grayPicture", &result.max_gray_delta_ms);
  check_alignment(&range_stamps, "rangePicture", &result.max_range_delta_ms);
  result.qualified = result.total_error_count == 0;
  return result;
}

Json::Value SessionZipUploader::qualityJson(const SessionQualityResult& q) const {
  Json::Value out(Json::objectValue);
  out["qualified"] = q.qualified;
  out["total_error_count"] = Json::UInt64(q.total_error_count);
  out["errors_truncated"] = q.errors_truncated;
  out["error_detail_limit"] = Json::UInt64(quality_error_detail_limit_);
  out["json_records"] = Json::UInt64(q.json_records);
  out["camera0_files"] = Json::UInt64(q.camera0_files);
  out["camera1_files"] = Json::UInt64(q.camera1_files);
  out["gray_files"] = Json::UInt64(q.gray_files);
  out["range_files"] = Json::UInt64(q.range_files);
  out["archive_entries"] = Json::UInt64(q.archive_entries);
  out["max_camera0_delta_ms"] = Json::Int64(q.max_camera0_delta_ms);
  out["max_camera1_delta_ms"] = Json::Int64(q.max_camera1_delta_ms);
  out["max_gray_delta_ms"] = Json::Int64(q.max_gray_delta_ms);
  out["max_range_delta_ms"] = Json::Int64(q.max_range_delta_ms);
  Json::Value errors(Json::arrayValue);
  for (const auto& error : q.errors) errors.append(error);
  out["errors"] = errors;
  return out;
}

void SessionZipUploader::logEvent(const fs::path& dataset_root, const std::string& event,
                                  const Json::Value& details) const {
  const std::string dataset_name = dataset_root.filename().string();
  const fs::path expected = scan_root_ / dataset_name;
  if (!std::regex_match(dataset_name, kDatasetRe) ||
      fs::absolute(dataset_root).lexically_normal() != fs::absolute(expected).lexically_normal() ||
      !isPathUnder(dataset_root, scan_root_)) {
    ROS_ERROR("refusing to write capture log outside a managed dataset: %s", dataset_root.c_str());
    return;
  }
  Json::Value line(Json::objectValue);
  line["timestamp_ms"] = Json::Int64(unixMs());
  line["event"] = event;
  line["details"] = details;
  const std::string text_line = jsonToString(line) + "\n";
  std::lock_guard<std::mutex> lock(log_mutex_);
  std::error_code ec;
  const fs::file_status dataset_status = fs::symlink_status(dataset_root, ec);
  if (ec || !fs::is_directory(dataset_status) || fs::is_symlink(dataset_status)) {
    ROS_WARN("capture log skipped because managed dataset directory is unavailable: %s",
             dataset_root.c_str());
    return;
  }
  const fs::path path = dataset_root / "capture_upload.log";
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) return;
  size_t written = 0;
  while (written < text_line.size()) {
    const ssize_t count = ::write(fd, text_line.data() + written, text_line.size() - written);
    if (count < 0) {
      if (errno == EINTR) continue;
      break;
    }
    written += static_cast<size_t>(count);
  }
  ::fsync(fd);
  ::close(fd);
}

bool SessionZipUploader::createAndVerifyZip(const fs::path& session_root,
                                            const fs::path& archive_path,
                                            size_t* entry_count,
                                            std::string* error) const {
  std::error_code ec;
  if (!std::regex_match(session_root.filename().string(), kSessionDirRe) ||
      !std::regex_match(session_root.parent_path().filename().string(), kDatasetRe) ||
      !isPathUnder(session_root, scan_root_) || fs::is_symlink(fs::symlink_status(session_root, ec))) {
    if (error) *error = "unsafe session path";
    return false;
  }
  size_t count = 0;
  for (auto it = fs::recursive_directory_iterator(session_root, ec); !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (fs::is_symlink(it->symlink_status(ec))) {
      if (error) *error = "ZIP source contains symbolic link";
      return false;
    }
    if (it->is_regular_file(ec)) ++count;
  }
  if (ec || count == 0) {
    if (error) *error = ec ? ec.message() : "ZIP source contains no files";
    return false;
  }
  ensureDirOrThrow(zip_tmp_root_);
  const fs::path tmp = zip_tmp_root_ /
      ("." + session_root.filename().string() + "." + std::to_string(unixMs()) + ".tmp.zip");
  removeFileIfExists(tmp);
  const std::string level = "-" + std::to_string(zip_compression_level_);
  if (!runProgram({zip_binary_, "-q", level, "-r", tmp.string(), session_root.filename().string()},
                  session_root.parent_path()) ||
      !runProgram({unzip_binary_, "-tqq", tmp.string()})) {
    removeFileIfExists(tmp);
    if (error) *error = "zip creation or CRC verification failed";
    return false;
  }
  const int tmp_fd = ::open(tmp.c_str(), O_RDONLY);
  if (tmp_fd < 0 || ::fsync(tmp_fd) != 0) {
    if (tmp_fd >= 0) ::close(tmp_fd);
    removeFileIfExists(tmp);
    if (error) *error = "cannot fsync verified ZIP";
    return false;
  }
  ::close(tmp_fd);
  fs::rename(tmp, archive_path, ec);
  if (ec) {
    removeFileIfExists(tmp);
    if (error) *error = "cannot publish ZIP: " + ec.message();
    return false;
  }
  const int dir_fd = ::open(archive_path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd >= 0) {
    ::fsync(dir_fd);
    ::close(dir_fd);
  }
  if (entry_count) *entry_count = count;
  return true;
}

bool SessionZipUploader::validTaskPaths(const Json::Value& task,
                                        fs::path* dataset_root,
                                        fs::path* session_root,
                                        fs::path* archive_path) const {
  if (!task.isObject() || task.get("archive_kind", "").asString() != "session_zip") return false;
  const std::string dataset_name = task.get("dataset_name", "").asString();
  const std::string session_name = task.get("session_name", "").asString();
  if (!std::regex_match(dataset_name, kDatasetRe) ||
      !std::regex_match(session_name, kSessionDirRe) ||
      session_name.substr(0, 8) != dataset_name.substr(dataset_name.size() - 8)) return false;
  const fs::path expected_dataset = scan_root_ / dataset_name;
  const fs::path expected_session = expected_dataset / session_name;
  const std::string archive_name = task.get("archive_name", "").asString();
  if (archive_name != session_name + ".zip" && archive_name != session_name + "_Damaged.zip") return false;
  const fs::path expected_archive = expected_dataset / archive_name;
  const fs::path actual_dataset(task.get("dataset_root_abs", "").asString());
  const fs::path actual_session(task.get("session_root_abs", "").asString());
  const fs::path actual_archive(task.get("archive_path", "").asString());
  if (fs::absolute(actual_dataset).lexically_normal() != fs::absolute(expected_dataset).lexically_normal() ||
      fs::absolute(actual_session).lexically_normal() != fs::absolute(expected_session).lexically_normal() ||
      fs::absolute(actual_archive).lexically_normal() != fs::absolute(expected_archive).lexically_normal() ||
      !isPathUnder(expected_dataset, scan_root_)) return false;
  if (dataset_root) *dataset_root = expected_dataset;
  if (session_root) *session_root = expected_session;
  if (archive_path) *archive_path = expected_archive;
  return true;
}

bool SessionZipUploader::readTask(const fs::path& path, Json::Value* task) const {
  if (!task || !isPathUnder(path, status_root_)) return false;
  std::lock_guard<std::mutex> lock(status_mutex_);
  std::error_code ec;
  const fs::file_status status = fs::symlink_status(path, ec);
  if (ec || !fs::is_regular_file(status) || fs::is_symlink(status)) return false;
  return readJsonFile(path, task);
}

std::vector<fs::path> SessionZipUploader::listTaskFiles() const {
  std::vector<fs::path> files;
  std::lock_guard<std::mutex> lock(status_mutex_);
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(status_root_, ec)) {
    const fs::file_status status = entry.symlink_status(ec);
    if (ec) break;
    if (fs::is_regular_file(status) && !fs::is_symlink(status) &&
        entry.path().extension() == ".json" && isPathUnder(entry.path(), status_root_)) {
      files.push_back(entry.path());
    }
  }
  if (ec) ROS_WARN("session task enumeration failed: %s", ec.message().c_str());
  std::sort(files.begin(), files.end());
  return files;
}

bool SessionZipUploader::taskPathExists(const fs::path& path) const {
  if (!isPathUnder(path, status_root_)) return true;
  std::lock_guard<std::mutex> lock(status_mutex_);
  std::error_code ec;
  return fs::symlink_status(path, ec).type() != fs::file_type::not_found ||
      (ec && ec != std::errc::no_such_file_or_directory);
}

bool SessionZipUploader::tryClaimSession(const fs::path& session_root) {
  if (!std::regex_match(session_root.filename().string(), kSessionDirRe) ||
      !isPathUnder(session_root, scan_root_)) {
    return false;
  }
  std::error_code ec;
  const std::string key = fs::absolute(session_root, ec).lexically_normal().string();
  if (ec || key.empty()) return false;
  std::lock_guard<std::mutex> lock(claim_mutex_);
  auto it = active_session_claims_.find(key);
  const std::thread::id current = std::this_thread::get_id();
  if (it == active_session_claims_.end()) {
    active_session_claims_.emplace(key, std::make_pair(current, 1U));
    return true;
  }
  // Nested helpers on the same worker share the lifecycle claim.  Another
  // worker never waits while holding a status/filesystem lock; it skips this
  // session and retries it on the next scan.
  if (it->second.first == current) {
    ++it->second.second;
    return true;
  }
  return false;
}

void SessionZipUploader::releaseSession(const fs::path& session_root) {
  std::error_code ec;
  const std::string key = fs::absolute(session_root, ec).lexically_normal().string();
  if (ec || key.empty()) return;
  std::lock_guard<std::mutex> lock(claim_mutex_);
  auto it = active_session_claims_.find(key);
  if (it == active_session_claims_.end() ||
      it->second.first != std::this_thread::get_id()) {
    return;
  }
  if (--it->second.second == 0) active_session_claims_.erase(it);
}

void SessionZipUploader::scheduleQualityScanAfter(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double ready_at = monotonicSeconds() + std::max(0.1, seconds);
  if (!next_forced_quality_scan_at_ || ready_at < *next_forced_quality_scan_at_) {
    next_forced_quality_scan_at_ = ready_at;
  }
  quality_cv_.notify_one();
}

bool SessionZipUploader::waitForStop(double seconds) {
  std::unique_lock<std::mutex> lock(mutex_);
  return upload_cv_.wait_for(
      lock, std::chrono::duration<double>(std::max(0.0, seconds)),
      [this] { return stop_ || !ros::ok(); });
}

bool SessionZipUploader::datasetHasPendingTasks(const fs::path& dataset_root) const {
  const std::string dataset_name = dataset_root.filename().string();
  if (!std::regex_match(dataset_name, kDatasetRe)) return true;
  const std::string dataset_date = dataset_name.substr(dataset_name.size() - 8);
  const std::regex status_re("^([0-9]{14})\\.json(\\.tmp)?$");
  std::lock_guard<std::mutex> lock(status_mutex_);
  std::error_code ec;
  fs::directory_iterator it(status_root_, fs::directory_options::none, ec);
  for (; !ec && it != fs::directory_iterator(); it.increment(ec)) {
    std::smatch match;
    const std::string name = it->path().filename().string();
    if (std::regex_match(name, match, status_re) &&
        match[1].str().substr(0, 8) == dataset_date) {
      return true;
    }
  }
  if (ec) {
    ROS_WARN("completed dataset cleanup retained %s because upload task enumeration failed: %s",
             dataset_root.c_str(), ec.message().c_str());
    return true;
  }
  return false;
}

bool SessionZipUploader::cleanupCompletedDataset(const fs::path& dataset_root) {
  const std::string dataset_name = dataset_root.filename().string();
  const fs::path expected = scan_root_ / dataset_name;
  if (!std::regex_match(dataset_name, kDatasetRe) ||
      fs::absolute(dataset_root).lexically_normal() != fs::absolute(expected).lexically_normal() ||
      !isPathUnder(dataset_root, scan_root_)) {
    ROS_WARN("completed dataset cleanup refused unsafe path: %s", dataset_root.c_str());
    return false;
  }

  const std::string dataset_date = dataset_name.substr(dataset_name.size() - 8);
  const std::string current_date = timeFormat(std::time(nullptr), "%Y%m%d");
  if (dataset_date == current_date) return false;
  if (dataset_date > current_date) {
    ROS_WARN("completed dataset cleanup retained future-dated directory: %s current_date=%s",
             dataset_root.c_str(), current_date.c_str());
    return false;
  }

  std::lock_guard<std::mutex> session_lock(node_->session_mutex_);
  if (node_->recording_ &&
      fs::absolute(node_->dataset_root_).lexically_normal() ==
          fs::absolute(dataset_root).lexically_normal()) {
    return false;
  }
  if (datasetHasPendingTasks(dataset_root)) return false;

  std::lock_guard<std::mutex> log_lock(log_mutex_);
  std::error_code ec;
  const fs::file_status root_status = fs::symlink_status(dataset_root, ec);
  if (ec) {
    if (ec == std::errc::no_such_file_or_directory) return true;
    ROS_WARN("completed dataset cleanup cannot inspect %s: %s",
             dataset_root.c_str(), ec.message().c_str());
    return false;
  }
  if (!fs::is_directory(root_status) || fs::is_symlink(root_status)) {
    ROS_WARN("completed dataset cleanup retained non-directory or symbolic-link path: %s",
             dataset_root.c_str());
    return false;
  }

  bool has_capture_log = false;
  fs::directory_iterator it(dataset_root, fs::directory_options::none, ec);
  for (; !ec && it != fs::directory_iterator(); it.increment(ec)) {
    const fs::file_status entry_status = it->symlink_status(ec);
    if (ec) break;
    if (it->path().filename() == "capture_upload.log" &&
        fs::is_regular_file(entry_status) && !fs::is_symlink(entry_status)) {
      has_capture_log = true;
      continue;
    }
    ROS_WARN("completed dataset cleanup retained %s because it contains: %s",
             dataset_root.c_str(), it->path().filename().c_str());
    return false;
  }
  if (ec) {
    ROS_WARN("completed dataset cleanup cannot enumerate %s: %s",
             dataset_root.c_str(), ec.message().c_str());
    return false;
  }

  if (has_capture_log) {
    const fs::path log_path = dataset_root / "capture_upload.log";
    const bool removed = fs::remove(log_path, ec);
    if (ec || !removed) {
      ROS_WARN("completed dataset cleanup failed to remove %s: %s",
               log_path.c_str(), ec.message().c_str());
      return false;
    }
  }
  const bool removed = fs::remove(dataset_root, ec);
  if (ec || !removed) {
    ROS_WARN("completed dataset cleanup failed to remove empty directory %s: %s",
             dataset_root.c_str(), ec.message().c_str());
    return false;
  }
  ROS_INFO("completed historical dataset directory removed: %s", dataset_root.c_str());
  return true;
}

void SessionZipUploader::cleanupCompletedDatasets() {
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    cleanupCompletedDataset(dataset.second);
  }
}

void SessionZipUploader::cleanupSessionCacheArtifacts() {
  const auto age_seconds = [](const fs::path& path, double* age) {
    std::error_code ec;
    const auto stamp = fs::last_write_time(path, ec);
    if (ec) return false;
    *age = std::max(0.0, std::chrono::duration<double>(
        fs::file_time_type::clock::now() - stamp).count());
    return true;
  };
  const auto related_paths = [&](const std::string& session_name,
                                 fs::path* dataset_root) {
    if (!std::regex_match(session_name, kSessionDirRe)) return false;
    *dataset_root = scan_root_ / ("excavator_data_" + session_name.substr(0, 8));
    std::error_code ec;
    return fs::is_directory(*dataset_root / session_name, ec) ||
        fs::is_regular_file(*dataset_root / (session_name + ".zip"), ec) ||
        fs::is_regular_file(*dataset_root / (session_name + "_Damaged.zip"), ec);
  };
  const auto report_removed = [&](const std::string& session_name, const fs::path& path,
                                  uint64_t bytes, const std::string& kind) {
    ROS_WARN("session cache cleanup removed %s: %s bytes=%llu", kind.c_str(), path.c_str(),
             static_cast<unsigned long long>(bytes));
    fs::path dataset_root;
    std::error_code ec;
    if (std::regex_match(session_name, kSessionDirRe)) {
      dataset_root = scan_root_ / ("excavator_data_" + session_name.substr(0, 8));
    }
    if (!dataset_root.empty() && fs::is_directory(dataset_root, ec)) {
      Json::Value details(Json::objectValue);
      details["session"] = session_name;
      details["path"] = path.filename().string();
      details["bytes"] = Json::UInt64(bytes);
      details["cache_kind"] = kind;
      logEvent(dataset_root, "cache_cleanup", details);
    }
  };

  const std::regex tmp_zip_re("^\\.([0-9]{14})\\.[0-9]+\\.tmp\\.zip$");
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(zip_tmp_root_, ec)) {
    const fs::file_status status = entry.symlink_status(ec);
    if (ec) break;
    if (fs::is_symlink(status)) {
      ROS_WARN("session cache cleanup retained symbolic link: %s", entry.path().c_str());
      continue;
    }
    if (!fs::is_regular_file(status) || !isPathUnder(entry.path(), zip_tmp_root_)) continue;
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (!std::regex_match(name, match, tmp_zip_re)) continue;
    const std::string session_name = match[1].str();
    SessionClaim claim(this, scan_root_ / ("excavator_data_" + session_name.substr(0, 8)) /
                               session_name);
    if (!claim) continue;
    double age = 0.0;
    if (!age_seconds(entry.path(), &age) || age < zip_tmp_ttl_s_) continue;
    const uint64_t bytes = fileSize(entry.path());
    fs::remove(entry.path(), ec);
    if (!ec) report_removed(session_name, entry.path(), bytes, "orphan_session_zip_tmp");
    else {
      ROS_WARN("session ZIP tmp cleanup failed: %s error=%s", entry.path().c_str(), ec.message().c_str());
      ec.clear();
    }
  }
  if (ec) {
    ROS_WARN("session ZIP tmp enumeration failed: %s", ec.message().c_str());
    ec.clear();
  }

  const std::regex status_re("^([0-9]{14})\\.json$");
  const std::regex status_tmp_re("^([0-9]{14})\\.json\\.tmp$");
  std::vector<fs::path> status_paths;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (const auto& entry : fs::directory_iterator(status_root_, ec)) {
      const fs::file_status status = entry.symlink_status(ec);
      if (ec) break;
      if (fs::is_symlink(status)) {
        ROS_WARN("session status cleanup retained symbolic link: %s", entry.path().c_str());
        continue;
      }
      if (fs::is_regular_file(status) && isPathUnder(entry.path(), status_root_)) {
        status_paths.push_back(entry.path());
      }
    }
  }
  if (ec) {
    ROS_WARN("session status enumeration failed: %s", ec.message().c_str());
    ec.clear();
  }
  for (const auto& status_path : status_paths) {
    const std::string name = status_path.filename().string();
    std::smatch match;
    const bool is_tmp = std::regex_match(name, match, status_tmp_re);
    if (!is_tmp && !std::regex_match(name, match, status_re)) continue;
    const std::string session_name = match[1].str();
    const fs::path managed_session = scan_root_ /
        ("excavator_data_" + session_name.substr(0, 8)) / session_name;
    SessionClaim claim(this, managed_session);
    if (!claim) continue;
    fs::path dataset_root;
    const bool has_related_data = related_paths(session_name, &dataset_root);
    Json::Value task;
    fs::path task_dataset, task_session, task_archive;
    double age = 0.0;
    bool valid = false;
    bool recovered = false;
    bool removed = false;
    uint64_t removed_bytes = 0;
    std::string removed_kind;
    fs::path recovered_path;
    {
      // The claim is acquired first; this lock then covers only status-file
      // inspection/recovery/removal and never ZIP verification, hashing or NAS I/O.
      std::lock_guard<std::mutex> lock(status_mutex_);
      const fs::file_status status = fs::symlink_status(status_path, ec);
      if (ec || fs::is_symlink(status) || !fs::is_regular_file(status)) {
        ec.clear();
        continue;
      }
      if (!age_seconds(status_path, &age)) continue;
      valid = readJsonFile(status_path, &task) &&
          validTaskPaths(task, &task_dataset, &task_session, &task_archive) &&
          task.get("session_name", "").asString() == session_name;
      if (is_tmp && valid) {
        fs::path final_path = status_path;
        final_path.replace_extension();
        const bool final_exists = fs::exists(final_path, ec);
        if (!ec && !final_exists) {
          fs::rename(status_path, final_path, ec);
          if (!ec) {
            recovered = true;
            recovered_path = final_path;
          } else {
            ROS_WARN("session status recovery failed: %s error=%s",
                     status_path.c_str(), ec.message().c_str());
            ec.clear();
          }
        } else if (!ec && age >= zip_tmp_ttl_s_) {
          removed_bytes = fileSize(status_path);
          removed = fs::remove(status_path, ec) && !ec;
          removed_kind = "stale_status_tmp";
          ec.clear();
        } else {
          ec.clear();
        }
      } else if (!has_related_data && age >= status_orphan_ttl_s_) {
        removed_bytes = fileSize(status_path);
        removed = fs::remove(status_path, ec) && !ec;
        removed_kind = is_tmp ? "orphan_status_tmp" : "orphan_status";
        ec.clear();
      }
    }
    if (recovered) {
      ROS_WARN("recovered interrupted session status write: %s", recovered_path.c_str());
      if (fs::is_directory(task_dataset, ec)) {
        Json::Value details(Json::objectValue);
        details["session"] = session_name;
        details["path"] = recovered_path.filename().string();
        logEvent(task_dataset, "cache_status_recovered", details);
      }
    } else if (removed) {
      report_removed(session_name, status_path, removed_bytes, removed_kind);
    } else if (!valid && has_related_data && age >= status_orphan_ttl_s_) {
      ROS_WARN("invalid session status retained because source/ZIP exists: %s",
               status_path.c_str());
    }
  }
  cleanupCompletedDatasets();
}

bool SessionZipUploader::safeRemoveSession(const fs::path& session_root) const {
  std::error_code ec;
  const fs::path dataset_root = session_root.parent_path();
  const std::string dataset_name = dataset_root.filename().string();
  const std::string session_name = session_root.filename().string();
  if (!std::regex_match(dataset_name, kDatasetRe) || !std::regex_match(session_name, kSessionDirRe) ||
      session_name.substr(0, 8) != dataset_name.substr(dataset_name.size() - 8) ||
      !isPathUnder(session_root, scan_root_)) return false;
  {
    std::lock_guard<std::mutex> lock(node_->session_mutex_);
    if (node_->recording_ && fs::absolute(node_->session_root_).lexically_normal() ==
                                 fs::absolute(session_root).lexically_normal()) return false;
  }
  if (!fs::exists(session_root, ec)) return !ec;
  const fs::file_status root_status = fs::symlink_status(session_root, ec);
  if (ec || !fs::is_directory(root_status) || fs::is_symlink(root_status)) return false;
  for (auto it = fs::recursive_directory_iterator(session_root, ec);
       !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    const fs::file_status status = it->symlink_status(ec);
    if (ec || fs::is_symlink(status)) return false;
  }
  if (ec) return false;
  fs::remove_all(session_root, ec);
  return !ec && !fs::exists(session_root);
}

bool SessionZipUploader::saveTask(Json::Value* task) {
  (*task)["updated_at"] = static_cast<double>(unixMs()) / 1000.0;
  const fs::path path((*task)["status_path"].asString());
  Json::Value payload = *task;
  payload.removeMember("status_path");
  if (!isPathUnder(path, status_root_)) return false;
  const std::string serialized = jsonToString(payload) + "\n";
  std::lock_guard<std::mutex> lock(status_mutex_);
  return writeTextAtomic(path, serialized);
}

void SessionZipUploader::removeTask(const Json::Value& task) {
  const fs::path path(task.get("status_path", "").asString());
  if (path.empty() || !isPathUnder(path, status_root_)) return;
  std::lock_guard<std::mutex> lock(status_mutex_);
  removeFileIfExists(path);
}

bool SessionZipUploader::processSession(const fs::path& session_root) {
  SessionClaim claim(this, session_root);
  if (!claim) return true;
  // A task may have appeared after scanAndProcessSessions() made its initial
  // check (for example during startup recovery).  Recheck under the lifecycle
  // claim before reading source data or publishing a replacement ZIP.
  if (sessionHasQueuedArchive(session_root)) return true;
  const fs::path dataset_root = session_root.parent_path();
  const std::string session_name = session_root.filename().string();
  SessionQualityResult quality = validateSession(session_root);
  Json::Value quality_details = qualityJson(quality);
  quality_details["session"] = session_name;
  logEvent(dataset_root, quality.qualified ? "validation_passed" : "validation_failed", quality_details);

  const fs::path archive_path = dataset_root /
      (session_name + (quality.qualified ? ".zip" : "_Damaged.zip"));
  size_t entry_count = 0;
  std::string archive_error;
  if (!createAndVerifyZip(session_root, archive_path, &entry_count, &archive_error)) {
    Json::Value details(Json::objectValue);
    details["session"] = session_name;
    details["error"] = archive_error;
    logEvent(dataset_root, "zip_failed", details);
    return false;
  }
  std::string digest;
  if (!sha256File(archive_path, &digest)) {
    Json::Value details(Json::objectValue);
    details["session"] = session_name;
    details["archive"] = archive_path.filename().string();
    details["error"] = "cannot calculate ZIP SHA-256";
    logEvent(dataset_root, "zip_failed", details);
    return false;
  }
  Json::Value task(Json::objectValue);
  task["archive_kind"] = "session_zip";
  task["dataset_name"] = dataset_root.filename().string();
  task["dataset_root_abs"] = dataset_root.string();
  task["session_name"] = session_name;
  task["session_root_abs"] = session_root.string();
  task["archive_path"] = archive_path.string();
  task["archive_name"] = archive_path.filename().string();
  task["archive_size"] = Json::UInt64(fileSize(archive_path));
  task["archive_sha256"] = digest;
  task["archive_entries"] = Json::UInt64(entry_count);
  task["total_chunks"] = Json::UInt64(std::max<uint64_t>(1, (fileSize(archive_path) + chunk_size_ - 1) / chunk_size_));
  task["upload_id"] = "session_zip_" + dataset_root.filename().string() + "_" + session_name + "_" + digest.substr(0, 24);
  task["status_path"] = (status_root_ / (session_name + ".json")).string();
  task["state"] = quality.qualified ? "archive_ready" : "damaged_archive_ready";
  task["created_at"] = static_cast<double>(unixMs()) / 1000.0;
  task["quality"] = quality_details;
  if (!saveTask(&task)) return false;
  Json::Value zip_details(Json::objectValue);
  zip_details["session"] = session_name;
  zip_details["archive"] = archive_path.filename().string();
  zip_details["size"] = task["archive_size"];
  zip_details["sha256"] = digest;
  zip_details["entries"] = Json::UInt64(entry_count);
  zip_details["is_damaged"] = !quality.qualified;
  logEvent(dataset_root, "zip_created", zip_details);

  if (!quality.qualified) {
    if (!safeRemoveSession(session_root)) {
      task["state"] = "damaged_cleanup_failed";
      task["last_error"] = "refused or failed to remove damaged source session";
      saveTask(&task);
      Json::Value cleanup_details = zip_details;
      cleanup_details["error"] = task["last_error"];
      logEvent(dataset_root, "local_cleanup_failed", cleanup_details);
      return false;
    }
    logEvent(dataset_root, "damaged_source_removed", zip_details);
    task["state"] = "damaged_waiting_upload";
    task["last_error"] = Json::nullValue;
    if (!saveTask(&task)) return false;
    logEvent(dataset_root, "damaged_upload_queued", zip_details);
    notifyUploadWork();
    return true;
  }
  // Upload is deliberately delegated to uploadLoop().  Keeping this path free
  // of NAS I/O lets the quality worker continue with every other closed
  // session even when the oldest archive is slow or impossible to upload.
  notifyUploadWork();
  return true;
}

bool SessionZipUploader::taskIsDamaged(const Json::Value& task) const {
  const std::string session_name = task.get("session_name", "").asString();
  return std::regex_match(session_name, kSessionDirRe) &&
      task.get("archive_name", "").asString() == session_name + "_Damaged.zip";
}

bool SessionZipUploader::sessionHasQueuedArchive(const fs::path& session_root) const {
  const std::string session_name = session_root.filename().string();
  if (!std::regex_match(session_name, kSessionDirRe) ||
      !isPathUnder(session_root, scan_root_)) {
    return false;
  }
  const fs::path status_path = status_root_ / (session_name + ".json");
  Json::Value task;
  if (!readTask(status_path, &task)) return false;
  task["status_path"] = status_path.string();

  fs::path managed_session, archive_path;
  if (!validTaskPaths(task, nullptr, &managed_session, &archive_path) ||
      fs::absolute(managed_session).lexically_normal() !=
          fs::absolute(session_root).lexically_normal()) {
    return false;
  }
  std::error_code ec;
  const fs::file_status archive_status = fs::symlink_status(archive_path, ec);
  const std::string digest = task.get("archive_sha256", "").asString();
  return !ec && fs::is_regular_file(archive_status) && !fs::is_symlink(archive_status) &&
      fileSize(archive_path) == task.get("archive_size", Json::UInt64(0)).asUInt64() &&
      digest.size() == 64;
}

bool SessionZipUploader::hasPendingQualitySessions() const {
  fs::path active_session;
  {
    std::lock_guard<std::mutex> lock(node_->session_mutex_);
    if (node_->recording_) active_session = node_->session_root_;
  }
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    for (const auto& session : listSessionDirs(dataset.second)) {
      if (!active_session.empty() &&
          fs::absolute(active_session).lexically_normal() ==
              fs::absolute(session.second).lexically_normal()) {
        continue;
      }
      // Even an as-yet unstable closed directory counts as pending.  This
      // keeps historical damaged uploads behind newly captured data until the
      // quality worker has published a durable archive task for it.
      if (!sessionHasQueuedArchive(session.second)) return true;
    }
  }
  return false;
}

bool SessionZipUploader::hasPendingNormalTasks() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(status_root_, ec)) {
    const fs::file_status status = entry.symlink_status(ec);
    if (ec) break;
    if (!fs::is_regular_file(status) || fs::is_symlink(status) ||
        entry.path().extension() != ".json" || !isPathUnder(entry.path(), status_root_)) {
      continue;
    }
    Json::Value task;
    if (readJsonFile(entry.path(), &task) &&
        task.get("archive_kind", "").asString() == "session_zip" &&
        !taskIsDamaged(task)) {
      return true;
    }
  }
  if (ec) {
    // Enumeration uncertainty is treated conservatively: do not let damaged
    // work get ahead of a normal task that could not be inspected.
    ROS_WARN("normal session task enumeration failed: %s", ec.message().c_str());
    return true;
  }
  return false;
}

bool SessionZipUploader::damagedUploadsMayStart() {
  uint64_t observed_close_generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || !startup_recovery_complete_ ||
        !initial_quality_scan_complete_ || quality_scan_active_ ||
        next_forced_quality_scan_at_) {
      return false;
    }
    observed_close_generation = close_generation_;
  }
  if (hasPendingQualitySessions()) return false;
  // Linearize the damaged decision against notifySessionClosed().  A close
  // arriving during the filesystem scan increments the generation and also
  // installs a forced quality deadline, so the second check rejects the stale
  // result.  Once this lock is released, a later close is ordered after the
  // decision; an already-started damaged upload is intentionally not preempted.
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_ || !startup_recovery_complete_ ||
      !initial_quality_scan_complete_ || quality_scan_active_ ||
      next_forced_quality_scan_at_ ||
      close_generation_ != observed_close_generation) {
    return false;
  }
  return !hasPendingNormalTasks();
}

void SessionZipUploader::notifyUploadWork() {
  std::lock_guard<std::mutex> lock(mutex_);
  upload_work_pending_ = true;
  upload_cv_.notify_one();
}

bool SessionZipUploader::validateCompletedResponse(const Json::Value& task,
                                                   const Json::Value& response) const {
  const std::string expected_relative = task["dataset_name"].asString() + "/" +
      task["archive_name"].asString();
  return response.get("state", "").asString() == "completed" &&
      response.get("archive_kind", "").asString() == "session_zip" &&
      response.get("archive_name", "").asString() == task["archive_name"].asString() &&
      response.get("archive_sha256", "").asString() == task["archive_sha256"].asString() &&
      response.get("archive_size", Json::UInt64(0)).asUInt64() == task["archive_size"].asUInt64() &&
      response.get("archive_relative_path", "").asString() == expected_relative &&
      response.get("entries_verified", Json::UInt64(0)).asUInt64() > 0 &&
      response.get("is_damaged", false).asBool() == taskIsDamaged(task);
}

bool SessionZipUploader::waitForCompletion(const Json::Value& task, Json::Value* completed) {
  const double deadline = monotonicSeconds() + complete_wait_timeout_s_;
  while (ros::ok() && monotonicSeconds() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_) return false;
    }
    const HttpResponse response = http_session_->getJson(
        upload_status_url_ + "/" + task["upload_id"].asString(), connect_timeout_s_, request_timeout_s_);
    Json::Value status;
    if (response.ok && parseJson(response.body, &status)) {
      if (validateCompletedResponse(task, status)) {
        *completed = status;
        return true;
      }
      if (status.get("state", "").asString() == "error") return false;
    }
    if (waitForStop(upload_status_interval_s_)) return false;
  }
  return false;
}

bool SessionZipUploader::uploadTask(Json::Value* task) {
  const fs::path archive_path((*task)["archive_path"].asString());
  const fs::path session_root((*task)["session_root_abs"].asString());
  SessionClaim claim(this, session_root);
  if (!claim) return false;
  const fs::path dataset_root((*task)["dataset_root_abs"].asString());
  const std::string dataset_name = (*task)["dataset_name"].asString();
  const std::string session_name = (*task)["session_name"].asString();
  const bool damaged = taskIsDamaged(*task);
  const fs::path expected_dataset = scan_root_ / dataset_name;
  const fs::path expected_session = expected_dataset / session_name;
  const std::string expected_archive_name =
      session_name + (damaged ? "_Damaged.zip" : ".zip");
  const fs::path expected_archive = expected_dataset / expected_archive_name;
  if (!std::regex_match(dataset_name, kDatasetRe) || !std::regex_match(session_name, kSessionDirRe) ||
      session_name.substr(0, 8) != dataset_name.substr(dataset_name.size() - 8) ||
      fs::absolute(dataset_root).lexically_normal() != fs::absolute(expected_dataset).lexically_normal() ||
      fs::absolute(session_root).lexically_normal() != fs::absolute(expected_session).lexically_normal() ||
      fs::absolute(archive_path).lexically_normal() != fs::absolute(expected_archive).lexically_normal() ||
      (*task)["archive_name"].asString() != expected_archive_name ||
      !isPathUnder(dataset_root, scan_root_) ||
      !fs::is_regular_file(archive_path) || !isPathUnder(archive_path, dataset_root) ||
      fileSize(archive_path) != (*task)["archive_size"].asUInt64()) return false;
  std::string current_digest;
  const bool digest_ok = sha256File(archive_path, &current_digest) &&
      current_digest == (*task)["archive_sha256"].asString();
  if (!digest_ok) {
    std::error_code source_ec;
    const fs::file_status source_status = fs::symlink_status(session_root, source_ec);
    if (!damaged && !source_ec && fs::is_directory(source_status) &&
        !fs::is_symlink(source_status)) {
      // Size alone cannot prove that a cached ZIP still matches its task.  If
      // the qualified source is available, discard only the stale task marker
      // and let the quality worker atomically replace/re-hash the archive.
      // Retaining the source preserves recoverability if that rebuild fails.
      Json::Value details(Json::objectValue);
      details["session"] = session_name;
      details["archive"] = archive_path.filename().string();
      details["expected_sha256"] = (*task)["archive_sha256"];
      details["actual_sha256"] = current_digest;
      details["error"] = "qualified ZIP SHA-256 mismatch; scheduling rebuild";
      logEvent(dataset_root, "local_archive_verification_failed", details);
      removeTask(*task);
      scheduleQualityScanAfter(0.1);
    }
    return false;
  }

  Json::Value init(Json::objectValue);
  for (const auto& key : {"upload_id", "archive_kind", "dataset_name", "session_name", "archive_name", "archive_sha256", "archive_entries"}) {
    const std::string source_key = std::string(key) == "dataset_name" ? "dataset_name" : key;
    init[std::string(key) == "dataset_name" ? "dataset_root" : key] = (*task)[source_key];
  }
  init["total_size"] = (*task)["archive_size"];
  init["chunk_size"] = Json::UInt64(chunk_size_);
  init["total_chunks"] = (*task)["total_chunks"];
  HttpResponse response = http_session_->postJson(upload_init_url_, jsonToString(init), connect_timeout_s_, request_timeout_s_);
  Json::Value remote;
  if (!response.ok || !parseJson(response.body, &remote)) {
    (*task)["state"] = "upload_init_failed";
    (*task)["last_error"] = httpResponseError(response);
    saveTask(task);
    Json::Value details(Json::objectValue);
    details["session"] = session_name;
    details["upload_id"] = (*task)["upload_id"];
    details["stage"] = "upload_init";
    details["error"] = (*task)["last_error"];
    details["is_damaged"] = damaged;
    logEvent(dataset_root, damaged ? "damaged_upload_failed" : "upload_failed", details);
    return false;
  }
  if (validateCompletedResponse(*task, remote)) {
    (*task)["state"] = "remote_completed";
  } else {
    std::set<uint64_t> uploaded;
    for (const auto& item : remote["uploaded_chunks"]) uploaded.insert(item.asUInt64());
    std::vector<uint64_t> missing;
    const uint64_t total_chunks = (*task)["total_chunks"].asUInt64();
    for (uint64_t i = 0; i < total_chunks; ++i) if (!uploaded.count(i)) missing.push_back(i);
    (*task)["state"] = "uploading";
    (*task)["last_error"] = Json::nullValue;
    saveTask(task);
    Json::Value start_details(Json::objectValue);
    start_details["session"] = (*task)["session_name"];
    start_details["upload_id"] = (*task)["upload_id"];
    start_details["missing_chunks"] = Json::UInt64(missing.size());
    start_details["is_damaged"] = damaged;
    logEvent(dataset_root, damaged ? "damaged_upload_started" : "upload_started",
             start_details);

    std::atomic<size_t> cursor{0};
    std::atomic<size_t> completed_chunks{0};
    std::atomic<bool> failed{false};
    std::mutex error_mutex;
    std::string first_error;
    const int worker_count = std::min<int>(upload_parallel_chunks_, std::max<size_t>(1, missing.size()));
    std::vector<std::thread> workers;
    for (int worker = 0; worker < worker_count; ++worker) {
      workers.emplace_back([&, worker] {
        (void)worker;
        NasHttpSession session;
        const int fd = ::open(archive_path.c_str(), O_RDONLY);
        if (fd < 0) {
          failed = true;
          std::lock_guard<std::mutex> lock(error_mutex);
          if (first_error.empty()) first_error = "cannot open ZIP for chunk upload";
          return;
        }
        while (!failed.load()) {
          const size_t pos = cursor.fetch_add(1);
          if (pos >= missing.size()) break;
          const uint64_t index = missing[pos];
          const uint64_t offset = index * chunk_size_;
          const uint64_t wanted = std::min<uint64_t>(chunk_size_, (*task)["archive_size"].asUInt64() - offset);
          std::vector<char> bytes(static_cast<size_t>(wanted));
          size_t got = 0;
          while (got < bytes.size()) {
            const ssize_t n = ::pread(fd, bytes.data() + got, bytes.size() - got,
                                      static_cast<off_t>(offset + got));
            if (n <= 0) break;
            got += static_cast<size_t>(n);
          }
          if (got != bytes.size()) {
            failed = true;
            std::lock_guard<std::mutex> lock(error_mutex);
            if (first_error.empty()) first_error = "ZIP chunk read failed at " + std::to_string(index);
            break;
          }
          bool sent = false;
          HttpResponse chunk_response;
          for (int attempt = 0; attempt <= upload_retry_count_ && !sent; ++attempt) {
            chunk_response = session.postMultipartChunk(
                upload_chunk_url_, (*task)["upload_id"].asString(), index,
                (*task)["upload_id"].asString() + "." + zeroPadded(index, 6) + ".part",
                bytes, connect_timeout_s_, request_timeout_s_);
            sent = chunk_response.ok;
            if (!sent && attempt < upload_retry_count_) {
              Json::Value retry_details(Json::objectValue);
              retry_details["session"] = session_name;
              retry_details["upload_id"] = (*task)["upload_id"];
              retry_details["chunk_index"] = Json::UInt64(index);
              retry_details["attempt"] = attempt + 1;
              retry_details["error"] = httpResponseError(chunk_response);
              retry_details["is_damaged"] = damaged;
              logEvent(dataset_root, "upload_chunk_retry", retry_details);
              if (waitForStop(static_cast<double>(1 << std::min(attempt, 3)))) {
                failed = true;
                std::lock_guard<std::mutex> lock(error_mutex);
                if (first_error.empty()) first_error = "upload stopped during chunk retry";
                break;
              }
            }
          }
          if (!sent) {
            failed = true;
            std::lock_guard<std::mutex> lock(error_mutex);
            if (first_error.empty()) first_error = "chunk " + std::to_string(index) + ": " + httpResponseError(chunk_response);
            break;
          }
          const size_t done = completed_chunks.fetch_add(1) + 1;
          Json::Value progress_details(Json::objectValue);
          progress_details["session"] = session_name;
          progress_details["upload_id"] = (*task)["upload_id"];
          progress_details["chunk_index"] = Json::UInt64(index);
          progress_details["completed_missing_chunks"] = Json::UInt64(done);
          progress_details["missing_chunks"] = Json::UInt64(missing.size());
          progress_details["total_chunks"] = (*task)["total_chunks"];
          progress_details["is_damaged"] = damaged;
          logEvent(dataset_root, "upload_chunk_progress", progress_details);
          ROS_INFO("session ZIP upload progress. upload_id=%s chunks=%zu/%zu",
                   (*task)["upload_id"].asCString(), done, missing.size());
        }
        ::close(fd);
      });
    }
    for (auto& worker : workers) worker.join();
    if (failed) {
      (*task)["state"] = "upload_chunk_failed";
      (*task)["last_error"] = first_error;
      saveTask(task);
      Json::Value details(Json::objectValue);
      details["session"] = (*task)["session_name"];
      details["upload_id"] = (*task)["upload_id"];
      details["error"] = first_error;
      details["is_damaged"] = damaged;
      logEvent(dataset_root, damaged ? "damaged_upload_failed" : "upload_failed", details);
      return false;
    }

    Json::Value complete(Json::objectValue);
    complete["upload_id"] = (*task)["upload_id"];
    response = http_session_->postJson(upload_complete_url_, jsonToString(complete), connect_timeout_s_, request_timeout_s_);
    remote = Json::Value(Json::objectValue);
    bool confirmed = response.ok && parseJson(response.body, &remote) && validateCompletedResponse(*task, remote);
    if (!confirmed && (response.status == 0 || response.status == 502 || response.status == 504)) {
      confirmed = waitForCompletion(*task, &remote);
    }
    if (!confirmed) {
      (*task)["state"] = "upload_complete_unconfirmed";
      (*task)["last_error"] = httpResponseError(response);
      saveTask(task);
      Json::Value details(Json::objectValue);
      details["session"] = session_name;
      details["upload_id"] = (*task)["upload_id"];
      details["stage"] = "upload_complete";
      details["error"] = (*task)["last_error"];
      details["is_damaged"] = damaged;
      logEvent(dataset_root, damaged ? "damaged_upload_failed" : "upload_failed", details);
      return false;
    }
    (*task)["state"] = "remote_completed";
  }

  saveTask(task);
  Json::Value completed_details(Json::objectValue);
  completed_details["session"] = (*task)["session_name"];
  completed_details["upload_id"] = (*task)["upload_id"];
  completed_details["archive"] = (*task)["archive_name"];
  completed_details["size"] = (*task)["archive_size"];
  completed_details["sha256"] = (*task)["archive_sha256"];
  completed_details["is_damaged"] = damaged;
  logEvent(dataset_root, damaged ? "damaged_upload_completed" : "upload_completed", completed_details);
  if (!safeRemoveSession(session_root)) {
    (*task)["state"] = "local_cleanup_failed";
    (*task)["last_error"] = damaged ?
        "refused or failed to remove damaged source session" :
        "refused or failed to remove qualified source session";
    saveTask(task);
    Json::Value cleanup_details = completed_details;
    cleanup_details["error"] = (*task)["last_error"];
    logEvent(dataset_root, "local_cleanup_failed", cleanup_details);
    return false;
  }
  removeFileIfExists(archive_path);
  if (fs::exists(archive_path)) {
    (*task)["state"] = "local_cleanup_failed";
    (*task)["last_error"] = damaged ?
        "failed to remove local damaged ZIP" : "failed to remove local qualified ZIP";
    saveTask(task);
    Json::Value cleanup_details = completed_details;
    cleanup_details["error"] = (*task)["last_error"];
    logEvent(dataset_root, "local_cleanup_failed", cleanup_details);
    return false;
  }
  logEvent(dataset_root, damaged ? "damaged_local_cache_removed" : "local_cache_removed",
           completed_details);
  removeTask(*task);
  cleanupCompletedDataset(dataset_root);
  return true;
}

void SessionZipUploader::discoverDamagedArchives() {
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dataset.second, ec)) {
      const fs::file_status archive_status = entry.symlink_status(ec);
      if (ec) break;
      if (fs::is_symlink(archive_status) || !fs::is_regular_file(archive_status)) continue;
      std::smatch match;
      const std::string archive_name = entry.path().filename().string();
      if (!std::regex_match(archive_name, match, kDamagedArchiveRe)) continue;
      const std::string session_name = match[1].str();
      if (session_name.substr(0, 8) != dataset.first.substr(dataset.first.size() - 8) ||
          !isPathUnder(entry.path(), dataset.second)) continue;

      SessionClaim claim(this, dataset.second / session_name);
      if (!claim) continue;
      const fs::path status_path = status_root_ / (session_name + ".json");
      if (taskPathExists(status_path)) continue;

      std::string digest;
      if (fileSize(entry.path()) == 0 ||
          !runProgram({unzip_binary_, "-tqq", entry.path().string()}) ||
          !sha256File(entry.path(), &digest)) {
        Json::Value details(Json::objectValue);
        details["session"] = session_name;
        details["archive"] = archive_name;
        details["error"] = "historical damaged ZIP failed CRC or SHA-256 validation";
        logEvent(dataset.second, "damaged_discovery_failed", details);
        continue;
      }

      Json::Value task(Json::objectValue);
      task["archive_kind"] = "session_zip";
      task["dataset_name"] = dataset.first;
      task["dataset_root_abs"] = dataset.second.string();
      task["session_name"] = session_name;
      task["session_root_abs"] = (dataset.second / session_name).string();
      task["archive_path"] = entry.path().string();
      task["archive_name"] = archive_name;
      task["archive_size"] = Json::UInt64(fileSize(entry.path()));
      task["archive_sha256"] = digest;
      // Old releases removed the state file after creating a damaged ZIP, so
      // the original entry count is unknown. NAS accepts zero only for damaged
      // archives and reports the actual verified count at completion.
      task["archive_entries"] = Json::UInt64(0);
      task["total_chunks"] = Json::UInt64(std::max<uint64_t>(
          1, (fileSize(entry.path()) + chunk_size_ - 1) / chunk_size_));
      task["upload_id"] = "session_zip_" + dataset.first + "_" + session_name + "_" +
          digest.substr(0, 24);
      task["status_path"] = status_path.string();
      task["state"] = "damaged_waiting_upload";
      task["created_at"] = static_cast<double>(unixMs()) / 1000.0;
      Json::Value quality(Json::objectValue);
      quality["qualified"] = false;
      quality["recovered_historical_archive"] = true;
      task["quality"] = quality;
      if (!saveTask(&task)) {
        ROS_ERROR("failed to create upload state for historical damaged ZIP: %s",
                  entry.path().c_str());
        continue;
      }
      Json::Value details(Json::objectValue);
      details["session"] = session_name;
      details["archive"] = archive_name;
      details["size"] = task["archive_size"];
      details["sha256"] = digest;
      logEvent(dataset.second, "damaged_upload_task_recovered", details);
    }
    if (ec) ROS_WARN("failed to discover damaged ZIPs under %s: %s",
                     dataset.second.c_str(), ec.message().c_str());
  }
}

bool SessionZipUploader::reclaimDamagedArchivesForCapacity() {
  bool size_ok = false;
  uint64_t used_bytes = directoryLogicalSize(scan_root_, &size_ok);
  const uint64_t limit_bytes = node_->maxScanRootBytes();
  if (!size_ok) {
    ROS_ERROR("cannot calculate scan_root size; damaged ZIP capacity cleanup skipped: %s",
              scan_root_.c_str());
    return false;
  }
  if (used_bytes < limit_bytes) return true;

  struct Candidate {
    std::string session_name;
    fs::path dataset_root;
    fs::path archive_path;
  };
  std::vector<Candidate> candidates;
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    std::error_code iter_ec;
    fs::directory_iterator it(dataset.second, fs::directory_options::none, iter_ec);
    for (; !iter_ec && it != fs::directory_iterator(); it.increment(iter_ec)) {
      std::error_code status_ec;
      const fs::file_status status = it->symlink_status(status_ec);
      if (status_ec || fs::is_symlink(status) || !fs::is_regular_file(status)) continue;
      std::smatch match;
      const std::string archive_name = it->path().filename().string();
      if (!std::regex_match(archive_name, match, kDamagedArchiveRe)) continue;
      const std::string session_name = match[1].str();
      const fs::path expected = dataset.second / (session_name + "_Damaged.zip");
      if (session_name.substr(0, 8) != dataset.first.substr(dataset.first.size() - 8) ||
          fs::absolute(it->path()).lexically_normal() != fs::absolute(expected).lexically_normal() ||
          !isPathUnder(it->path(), dataset.second)) continue;
      candidates.push_back({session_name, dataset.second, it->path()});
    }
    if (iter_ec) {
      ROS_WARN("failed to scan damaged ZIP candidates under %s: %s",
               dataset.second.c_str(), iter_ec.message().c_str());
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    if (a.session_name != b.session_name) return a.session_name < b.session_name;
    return a.archive_path.string() < b.archive_path.string();
  });

  size_t removed_count = 0;
  for (const auto& candidate : candidates) {
    if (used_bytes < limit_bytes) break;
    SessionClaim claim(this, candidate.dataset_root / candidate.session_name);
    // In particular, an upload owns this claim from its first archive check
    // through NAS confirmation and local cleanup.  Capacity pressure must
    // never remove that worker's open _Damaged.zip.
    if (!claim) continue;
    {
      std::lock_guard<std::mutex> lock(node_->session_mutex_);
      if (node_->recording_ &&
          fs::absolute(node_->session_root_).lexically_normal() ==
              fs::absolute(candidate.dataset_root / candidate.session_name).lexically_normal()) {
        continue;
      }
    }

    std::error_code status_ec;
    const fs::file_status status = fs::symlink_status(candidate.archive_path, status_ec);
    if (status_ec || fs::is_symlink(status) || !fs::is_regular_file(status) ||
        !isPathUnder(candidate.archive_path, candidate.dataset_root)) {
      continue;
    }
    const uint64_t archive_size = fileSize(candidate.archive_path);
    if (archive_size == 0 ||
        !runProgram({unzip_binary_, "-tqq", candidate.archive_path.string()})) {
      Json::Value details(Json::objectValue);
      details["session"] = candidate.session_name;
      details["archive"] = candidate.archive_path.filename().string();
      details["error"] = "capacity cleanup retained ZIP because CRC validation failed";
      logEvent(candidate.dataset_root, "damaged_capacity_cleanup_skipped", details);
      continue;
    }

    const uint64_t used_before = used_bytes;
    std::error_code remove_ec;
    const bool archive_removed = fs::remove(candidate.archive_path, remove_ec);
    if (remove_ec || !archive_removed) {
      ROS_WARN("failed to remove damaged ZIP during capacity cleanup: %s error=%s",
               candidate.archive_path.c_str(), remove_ec.message().c_str());
      continue;
    }

    bool status_removed = false;
    const fs::path task_path = status_root_ / (candidate.session_name + ".json");
    Json::Value task;
    fs::path managed_dataset, managed_archive;
    if (readTask(task_path, &task)) {
      task["status_path"] = task_path.string();
      if (taskIsDamaged(task) &&
          validTaskPaths(task, &managed_dataset, nullptr, &managed_archive) &&
          fs::absolute(managed_dataset).lexically_normal() ==
              fs::absolute(candidate.dataset_root).lexically_normal() &&
          fs::absolute(managed_archive).lexically_normal() ==
              fs::absolute(candidate.archive_path).lexically_normal()) {
        removeTask(task);
        status_removed = !taskPathExists(task_path);
        if (!status_removed) {
          ROS_WARN("damaged ZIP was evicted but its matching task state was retained: %s",
                   task_path.c_str());
        }
      }
    }

    bool after_ok = false;
    const uint64_t used_after = directoryLogicalSize(scan_root_, &after_ok);
    Json::Value details(Json::objectValue);
    details["session"] = candidate.session_name;
    details["archive"] = candidate.archive_path.filename().string();
    details["archive_size"] = Json::UInt64(archive_size);
    details["used_before_bytes"] = Json::UInt64(used_before);
    details["used_after_bytes"] = Json::UInt64(used_after);
    details["limit_bytes"] = Json::UInt64(limit_bytes);
    details["matching_status_removed"] = status_removed;
    logEvent(candidate.dataset_root, "damaged_evicted_for_capacity", details);
    ++removed_count;
    if (!after_ok) {
      ROS_ERROR("damaged ZIP was evicted but scan_root size recalculation failed: %s",
                scan_root_.c_str());
      return false;
    }
    // logEvent itself appends a small file, so include that write in the next
    // admission decision instead of relying on the pre-log measurement.
    used_bytes = directoryLogicalSize(scan_root_, &size_ok);
    if (!size_ok) return false;
  }

  if (used_bytes >= limit_bytes) {
    ROS_WARN("scan_root remains over capacity after damaged ZIP cleanup. used=%llu limit=%llu "
             "removed=%zu; new capture remains blocked",
             static_cast<unsigned long long>(used_bytes),
             static_cast<unsigned long long>(limit_bytes), removed_count);
    return false;
  }
  ROS_INFO("damaged ZIP capacity cleanup completed. used=%llu limit=%llu removed=%zu",
           static_cast<unsigned long long>(used_bytes),
           static_cast<unsigned long long>(limit_bytes), removed_count);
  return true;
}

bool SessionZipUploader::resumePendingTasks(bool damaged_tasks, bool* uploaded_task) {
  if (uploaded_task) *uploaded_task = false;
  const std::vector<fs::path> files = listTaskFiles();
  std::error_code ec;
  for (const auto& path : files) {
    const std::string claimed_session_name = path.stem().string();
    if (!std::regex_match(claimed_session_name, kSessionDirRe)) {
      ROS_ERROR("invalid session upload task filename retained: %s", path.c_str());
      continue;
    }
    Json::Value task_snapshot;
    if (!readTask(path, &task_snapshot) ||
        task_snapshot.get("archive_kind", "").asString() != "session_zip") {
      ROS_ERROR("invalid session upload task retained: %s", path.c_str());
      continue;
    }
    if (task_snapshot.get("session_name", "").asString() != claimed_session_name) {
      ROS_ERROR("session upload task filename/payload mismatch retained: %s", path.c_str());
      continue;
    }
    if (taskIsDamaged(task_snapshot) != damaged_tasks) continue;

    SessionClaim claim(this, scan_root_ /
        ("excavator_data_" + claimed_session_name.substr(0, 8)) / claimed_session_name);
    if (!claim) {
      // This is the oldest task of the requested class in the ordered state
      // snapshot.  Waiting for its quality/cleanup owner preserves strict FIFO
      // instead of allowing a newer normal archive to overtake it.
      return true;
    }
    Json::Value task;
    if (!readTask(path, &task) || task.get("archive_kind", "").asString() != "session_zip") {
      // It may have completed and disappeared between the unlocked snapshot
      // read and claim acquisition.  Continue with the next durable task.
      continue;
    }
    task["status_path"] = path.string();
    if (task.get("session_name", "").asString() != claimed_session_name) {
      ROS_ERROR("session upload task filename/payload mismatch retained: %s", path.c_str());
      continue;
    }
    const bool damaged = taskIsDamaged(task);
    if (damaged != damaged_tasks) continue;
    const fs::path archive(task.get("archive_path", "").asString());
    if (!fs::is_regular_file(archive)) {
      fs::path managed_session;
      if (validTaskPaths(task, nullptr, &managed_session, nullptr) &&
          fs::is_directory(managed_session)) {
        removeTask(task);
        ROS_WARN("session archive missing; status removed so source can be rebuilt: %s", path.c_str());
      } else {
        ROS_WARN("orphan session status retained until TTL cleanup: %s", path.c_str());
      }
      continue;
    }
    const std::string session_name = task.get("session_name", "").asString();
    const fs::path dataset_root(task.get("dataset_root_abs", "").asString());
    const fs::path session_root(task.get("session_root_abs", "").asString());
    if (damaged) {
      std::string digest;
      fs::path managed_dataset, managed_session, managed_archive;
      const bool verified = validTaskPaths(
              task, &managed_dataset, &managed_session, &managed_archive) &&
          archive == managed_archive && isPathUnder(archive, scan_root_) &&
          runProgram({unzip_binary_, "-tqq", archive.string()}) &&
          sha256File(archive, &digest) && digest == task.get("archive_sha256", "").asString();
      if (!verified) {
        task["state"] = "damaged_verification_failed";
        task["last_error"] = "damaged ZIP verification failed";
        saveTask(&task);
        Json::Value details(Json::objectValue);
        details["session"] = session_name;
        details["archive"] = archive.filename().string();
        details["error"] = task["last_error"];
        logEvent(dataset_root, "local_cleanup_failed", details);
        continue;
      }
      const bool source_existed = fs::is_directory(session_root, ec);
      ec.clear();
      if (!safeRemoveSession(session_root)) {
        task["state"] = "damaged_cleanup_failed";
        task["last_error"] = "damaged source cleanup failed";
        saveTask(&task);
        Json::Value details(Json::objectValue);
        details["session"] = session_name;
        details["archive"] = archive.filename().string();
        details["error"] = task["last_error"];
        logEvent(dataset_root, "local_cleanup_failed", details);
        continue;
      }
      task["state"] = "damaged_waiting_upload";
      task["last_error"] = Json::nullValue;
      saveTask(&task);
      if (source_existed) {
        Json::Value details(Json::objectValue);
        details["session"] = session_name;
        details["archive"] = archive.filename().string();
        details["sha256"] = digest;
        logEvent(dataset_root, "damaged_source_removed_after_restart", details);
      }
    }
    if (!uploadTask(&task)) return true;
    if (uploaded_task) *uploaded_task = true;
    // A newly closed qualified session must get a chance before the next
    // damaged archive. The caller re-runs the normal/source phases between
    // damaged uploads; the currently active damaged upload is never preempted.
    if (damaged_tasks) return false;
  }
  return false;
}

bool SessionZipUploader::scanAndProcessSessions() {
  bool had_failure = false;
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    for (const auto& session : listSessionDirs(dataset.second)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_) return had_failure;
      }
      {
        std::lock_guard<std::mutex> lock(node_->session_mutex_);
        if (node_->recording_ && fs::absolute(node_->session_root_).lexically_normal() ==
                                     fs::absolute(session.second).lexically_normal()) continue;
      }
      // A qualified source directory intentionally remains on disk until NAS
      // confirms its ZIP.  Its durable task is the ownership marker that keeps
      // later scans from compressing or replacing the same archive again.
      if (sessionHasQueuedArchive(session.second)) continue;
      if (!sessionIsStable(session.second)) {
        // Multiple close events can share the same earliest forced wake-up.
        // If a later session has not completed its quiet period at that scan,
        // retain a near-term wake-up instead of falling back to the normal
        // upload_interval_s_ cycle (90 seconds by default).
        scheduleQualityScanAfter(std::max(0.5, stop_flush_delay_s_));
        continue;
      }
      try {
        if (!processSession(session.second)) {
          had_failure = true;
          ROS_ERROR("session quality/ZIP processing failed; continuing with later sessions: %s",
                    session.second.c_str());
        }
      } catch (const std::exception& e) {
        had_failure = true;
        ROS_ERROR("session quality/ZIP processing threw; continuing. session=%s error=%s",
                  session.second.c_str(), e.what());
      }
    }
  }
  return had_failure;
}

void SessionZipUploader::cleanupLegacyArtifacts() {
  const auto cleanup = [&](const fs::path& path, const std::set<std::string>& allowed_extensions) {
    std::error_code ec;
    if (!isPathUnder(path, scan_root_) || !fs::is_directory(path, ec) ||
        fs::is_symlink(fs::symlink_status(path, ec))) return;
    std::vector<fs::path> directories;
    size_t removed = 0;
    for (auto it = fs::recursive_directory_iterator(path, ec);
         !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
      const fs::file_status status = it->symlink_status(ec);
      if (ec) break;
      if (fs::is_symlink(status)) {
        it.disable_recursion_pending();
        ROS_WARN("legacy cleanup retained symbolic link: %s", it->path().c_str());
        continue;
      }
      if (fs::is_directory(status)) {
        directories.push_back(it->path());
        continue;
      }
      if (!fs::is_regular_file(status) || !allowed_extensions.count(it->path().extension().string())) {
        ROS_WARN("legacy cleanup retained unexpected file: %s", it->path().c_str());
        continue;
      }
      fs::remove(it->path(), ec);
      if (ec) break;
      ++removed;
    }
    if (ec) {
      ROS_WARN("legacy upload cache cleanup failed: %s error=%s", path.c_str(), ec.message().c_str());
      return;
    }
    std::sort(directories.rbegin(), directories.rend());
    directories.push_back(path);
    for (const auto& directory : directories) {
      ec.clear();
      if (fs::is_directory(directory, ec) && !fs::is_symlink(fs::symlink_status(directory, ec)) &&
          fs::is_empty(directory, ec)) fs::remove(directory, ec);
    }
    if (removed) ROS_WARN("legacy upload artifacts removed: %zu under %s; source sessions retained",
                          removed, path.c_str());
  };
  cleanup(scan_root_ / kTmpRootDirname, {".tar", ".tmp"});
  cleanup(scan_root_ / kStatusRootDirname, {".json", ".tmp", ".invalid"});
}

void SessionZipUploader::qualityLoop() {
  // Recovery is completed before either worker mutates persisted upload
  // state.  As soon as the barrier opens the upload worker may resume normal
  // archives; damaged archives remain gated by the first full quality scan.
  try {
    cleanupLegacyArtifacts();
    cleanupSessionCacheArtifacts();
    reclaimDamagedArchivesForCapacity();
    discoverDamagedArchives();
  } catch (const std::exception& e) {
    // Do not leave both workers permanently parked if best-effort cache
    // recovery encounters one bad historical artifact.
    ROS_ERROR("session ZIP startup recovery failed: %s", e.what());
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    startup_recovery_complete_ = true;
    upload_work_pending_ = true;
    upload_cv_.notify_one();
  }

  double next_cycle_at = monotonicSeconds();
  double next_cache_cleanup_at = monotonicSeconds() + cache_cleanup_interval_s_;
  while (ros::ok()) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stop_) break;
      const double now = monotonicSeconds();
      const bool forced_ready = next_forced_quality_scan_at_ &&
          now >= *next_forced_quality_scan_at_;
      if (now < next_cycle_at && !forced_ready) {
        double wake_at = next_cycle_at;
        if (next_forced_quality_scan_at_) {
          wake_at = std::min(wake_at, *next_forced_quality_scan_at_);
        }
        quality_cv_.wait_for(
            lock, std::chrono::duration<double>(std::max(0.1, wake_at - now)));
        continue;
      }
      if (forced_ready) next_forced_quality_scan_at_.reset();
      quality_scan_active_ = true;
    }

    try {
      if (monotonicSeconds() >= next_cache_cleanup_at) {
        cleanupSessionCacheArtifacts();
        next_cache_cleanup_at = monotonicSeconds() + cache_cleanup_interval_s_;
      }
      // These local maintenance operations never perform NAS I/O.  Capacity
      // reclamation therefore continues even when the upload worker is stuck
      // retrying an unavailable server.
      reclaimDamagedArchivesForCapacity();
      discoverDamagedArchives();
      const bool had_failure = scanAndProcessSessions();
      if (had_failure) {
        ROS_WARN("one or more session quality/ZIP jobs failed; they will be retried next cycle");
      }
    } catch (const std::exception& e) {
      ROS_ERROR("session quality/ZIP cycle failed: %s", e.what());
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      quality_scan_active_ = false;
      initial_quality_scan_complete_ = true;
      upload_work_pending_ = true;
      upload_cv_.notify_one();
    }
    next_cycle_at = monotonicSeconds() + upload_interval_s_;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    quality_scan_active_ = false;
    upload_cv_.notify_all();
  }
}

void SessionZipUploader::uploadLoop() {
  double next_cycle_at = monotonicSeconds();
  while (ros::ok()) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!stop_ && ros::ok() && !startup_recovery_complete_) {
        upload_cv_.wait(lock);
      }
      if (stop_ || !ros::ok()) break;
      const double now = monotonicSeconds();
      if (!upload_work_pending_ && now < next_cycle_at) {
        upload_cv_.wait_for(
            lock, std::chrono::duration<double>(std::max(0.1, next_cycle_at - now)));
        continue;
      }
      upload_work_pending_ = false;
    }

    bool blocked = false;
    bool normal_uploaded = false;
    bool damaged_uploaded = false;
    try {
      // The oldest normal task is strict priority.  A failed normal upload
      // delays later upload attempts, but it has no relationship with the
      // quality worker and cannot prevent new ZIPs from being created.
      blocked = resumePendingTasks(false, &normal_uploaded);
      if (!blocked && damagedUploadsMayStart()) {
        // resumePendingTasks intentionally handles only one damaged archive.
        // The next iteration checks normal priority again before continuing.
        blocked = resumePendingTasks(true, &damaged_uploaded);
      }
    } catch (const std::exception& e) {
      blocked = true;
      ROS_ERROR("session ZIP upload cycle failed: %s", e.what());
    }

    if (!blocked && damaged_uploaded) {
      next_cycle_at = monotonicSeconds();
    } else {
      next_cycle_at = monotonicSeconds() + upload_interval_s_;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_to_NAS_and_ubuntu2204");
  try {
    UnifiedDataNode node;
    ros::AsyncSpinner spinner(std::max(2, node.getParam<int>("ros_spinner_threads", 4)));
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    node.shutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("data_to_NAS_and_ubuntu2204 C++ fatal error: %s", e.what());
    return 1;
  }
  return 0;
}
