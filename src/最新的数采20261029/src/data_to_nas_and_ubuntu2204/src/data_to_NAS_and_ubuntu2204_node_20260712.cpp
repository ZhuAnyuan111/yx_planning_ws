/*
项目                           修改人     修改日期       改动
无人挖机——NAS数采+大脑通信      wuql17     2026/07/03    基于20260629的python程序改C++（新增程序启动代码版本输入;新增控制、轨迹融合、vla轨迹订阅；修改数采文件夹保存逻辑；增加大脑手柄信号保存逻辑）
无人挖机——NAS数采+大脑通信      wuql17     2026/07/06    移除opencv,换gstreamer拉流，libpng处理图片
无人挖机——NAS数采+大脑通信      wuql17     2026/07/07    改代码逻辑，修复摄像头数据保存频率低的问题
无人挖机——NAS数采+大脑通信      wuql17     2026/07/09    修复摄像头功能安全误报问题
无人挖机——NAS数采+大脑通信      wuql17     2026/07/12    解决NAS数据上传问题




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
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace {

const std::regex kDatasetRe("^excavator_data_[0-9]{8}$");
const std::regex kSessionDirRe("^[0-9]{14}$");
const std::vector<std::string> kDataSubdirNames = {"bag", "grayPicture", "rangePicture", "camera_data"};

const char* kDefaultScanRoot = "/media/nvidia/G500";
const char* kManagedMountRoot = "/media/nvidia/G500";
const char* kDefaultBrainBaseUrl = "http://192.168.1.140:8082";
const char* kDefaultNasBaseUrl = "http://192.168.1.243:8082";
const char* kTmpRootDirname = ".sy200_upload_tmp";
const char* kStatusRootDirname = ".sy200_upload_status";
const char* kDefaultTarBinary = "/usr/bin/tar";
const char* kDefaultRawVideoParseFormat = "4";
constexpr int kDefaultCameraInputWidth = 1920;
constexpr int kDefaultCameraInputHeight = 1080;

constexpr uint64_t kDefaultTarBatchMaxBytes = 256ULL * 1024ULL * 1024ULL;
constexpr uint64_t kDefaultChunkSizeBytes = 8ULL * 1024ULL * 1024ULL;
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

fs::path requireManagedPath(const fs::path& path) {
  std::error_code ec;
  fs::path normalized = fs::weakly_canonical(fs::absolute(path), ec);
  if (ec) normalized = fs::absolute(path).lexically_normal();
  if (!isPathUnder(normalized, fs::path(kManagedMountRoot))) {
    throw std::runtime_error("managed path must stay under " + std::string(kManagedMountRoot) + ": " + normalized.string());
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

void removeFileIfExists(const fs::path& path) {
  std::error_code ec;
  if (!path.empty()) {
    fs::remove(path, ec);
  }
}

bool isStableFile(const fs::path& path, double stable_age_s) {
  std::error_code ec;
  auto ftime = fs::last_write_time(path, ec);
  if (ec) {
    return false;
  }
  auto now = fs::file_time_type::clock::now();
  return std::chrono::duration<double>(now - ftime).count() >= stable_age_s;
}

uint64_t fileSize(const fs::path& path) {
  std::error_code ec;
  auto size = fs::file_size(path, ec);
  return ec ? 0 : static_cast<uint64_t>(size);
}

int64_t fileMtimeMs(const fs::path& path) {
  struct stat info {};
  if (::stat(path.c_str(), &info) != 0) return 0;
  return static_cast<int64_t>(info.st_mtim.tv_sec) * 1000LL +
      static_cast<int64_t>(info.st_mtim.tv_nsec / 1000000L);
}

std::string relativePosix(const fs::path& path, const fs::path& start) {
  std::error_code ec;
  fs::path rel = fs::relative(path, start, ec);
  if (ec) {
    rel = path.filename();
  }
  return rel.generic_string();
}

bool normalizeUploadRelativePath(const std::string& input,
                                 const std::string& dataset_name,
                                 std::string* normalized,
                                 std::string* error) {
  std::string path = input;
  std::replace(path.begin(), path.end(), '\\', '/');
  if (path.empty() || path.front() == '/' || (path.size() >= 2 && path[1] == ':')) {
    if (error) *error = "upload relative path must not be empty or absolute: " + input;
    return false;
  }

  std::vector<std::string> parts;
  std::istringstream stream(path);
  std::string part;
  while (std::getline(stream, part, '/')) {
    if (part.empty() || part == ".") continue;
    if (part == "..") {
      if (error) *error = "upload relative path must not contain '..': " + input;
      return false;
    }
    parts.push_back(part);
  }
  if (parts.empty()) {
    if (error) *error = "upload relative path is required";
    return false;
  }

  const auto is_data_dir = [](const std::string& value) {
    return std::find(kDataSubdirNames.begin(), kDataSubdirNames.end(), value) !=
        kDataSubdirNames.end();
  };
  if (is_data_dir(parts[0])) {
    // Legacy flat layout.
  } else if (std::regex_match(parts[0], kSessionDirRe)) {
    if (parts.size() < 2 || !is_data_dir(parts[1])) {
      if (error) *error = "session upload path must continue with a data directory: " + input;
      return false;
    }
    if (!std::regex_match(dataset_name, kDatasetRe) ||
        parts[0].substr(0, 8) != dataset_name.substr(dataset_name.size() - 8)) {
      if (error) *error = "session directory date must match dataset_name: " + input;
      return false;
    }
  } else {
    if (error) *error = "upload path must start with a data directory or 14-digit session directory: " + input;
    return false;
  }

  std::ostringstream out;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i) out << '/';
    out << parts[i];
  }
  if (normalized) *normalized = out.str();
  return true;
}

void pruneEmptyTree(const fs::path& root) {
  std::error_code ec;
  if (!fs::is_directory(root, ec)) {
    return;
  }
  std::vector<fs::path> dirs;
  for (auto it = fs::recursive_directory_iterator(root, ec); !ec && it != fs::recursive_directory_iterator(); ++it) {
    if (it->is_directory(ec)) {
      dirs.push_back(it->path());
    }
  }
  std::sort(dirs.rbegin(), dirs.rend());
  for (const auto& dir : dirs) {
    if (fs::is_empty(dir, ec)) {
      fs::remove(dir, ec);
    }
  }
  if (fs::is_directory(root, ec) && fs::is_empty(root, ec)) {
    fs::remove(root, ec);
  }
}

std::vector<std::pair<std::string, fs::path>> listDatasetDirs(const fs::path& scan_root) {
  std::vector<std::pair<std::string, fs::path>> out;
  std::error_code ec;
  if (!fs::is_directory(scan_root, ec)) {
    return out;
  }
  for (const auto& entry : fs::directory_iterator(scan_root, ec)) {
    if (!entry.is_directory(ec)) {
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
    if (!entry.is_directory(ec)) {
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

void removeDatasetRootIfEmpty(const fs::path& dataset_root) {
  std::error_code ec;
  if (!fs::is_directory(dataset_root, ec)) {
    return;
  }
  for (const auto& item : listSessionDirs(dataset_root)) {
    const auto& session_root = item.second;
    for (const auto& name : kDataSubdirNames) {
      pruneEmptyTree(session_root / name);
    }
    if (fs::is_directory(session_root, ec) && fs::is_empty(session_root, ec)) {
      fs::remove(session_root, ec);
    }
  }
  for (const auto& name : kDataSubdirNames) {
    pruneEmptyTree(dataset_root / name);
  }
  if (fs::is_directory(dataset_root, ec) && fs::is_empty(dataset_root, ec)) {
    fs::remove(dataset_root, ec);
  }
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
  {
    std::ofstream out(tmp, std::ios::binary);
    if (!out) {
      return false;
    }
    out << data;
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

struct CandidateFile {
  fs::path abs_path;
  std::string rel_path;
  uint64_t size_bytes = 0;
  int64_t mtime_ms = 0;
  std::string cleanup_mode = "regular";
};

class UnifiedDataNode;

enum class ArchiveCheckResult { kReady, kDiscarded, kFailed };

class ChunkedArchiveUploader {
 public:
  ChunkedArchiveUploader(UnifiedDataNode* node, std::string archive_kind);
  void start();
  void stop();
  void handleSaveSignalTransition(int previous, int current);

 private:
  std::vector<CandidateFile> collectCandidates(const fs::path& dataset_root, bool force);
  std::vector<CandidateFile> collectCameraCandidates(const fs::path& dataset_root, bool force);
  std::vector<CandidateFile> collectStateCandidates(const fs::path& dataset_root, bool force);
  void appendStateCandidates(const fs::path& dataset_root, const fs::path& root, bool force, const std::string& kind, std::vector<CandidateFile>* out);
  std::vector<std::vector<CandidateFile>> splitBatches(const std::vector<CandidateFile>& candidates) const;
  Json::Value buildTask(const std::string& dataset_name, const fs::path& dataset_root,
                        const std::vector<CandidateFile>& batch, int64_t cycle_ts_ms, int seq);
  bool saveTask(Json::Value* task);
  void removeTask(const Json::Value& task);
  void cleanupArtifacts(const Json::Value& task);
  bool createArchive(Json::Value* task);
  ArchiveCheckResult ensureTaskArchive(Json::Value* task);
  bool executeTask(Json::Value* task);
  bool resumePendingTasks();
  void recordRemoteFailure(Json::Value* task, const std::string& state,
                           const std::string& stage, const HttpResponse& response,
                           std::optional<uint64_t> chunk_index = std::nullopt);
  void markRemoteSuccess();
  void updateLocalStatus(Json::Value* task, const Json::Value& status);
  void progressLog(const Json::Value& task, const Json::Value& status) const;
  void cleanupUploadedTask(const Json::Value& task);
  bool runTar(const fs::path& archive_path, const fs::path& dataset_root,
              const fs::path& manifest_path) const;
  bool isLegacyTask(const Json::Value& task) const;
  void cleanupLegacyTask(const fs::path& status_path, const Json::Value& task);
  void runLoop();
  void runUploadCycle(bool force);

  UnifiedDataNode* node_;
  std::string archive_kind_;
  fs::path scan_root_;
  std::string upload_init_url_;
  std::string upload_chunk_url_;
  std::string upload_complete_url_;
  double upload_interval_s_ = 90.0;
  double file_stable_age_s_ = 10.0;
  double stop_flush_delay_s_ = 10.0;
  uint64_t tar_batch_max_bytes_ = kDefaultTarBatchMaxBytes;
  uint64_t chunk_size_ = kDefaultChunkSizeBytes;
  std::string tar_binary_ = kDefaultTarBinary;
  fs::path tar_tmp_root_;
  fs::path status_root_;
  double connect_timeout_s_ = kDefaultNasConnectTimeoutS;
  double request_timeout_s_ = kDefaultNasRequestTimeoutS;
  int session_recreate_failures_ = kDefaultSessionRecreateFailures;
  int consecutive_remote_failures_ = 0;
  std::unique_ptr<NasHttpSession> http_session_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::optional<double> force_scan_at_;
  std::thread thread_;
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
    scan_root_ = requireManagedPath(resolveScanRoot());
    min_free_bytes_ = static_cast<uint64_t>(getParam<double>("min_free_bytes", static_cast<double>(kMinFreeBytes)));
    local_image_queue_max_ = getParam<int>("local_image_queue_max", 1000);
    ensureDir(scan_root_);

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
    state_uploader_ = std::make_unique<ChunkedArchiveUploader>(this, "state");
    camera_uploader_ = std::make_unique<ChunkedArchiveUploader>(this, "camera");

    recoverOrphanStateJsonTmpFiles();
    brain_post_worker_.start();
    state_uploader_->start();
    camera_uploader_->start();
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
    if (state_uploader_) state_uploader_->stop();
    if (camera_uploader_) camera_uploader_->stop();
    for (auto& worker : camera_workers_) {
      if (worker) worker->stop();
    }
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      recording_ = false;
      finalizeSessionLocked();
    }
    brain_post_worker_.stop();
    ROS_INFO("data_to_NAS_and_ubuntu2204 C++ shutdown complete.");
  }

 private:
  fs::path resolveScanRoot() {
    std::string param_root;
    if (pnh_.getParam("scan_root", param_root) && !param_root.empty()) {
      return fs::absolute(param_root);
    }
    std::string env = getenvString("SY200_SCAN_ROOT");
    if (!env.empty()) {
      return fs::absolute(env);
    }
    std::string excavator_data_path;
    if (nh_.getParam("/excavator_data_path", excavator_data_path) && !excavator_data_path.empty()) {
      fs::path p = fs::absolute(excavator_data_path);
      if (std::regex_match(p.filename().string(), kDatasetRe)) {
        return p.parent_path();
      }
      return p;
    }
    return fs::absolute(kDefaultScanRoot);
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

  void logLowSpace(const std::string& source) {
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
      std::time_t host_ts_sec = std::time(nullptr);
      std::string dataset_name;
      fs::path json_tmp_path;
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (last_save_signal_ != previous || raw_save_signal_ != signal_value) {
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
        last_save_signal_ = signal_value;
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
      {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (last_save_signal_ != previous || raw_save_signal_ != signal_value) {
          return;
        }
        session_id = state_session_id_;
        dataset_name = dataset_name_;
        tmp_path = json_tmp_path_;
        final_path = json_final_path_;
        bag_dir = bag_dir_;
        recording_ = false;
        {
          std::lock_guard<std::mutex> shared_lock(session_shared_->mutex);
          session_shared_->recording = false;
          session_shared_->camera_0_dir.clear();
          session_shared_->camera_1_dir.clear();
        }
        last_save_signal_ = signal_value;
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
    }
    if (applied) {
      state_uploader_->handleSaveSignalTransition(previous, signal_value);
      camera_uploader_->handleSaveSignalTransition(previous, signal_value);
    }
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
    std_msgs::Int8 msg;
    msg.data = (state_ok && grid_ok && range_ok && cam0_http_status_.load() && cam1_http_status_.load()) ? 1 : 0;
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
  std::unique_ptr<ChunkedArchiveUploader> state_uploader_;
  std::unique_ptr<ChunkedArchiveUploader> camera_uploader_;
  std::shared_ptr<SessionShared> session_shared_;
  std::shared_ptr<std::atomic<bool>> shutdown_flag_;
  std::vector<std::unique_ptr<CameraWorker>> camera_workers_;
  std::atomic<bool> shutdown_done_{false};

  friend class ChunkedArchiveUploader;
};

ChunkedArchiveUploader::ChunkedArchiveUploader(UnifiedDataNode* node, std::string archive_kind)
    : node_(node), archive_kind_(std::move(archive_kind)), scan_root_(node->scanRoot()) {
  std::string nas_base = node_->getParam<std::string>("nas_webhook_url", getenvString("SY200_NAS_BASE_URL").empty() ? kDefaultNasBaseUrl : getenvString("SY200_NAS_BASE_URL"));
  upload_init_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_init_path", "/upload_init"));
  upload_chunk_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_chunk_path", "/upload_chunk"));
  upload_complete_url_ = joinUrl(nas_base, node_->getParam<std::string>("nas_upload_complete_path", "/upload_complete"));
  upload_interval_s_ = std::max(1.0, node_->getParam<double>("upload_interval_s", 90.0));
  file_stable_age_s_ = std::max(0.0, node_->getParam<double>("file_stable_age_s", 10.0));
  stop_flush_delay_s_ = std::max(0.0, node_->getParam<double>("stop_flush_delay_s", 10.0));
  tar_batch_max_bytes_ = static_cast<uint64_t>(std::max<double>(1.0, node_->getParam<double>("tar_batch_max_bytes", static_cast<double>(kDefaultTarBatchMaxBytes))));
  chunk_size_ = static_cast<uint64_t>(std::max<double>(1.0, node_->getParam<double>("chunk_size_bytes", static_cast<double>(kDefaultChunkSizeBytes))));
  tar_binary_ = node_->getParam<std::string>("tar_binary", kDefaultTarBinary);
  const fs::path default_tmp_root = scan_root_ / kTmpRootDirname / archive_kind_;
  const fs::path default_status_root = scan_root_ / kStatusRootDirname / archive_kind_;
  tar_tmp_root_ = requireManagedPath(node_->getParam<std::string>(
      "tar_tmp_root_" + archive_kind_, default_tmp_root.string()));
  status_root_ = requireManagedPath(node_->getParam<std::string>(
      "upload_status_root_" + archive_kind_, default_status_root.string()));
  connect_timeout_s_ = node_->getParam<double>("connect_timeout_s", kDefaultNasConnectTimeoutS);
  request_timeout_s_ = node_->getParam<double>("request_timeout_s", kDefaultNasRequestTimeoutS);
  session_recreate_failures_ = std::max(1, node_->getParam<int>(
      "nas_session_recreate_failures",
      node_->getParam<int>("http_session_recreate_failures", kDefaultSessionRecreateFailures)));
  http_session_ = std::make_unique<NasHttpSession>();
}

void ChunkedArchiveUploader::start() {
  ensureDir(tar_tmp_root_);
  ensureDir(status_root_);
  stop_ = false;
  thread_ = std::thread([this] { runLoop(); });
  ROS_INFO("%s uploader started. nas_protocol=python-compatible-v2 scan_root=%s "
           "interval=%.1fs batch_max=%llu chunk_size=%llu init_url=%s chunk_url=%s complete_url=%s",
           archive_kind_.c_str(), scan_root_.c_str(), upload_interval_s_,
           static_cast<unsigned long long>(tar_batch_max_bytes_),
           static_cast<unsigned long long>(chunk_size_), upload_init_url_.c_str(),
           upload_chunk_url_.c_str(), upload_complete_url_.c_str());
}

void ChunkedArchiveUploader::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    cv_.notify_all();
  }
  if (thread_.joinable()) thread_.join();
}

void ChunkedArchiveUploader::handleSaveSignalTransition(int previous, int current) {
  if (previous == 1 && current == 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    force_scan_at_ = monotonicSeconds() + stop_flush_delay_s_;
    cv_.notify_all();
  }
}

std::vector<CandidateFile> ChunkedArchiveUploader::collectCandidates(const fs::path& dataset_root, bool force) {
  if (archive_kind_ == "camera") return collectCameraCandidates(dataset_root, force);
  return collectStateCandidates(dataset_root, force);
}

std::vector<CandidateFile> ChunkedArchiveUploader::collectCameraCandidates(const fs::path& dataset_root, bool force) {
  std::vector<CandidateFile> out;
  std::error_code ec;
  for (const auto& root : listRecordingRoots(dataset_root)) {
    fs::path camera_root = root.second / "camera_data";
    if (!fs::is_directory(camera_root, ec)) continue;
    for (auto it = fs::recursive_directory_iterator(camera_root, ec); !ec && it != fs::recursive_directory_iterator(); ++it) {
      if (!it->is_regular_file(ec)) continue;
      std::string ext = it->path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext != ".jpg" && ext != ".jpeg") continue;
      if (!force && !isStableFile(it->path(), file_stable_age_s_)) continue;
      out.push_back({it->path(), relativePosix(it->path(), dataset_root), fileSize(it->path()), fileMtimeMs(it->path()), "regular"});
    }
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.rel_path < b.rel_path; });
  return out;
}

std::vector<CandidateFile> ChunkedArchiveUploader::collectStateCandidates(const fs::path& dataset_root, bool force) {
  std::vector<CandidateFile> out;
  for (const auto& root : listRecordingRoots(dataset_root)) {
    appendStateCandidates(dataset_root, root.second / "bag", force, "bag", &out);
    appendStateCandidates(dataset_root, root.second / "grayPicture", force, "gray", &out);
    appendStateCandidates(dataset_root, root.second / "rangePicture", force, "range", &out);
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.rel_path < b.rel_path; });
  return out;
}

void ChunkedArchiveUploader::appendStateCandidates(const fs::path& dataset_root, const fs::path& root, bool force, const std::string& kind, std::vector<CandidateFile>* out) {
  std::error_code ec;
  if (!fs::is_directory(root, ec)) return;
  for (auto it = fs::recursive_directory_iterator(root, ec); !ec && it != fs::recursive_directory_iterator(); ++it) {
    if (!it->is_regular_file(ec)) continue;
    std::string filename = it->path().filename().string();
    if (filename.size() >= 4 && filename.substr(filename.size() - 4) == ".tmp") continue;
    std::string ext = it->path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    std::string cleanup_mode = "regular";
    if (kind == "bag") {
      if (ext != ".json") continue;
      fs::path tmp = it->path();
      tmp += ".tmp";
      cleanup_mode = fs::exists(tmp, ec) ? "bag_snapshot" : "bag_final";
    } else if (kind == "gray" || kind == "range") {
      if (ext != ".png") continue;
      if (!force && !isStableFile(it->path(), file_stable_age_s_)) continue;
    }
    out->push_back({it->path(), relativePosix(it->path(), dataset_root), fileSize(it->path()), fileMtimeMs(it->path()), cleanup_mode});
  }
}

std::vector<std::vector<CandidateFile>> ChunkedArchiveUploader::splitBatches(const std::vector<CandidateFile>& candidates) const {
  std::vector<std::vector<CandidateFile>> batches;
  std::vector<CandidateFile> current;
  uint64_t current_size = 0;
  for (const auto& item : candidates) {
    if (!current.empty() && current_size + item.size_bytes > tar_batch_max_bytes_) {
      batches.push_back(current);
      current.clear();
      current_size = 0;
    }
    current.push_back(item);
    current_size += item.size_bytes;
  }
  if (!current.empty()) batches.push_back(current);
  return batches;
}

Json::Value ChunkedArchiveUploader::buildTask(
    const std::string& dataset_name, const fs::path& dataset_root,
    const std::vector<CandidateFile>& batch, int64_t cycle_ts_ms, int seq) {
  const std::string upload_id = "batch_" + archive_kind_ + "_" + dataset_name + "_" +
      std::to_string(cycle_ts_ms) + "_" + zeroPadded(static_cast<uint64_t>(seq), 4);
  Json::Value task(Json::objectValue);
  task["upload_id"] = upload_id;
  task["archive_kind"] = archive_kind_;
  task["dataset_name"] = dataset_name;
  task["dataset_root_abs"] = dataset_root.string();
  Json::Value source_files(Json::arrayValue);
  for (const auto& item : batch) {
    std::string rel_path;
    std::string path_error;
    if (!normalizeUploadRelativePath(item.rel_path, dataset_name, &rel_path, &path_error)) {
      throw std::runtime_error(path_error);
    }
    Json::Value source(Json::objectValue);
    source["rel_path"] = rel_path;
    source["size_bytes"] = Json::UInt64(item.size_bytes);
    source["mtime_ms"] = Json::Int64(item.mtime_ms);
    source["cleanup_mode"] = item.cleanup_mode;
    source_files.append(source);
  }
  task["source_files"] = source_files;
  task["archive_path"] = (tar_tmp_root_ / (upload_id + ".tar")).string();
  task["manifest_path"] = (tar_tmp_root_ / (upload_id + ".manifest")).string();
  task["status_path"] = (status_root_ / (upload_id + ".json")).string();
  task["archive_size"] = Json::UInt64(0);
  task["total_chunks"] = Json::UInt64(0);
  task["uploaded_chunk_count"] = Json::UInt64(0);
  task["bytes_received"] = Json::UInt64(0);
  task["state"] = "pending";
  task["last_error"] = Json::nullValue;
  const double now = static_cast<double>(unixMs()) / 1000.0;
  task["created_at"] = now;
  task["updated_at"] = now;
  return task;
}

bool ChunkedArchiveUploader::saveTask(Json::Value* task) {
  (*task)["updated_at"] = static_cast<double>(unixMs()) / 1000.0;
  const fs::path status_path((*task)["status_path"].asString());
  Json::Value payload = *task;
  payload.removeMember("status_path");
  return writeTextAtomic(status_path, jsonToString(payload) + "\n");
}

void ChunkedArchiveUploader::removeTask(const Json::Value& task) {
  const fs::path path(task.get("status_path", "").asString());
  if (!path.empty() && isPathUnder(path, status_root_)) removeFileIfExists(path);
}

void ChunkedArchiveUploader::cleanupArtifacts(const Json::Value& task) {
  const fs::path archive_path(task.get("archive_path", "").asString());
  const fs::path manifest_path(task.get("manifest_path", "").asString());
  if (!archive_path.empty() && isPathUnder(archive_path, tar_tmp_root_)) removeFileIfExists(archive_path);
  if (!manifest_path.empty() && isPathUnder(manifest_path, tar_tmp_root_)) removeFileIfExists(manifest_path);
  pruneEmptyTree(tar_tmp_root_);
}

bool ChunkedArchiveUploader::runTar(const fs::path& archive_path,
                                    const fs::path& dataset_root,
                                    const fs::path& manifest_path) const {
  const std::string archive = archive_path.string();
  const std::string root = dataset_root.string();
  const std::string manifest = manifest_path.string();
  const pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    execlp(tar_binary_.c_str(), tar_binary_.c_str(), "-cf", archive.c_str(),
           "-C", root.c_str(), "-T", manifest.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

bool ChunkedArchiveUploader::createArchive(Json::Value* task) {
  ensureDir(tar_tmp_root_);
  std::ostringstream manifest;
  for (const auto& source : (*task)["source_files"]) {
    std::string rel_path;
    std::string path_error;
    if (!normalizeUploadRelativePath(source["rel_path"].asString(),
                                     (*task)["dataset_name"].asString(),
                                     &rel_path, &path_error)) {
      (*task)["state"] = "invalid_source_path";
      (*task)["last_error"] = path_error;
      saveTask(task);
      ROS_WARN("%s tar create rejected invalid path. upload_id=%s error=%s",
               archive_kind_.c_str(), (*task)["upload_id"].asCString(), path_error.c_str());
      return false;
    }
    manifest << rel_path << "\n";
  }
  const fs::path manifest_path((*task)["manifest_path"].asString());
  const fs::path archive_path((*task)["archive_path"].asString());
  if (!writeTextAtomic(manifest_path, manifest.str()) ||
      !runTar(archive_path, fs::path((*task)["dataset_root_abs"].asString()), manifest_path)) {
    removeFileIfExists(archive_path);
    (*task)["state"] = "archive_create_failed";
    (*task)["last_error"] = "tar create failed";
    saveTask(task);
    ROS_WARN("%s tar create failed. upload_id=%s", archive_kind_.c_str(),
             (*task)["upload_id"].asCString());
    return false;
  }
  const uint64_t archive_size = fileSize(archive_path);
  (*task)["archive_size"] = Json::UInt64(archive_size);
  (*task)["total_chunks"] = Json::UInt64(
      std::max<uint64_t>(1, (archive_size + chunk_size_ - 1) / chunk_size_));
  (*task)["state"] = "archive_ready";
  (*task)["last_error"] = Json::nullValue;
  return saveTask(task);
}

ArchiveCheckResult ChunkedArchiveUploader::ensureTaskArchive(Json::Value* task) {
  const fs::path archive_path((*task).get("archive_path", "").asString());
  if (fs::is_regular_file(archive_path)) {
    const uint64_t archive_size = fileSize(archive_path);
    (*task)["archive_size"] = Json::UInt64(archive_size);
    (*task)["total_chunks"] = Json::UInt64(
        std::max<uint64_t>(1, (archive_size + chunk_size_ - 1) / chunk_size_));
    saveTask(task);
    return ArchiveCheckResult::kReady;
  }

  const fs::path dataset_root((*task).get("dataset_root_abs", "").asString());
  const Json::Value& sources = (*task)["source_files"];
  if (dataset_root.empty() || !sources.isArray() || sources.empty()) {
    cleanupArtifacts(*task);
    removeTask(*task);
    return ArchiveCheckResult::kDiscarded;
  }
  for (const auto& source : sources) {
    std::string rel_path;
    std::string path_error;
    if (!normalizeUploadRelativePath(source["rel_path"].asString(),
                                     (*task).get("dataset_name", "").asString(),
                                     &rel_path, &path_error)) {
      ROS_WARN("%s discard task with invalid upload path. upload_id=%s error=%s",
               archive_kind_.c_str(), (*task)["upload_id"].asCString(), path_error.c_str());
      cleanupArtifacts(*task);
      removeTask(*task);
      return ArchiveCheckResult::kDiscarded;
    }
    const fs::path abs_path = (dataset_root / fs::path(rel_path)).lexically_normal();
    const int64_t expected_mtime = source.get("mtime_ms", Json::Int64(0)).asInt64();
    if (!isPathUnder(abs_path, dataset_root) || !fs::is_regular_file(abs_path) ||
        (expected_mtime != 0 && fileMtimeMs(abs_path) != expected_mtime)) {
      ROS_WARN("%s pending task source missing or changed. upload_id=%s path=%s",
               archive_kind_.c_str(), (*task)["upload_id"].asCString(), abs_path.c_str());
      cleanupArtifacts(*task);
      removeTask(*task);
      return ArchiveCheckResult::kDiscarded;
    }
  }
  return createArchive(task) ? ArchiveCheckResult::kReady : ArchiveCheckResult::kFailed;
}

void ChunkedArchiveUploader::recordRemoteFailure(
    Json::Value* task, const std::string& state, const std::string& stage,
    const HttpResponse& response, std::optional<uint64_t> chunk_index) {
  (*task)["state"] = state;
  (*task)["last_error"] = httpResponseError(response);
  (*task)["failed_stage"] = stage;
  if (chunk_index) (*task)["failed_chunk_index"] = Json::UInt64(*chunk_index);
  else (*task)["failed_chunk_index"] = Json::nullValue;
  if (response.status != 0) (*task)["failed_status_code"] = Json::Int64(response.status);
  else (*task)["failed_status_code"] = Json::nullValue;
  (*task)["failed_at"] = static_cast<double>(unixMs()) / 1000.0;
  saveTask(task);
  ++consecutive_remote_failures_;
  if (consecutive_remote_failures_ >= session_recreate_failures_) {
    ROS_WARN("%s uploader recreating NAS HTTP session after %d failures",
             archive_kind_.c_str(), consecutive_remote_failures_);
    http_session_->recreate();
    consecutive_remote_failures_ = 0;
  }
}

void ChunkedArchiveUploader::markRemoteSuccess() {
  consecutive_remote_failures_ = 0;
}

void ChunkedArchiveUploader::updateLocalStatus(Json::Value* task,
                                               const Json::Value& status) {
  (*task)["uploaded_chunk_count"] = status.get("uploaded_chunk_count", Json::UInt64(0));
  (*task)["bytes_received"] = status.get("bytes_received", Json::UInt64(0));
  (*task)["remote_state"] = status.get("state", Json::Value(Json::nullValue));
  (*task)["last_error"] = status.get("last_error", Json::Value(Json::nullValue));
  task->removeMember("failed_stage");
  task->removeMember("failed_chunk_index");
  task->removeMember("failed_status_code");
  task->removeMember("failed_at");
  saveTask(task);
}

void ChunkedArchiveUploader::progressLog(const Json::Value& task,
                                         const Json::Value& status) const {
  const uint64_t total_chunks = std::max<uint64_t>(1, status.get(
      "total_chunks", task.get("total_chunks", Json::UInt64(1))).asUInt64());
  const uint64_t uploaded_chunks = status.get("uploaded_chunk_count", Json::UInt64(0)).asUInt64();
  const uint64_t total_size = std::max<uint64_t>(1, status.get(
      "total_size", task.get("archive_size", Json::UInt64(1))).asUInt64());
  const uint64_t bytes_received = status.get("bytes_received", Json::UInt64(0)).asUInt64();
  const double progress = 100.0 * static_cast<double>(bytes_received) / static_cast<double>(total_size);
  ROS_INFO("%s upload progress. upload_id=%s state=%s chunks=%llu/%llu bytes=%llu/%llu progress=%.1f%%",
           archive_kind_.c_str(), task["upload_id"].asCString(),
           status.get("state", "unknown").asCString(),
           static_cast<unsigned long long>(uploaded_chunks),
           static_cast<unsigned long long>(total_chunks),
           static_cast<unsigned long long>(bytes_received),
           static_cast<unsigned long long>(total_size), progress);
}

void ChunkedArchiveUploader::cleanupUploadedTask(const Json::Value& task) {
  const fs::path dataset_root(task["dataset_root_abs"].asString());
  std::error_code ec;
  for (const auto& source : task["source_files"]) {
    const fs::path abs_path = (dataset_root / fs::path(source["rel_path"].asString())).lexically_normal();
    const int64_t expected_mtime = source.get("mtime_ms", Json::Int64(0)).asInt64();
    if (isPathUnder(abs_path, dataset_root) && fs::exists(abs_path, ec) &&
        (expected_mtime == 0 || fileMtimeMs(abs_path) == expected_mtime)) {
      fs::remove(abs_path, ec);
    }
    if (source.get("cleanup_mode", "regular").asString() == "bag_final") {
      const fs::path sibling_bag = abs_path.parent_path() / (abs_path.stem().string() + ".bag");
      if (isPathUnder(sibling_bag, dataset_root)) fs::remove(sibling_bag, ec);
    }
    pruneEmptyTree(abs_path.parent_path());
  }
  removeDatasetRootIfEmpty(dataset_root);
  cleanupArtifacts(task);
  removeTask(task);
}

bool ChunkedArchiveUploader::executeTask(Json::Value* task) {
  const ArchiveCheckResult archive_check = ensureTaskArchive(task);
  if (archive_check == ArchiveCheckResult::kDiscarded) return true;
  if (archive_check == ArchiveCheckResult::kFailed) return false;

  Json::Value init(Json::objectValue);
  init["upload_id"] = (*task)["upload_id"];
  init["archive_kind"] = (*task)["archive_kind"];
  init["dataset_root"] = (*task)["dataset_name"];
  init["archive_name"] = fs::path((*task)["archive_path"].asString()).filename().string();
  init["total_size"] = (*task)["archive_size"];
  init["chunk_size"] = Json::UInt64(chunk_size_);
  init["total_chunks"] = (*task)["total_chunks"];
  Json::Value source_paths(Json::arrayValue);
  for (const auto& source : (*task)["source_files"]) {
    std::string rel_path;
    std::string path_error;
    if (!normalizeUploadRelativePath(source["rel_path"].asString(),
                                     (*task)["dataset_name"].asString(),
                                     &rel_path, &path_error)) {
      throw std::runtime_error(path_error);
    }
    source_paths.append(rel_path);
  }
  init["source_files"] = source_paths;

  HttpResponse response = http_session_->postJson(
      upload_init_url_, jsonToString(init), connect_timeout_s_, request_timeout_s_);
  if (!response.ok) {
    recordRemoteFailure(task, response.status == 0 ? "remote_init_exception" : "remote_init_failed",
                        "upload_init", response);
    ROS_WARN("%s upload_init failed. upload_id=%s status=%ld error=%s",
             archive_kind_.c_str(), (*task)["upload_id"].asCString(), response.status,
             (*task)["last_error"].asCString());
    return false;
  }
  markRemoteSuccess();
  Json::Value remote(Json::objectValue);
  parseJson(response.body, &remote);
  updateLocalStatus(task, remote);
  progressLog(*task, remote);
  if (remote.get("state", "").asString() == "completed") {
    cleanupUploadedTask(*task);
    return true;
  }

  std::set<uint64_t> uploaded_chunks;
  for (const auto& value : remote["uploaded_chunks"]) uploaded_chunks.insert(value.asUInt64());
  const uint64_t archive_size = (*task)["archive_size"].asUInt64();
  const uint64_t total_chunks = (*task)["total_chunks"].asUInt64();
  std::ifstream in((*task)["archive_path"].asString(), std::ios::binary);
  if (!in) return false;
  for (uint64_t index = 0; index < total_chunks; ++index) {
    if (uploaded_chunks.count(index)) continue;
    const uint64_t offset = index * chunk_size_;
    const uint64_t wanted = std::min<uint64_t>(chunk_size_, archive_size - offset);
    std::vector<char> bytes(static_cast<size_t>(wanted));
    in.seekg(static_cast<std::streamoff>(offset));
    in.read(bytes.data(), static_cast<std::streamsize>(wanted));
    if (static_cast<uint64_t>(in.gcount()) != wanted) {
      HttpResponse local_error;
      local_error.error = "archive chunk read failed";
      recordRemoteFailure(task, "local_chunk_read_failed", "upload_chunk", local_error, index);
      return false;
    }
    response = http_session_->postMultipartChunk(
        upload_chunk_url_, (*task)["upload_id"].asString(), index,
        (*task)["upload_id"].asString() + "." + zeroPadded(index, 6) + ".part",
        bytes, connect_timeout_s_, request_timeout_s_);
    if (!response.ok) {
      recordRemoteFailure(task, response.status == 0 ? "remote_chunk_exception" : "remote_chunk_failed",
                          "upload_chunk", response, index);
      ROS_WARN("%s upload_chunk failed. upload_id=%s chunk=%llu status=%ld error=%s",
               archive_kind_.c_str(), (*task)["upload_id"].asCString(),
               static_cast<unsigned long long>(index), response.status,
               (*task)["last_error"].asCString());
      return false;
    }
    markRemoteSuccess();
    remote = Json::Value(Json::objectValue);
    parseJson(response.body, &remote);
    updateLocalStatus(task, remote);
    progressLog(*task, remote);
  }

  Json::Value complete(Json::objectValue);
  complete["upload_id"] = (*task)["upload_id"];
  response = http_session_->postJson(upload_complete_url_, jsonToString(complete),
                                     connect_timeout_s_, request_timeout_s_);
  if (!response.ok) {
    recordRemoteFailure(task, response.status == 0 ? "remote_complete_exception" : "remote_complete_failed",
                        "upload_complete", response);
    ROS_WARN("%s upload_complete failed. upload_id=%s status=%ld error=%s",
             archive_kind_.c_str(), (*task)["upload_id"].asCString(), response.status,
             (*task)["last_error"].asCString());
    return false;
  }
  markRemoteSuccess();
  remote = Json::Value(Json::objectValue);
  parseJson(response.body, &remote);
  updateLocalStatus(task, remote);
  progressLog(*task, remote);
  cleanupUploadedTask(*task);
  return true;
}

bool ChunkedArchiveUploader::isLegacyTask(const Json::Value& task) const {
  if (!task.isObject() || !task.isMember("dataset_root_abs") ||
      !task.isMember("archive_path") || !task.isMember("manifest_path") ||
      !task["source_files"].isArray()) return true;
  const std::string upload_id = task.get("upload_id", "").asString();
  const std::regex expected_id("^batch_" + archive_kind_ +
      "_excavator_data_[0-9]{8}_[0-9]{13}_[0-9]{4}$");
  if (!std::regex_match(upload_id, expected_id) ||
      task.get("archive_kind", "").asString() != archive_kind_ ||
      !isPathUnder(fs::path(task["dataset_root_abs"].asString()), scan_root_) ||
      !isPathUnder(fs::path(task["archive_path"].asString()), tar_tmp_root_) ||
      !isPathUnder(fs::path(task["manifest_path"].asString()), tar_tmp_root_)) return true;
  for (const auto& source : task["source_files"]) {
    if (!source.isObject() || !source.isMember("rel_path") || !source["rel_path"].isString()) return true;
    std::string normalized;
    if (!normalizeUploadRelativePath(source["rel_path"].asString(),
                                     task.get("dataset_name", "").asString(),
                                     &normalized, nullptr)) return true;
  }
  return false;
}

void ChunkedArchiveUploader::cleanupLegacyTask(const fs::path& status_path,
                                               const Json::Value& task) {
  const std::string upload_id = task.get("upload_id", status_path.stem().string()).asString();
  if (fs::path(upload_id).filename().string() == upload_id) {
    removeFileIfExists(tar_tmp_root_ / (upload_id + ".tar"));
    removeFileIfExists(tar_tmp_root_ / (upload_id + ".manifest"));
  }
  removeFileIfExists(status_path);
  ROS_WARN("%s removed legacy upload task; source data retained. upload_id=%s",
           archive_kind_.c_str(), upload_id.c_str());
}

bool ChunkedArchiveUploader::resumePendingTasks() {
  std::vector<fs::path> status_files;
  std::error_code ec;
  if (!fs::is_directory(status_root_, ec)) return false;
  for (const auto& entry : fs::directory_iterator(status_root_, ec)) {
    if (entry.is_regular_file(ec) && entry.path().extension() == ".json") {
      status_files.push_back(entry.path());
    }
  }
  std::sort(status_files.begin(), status_files.end());
  for (const auto& status_path : status_files) {
    Json::Value task;
    if (!readJsonFile(status_path, &task)) {
      fs::path invalid_path = status_path;
      invalid_path += ".invalid";
      removeFileIfExists(invalid_path);
      fs::rename(status_path, invalid_path, ec);
      ROS_ERROR("discard invalid %s task file. path=%s", archive_kind_.c_str(), status_path.c_str());
      continue;
    }
    if (isLegacyTask(task)) {
      cleanupLegacyTask(status_path, task);
      continue;
    }
    task["status_path"] = status_path.string();
    if (!executeTask(&task)) return true;
  }
  return false;
}

void ChunkedArchiveUploader::runLoop() {
  double next_cycle_at = monotonicSeconds();
  while (ros::ok()) {
    bool force = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stop_) break;
      double now = monotonicSeconds();
      if (force_scan_at_ && now >= *force_scan_at_) {
        force = true;
        force_scan_at_.reset();
      } else if (now < next_cycle_at) {
        double wake_at = next_cycle_at;
        if (force_scan_at_) wake_at = std::min(wake_at, *force_scan_at_);
        cv_.wait_for(lock, std::chrono::duration<double>(std::max(0.1, wake_at - now)));
        continue;
      }
    }
    try {
      const bool blocked = resumePendingTasks();
      if (!blocked) {
        if (archive_kind_ == "state") node_->prepareUploadArtifacts();
        runUploadCycle(force);
      }
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(5.0, "%s upload cycle failed: %s", archive_kind_.c_str(), e.what());
    }
    next_cycle_at = monotonicSeconds() + upload_interval_s_;
  }
}

void ChunkedArchiveUploader::runUploadCycle(bool force) {
  if (!fs::is_directory(scan_root_)) {
    ROS_WARN_THROTTLE(5.0, "%s scan_root does not exist yet: %s", archive_kind_.c_str(), scan_root_.c_str());
    return;
  }
  int uploaded_archives = 0;
  int uploaded_files = 0;
  int failed_archives = 0;
  const int64_t cycle_ts_ms = unixMs();
  for (const auto& dataset : listDatasetDirs(scan_root_)) {
    auto candidates = collectCandidates(dataset.second, force);
    if (candidates.empty()) {
      removeDatasetRootIfEmpty(dataset.second);
      continue;
    }
    auto batches = splitBatches(candidates);
    int seq = 1;
    for (const auto& batch : batches) {
      Json::Value task = buildTask(dataset.first, dataset.second, batch, cycle_ts_ms, seq++);
      if (!saveTask(&task) || !executeTask(&task)) {
        ++failed_archives;
        return;
      }
      ++uploaded_archives;
      uploaded_files += static_cast<int>(batch.size());
    }
    removeDatasetRootIfEmpty(dataset.second);
  }
  if (uploaded_archives || failed_archives) {
    ROS_INFO("%s upload cycle finished. archives=%d files=%d failed=%d force=%s",
             archive_kind_.c_str(), uploaded_archives, uploaded_files, failed_archives,
             force ? "true" : "false");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_to_NAS_and_ubuntu2204");
  try {
    UnifiedDataNode node;
    ros::spin();
    node.shutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("data_to_NAS_and_ubuntu2204 C++ fatal error: %s", e.what());
    return 1;
  }
  return 0;
}
