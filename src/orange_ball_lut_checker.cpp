#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace {

struct CommandLineOptions
{
  std::filesystem::path lut_spec;
  int expected_frame_width = -1;
  int expected_frame_height = -1;
};

struct LutData
{
  cv::Mat ground_x_m;
  cv::Mat ground_y_m;
  cv::Mat valid_mask;
  cv::Mat area_prior_distance_m;
  cv::Mat area_prior_expected_area_px;
  int source_width = 0;
  int source_height = 0;
  double ocam_xc = 0.0;
  double ocam_yc = 0.0;
  double camera_height_m = 0.0;
  double ball_diameter_m = 0.0;
  int original_ground_x_type = -1;
  int original_ground_y_type = -1;
  int original_valid_mask_type = -1;
  int original_area_prior_distance_type = -1;
  int original_area_prior_expected_area_type = -1;
};

std::string pathToString(const std::filesystem::path & path)
{
  return path.string();
}

std::string cvTypeToString(int type)
{
  const int depth = type & CV_MAT_DEPTH_MASK;
  const int channels = 1 + (type >> CV_CN_SHIFT);

  std::string depth_str;
  switch (depth) {
    case CV_8U:
      depth_str = "8U";
      break;
    case CV_8S:
      depth_str = "8S";
      break;
    case CV_16U:
      depth_str = "16U";
      break;
    case CV_16S:
      depth_str = "16S";
      break;
    case CV_32S:
      depth_str = "32S";
      break;
    case CV_32F:
      depth_str = "32F";
      break;
    case CV_64F:
      depth_str = "64F";
      break;
    default:
      depth_str = "User";
      break;
  }

  std::ostringstream oss;
  oss << "CV_" << depth_str << "C" << channels;
  return oss.str();
}

bool isFinite(double value)
{
  return std::isfinite(value);
}

void printUsage(const char * program_name)
{
  std::cout
    << "Usage:\n"
    << "  " << program_name << " [--lut <filename-or-path>] [--frame-size <width>x<height>]\n"
    << "  " << program_name << " [--lut <filename-or-path>] [--frame-width <w> --frame-height <h>]\n"
    << "\n"
    << "Notes:\n"
    << "  - If --lut is omitted, the checker loads the newest raw_ball_top_lut*.xml from config/.\n"
    << "  - If --lut is a bare filename, the checker looks for it inside config/.\n"
    << "  - If --lut is a path with directories or an absolute path, that path is used directly.\n";
}

int parsePositiveInt(const std::string & text, const char * option_name)
{
  std::size_t parsed = 0;
  int value = 0;
  try {
    value = std::stoi(text, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid integer for option " + std::string(option_name) + ": " + text);
  }

  if (parsed != text.size() || value <= 0) {
    throw std::runtime_error("Option " + std::string(option_name) + " must be a positive integer.");
  }
  return value;
}

CommandLineOptions parseCommandLine(int argc, char ** argv)
{
  CommandLineOptions options;

  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string arg = argv[arg_index];

    auto consume_value = [&](const char * option_name) -> std::string {
        if (arg_index + 1 >= argc) {
          throw std::runtime_error("Missing value for option " + std::string(option_name));
        }
        ++arg_index;
        return argv[arg_index];
      };

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }

    if (arg == "--lut") {
      options.lut_spec = consume_value("--lut");
      continue;
    }

    if (arg == "--frame-width") {
      options.expected_frame_width = parsePositiveInt(consume_value("--frame-width"), "--frame-width");
      continue;
    }

    if (arg == "--frame-height") {
      options.expected_frame_height = parsePositiveInt(consume_value("--frame-height"), "--frame-height");
      continue;
    }

    if (arg == "--frame-size") {
      const std::string value = consume_value("--frame-size");
      const std::size_t sep = value.find('x');
      if (sep == std::string::npos) {
        throw std::runtime_error("Option --frame-size must look like <width>x<height>.");
      }
      options.expected_frame_width = parsePositiveInt(value.substr(0, sep), "--frame-size");
      options.expected_frame_height = parsePositiveInt(value.substr(sep + 1), "--frame-size");
      continue;
    }

    throw std::runtime_error("Unknown argument: " + arg);
  }

  if ((options.expected_frame_width > 0 && options.expected_frame_height <= 0) ||
    (options.expected_frame_height > 0 && options.expected_frame_width <= 0))
  {
    throw std::runtime_error(
            "Frame size must provide both width and height. Use --frame-size or both "
            "--frame-width and --frame-height.");
  }

  return options;
}

std::filesystem::path resolveConfigDir(const std::filesystem::path & program_path)
{
  const std::filesystem::path executable_dir = program_path.parent_path();
  const std::vector<std::filesystem::path> candidates = {
    std::filesystem::current_path() / "config",
    executable_dir / "config",
    executable_dir.parent_path().parent_path() / "share" / "rcj_localization" / "config"
  };

  for (const auto & candidate : candidates) {
    if (std::filesystem::exists(candidate) && std::filesystem::is_directory(candidate)) {
      return std::filesystem::canonical(candidate);
    }
  }

  throw std::runtime_error(
          "Cannot locate config directory. Tried ./config and install/share/rcj_localization/config.");
}

std::filesystem::path resolveNewestRawBallLutPath(const std::filesystem::path & config_dir)
{
  std::filesystem::path best_path;
  std::filesystem::file_time_type best_time{};

  for (const auto & entry : std::filesystem::directory_iterator(config_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const std::string filename = entry.path().filename().string();
    if (filename.rfind("raw_ball_top_lut", 0) != 0 || entry.path().extension() != ".xml") {
      continue;
    }

    const auto write_time = entry.last_write_time();
    if (best_path.empty() || write_time > best_time) {
      best_path = entry.path();
      best_time = write_time;
    }
  }

  if (best_path.empty()) {
    throw std::runtime_error("No raw_ball_top_lut*.xml files found in config directory: " + pathToString(config_dir));
  }

  return best_path;
}

std::filesystem::path resolveLutPath(
  const CommandLineOptions & options,
  const std::filesystem::path & program_path)
{
  const std::filesystem::path config_dir = resolveConfigDir(program_path);

  if (options.lut_spec.empty()) {
    return resolveNewestRawBallLutPath(config_dir);
  }

  if (options.lut_spec.is_absolute() || options.lut_spec.has_parent_path()) {
    if (!std::filesystem::exists(options.lut_spec)) {
      throw std::runtime_error("Specified LUT path does not exist: " + pathToString(options.lut_spec));
    }
    return std::filesystem::canonical(options.lut_spec);
  }

  const std::filesystem::path lut_path = config_dir / options.lut_spec;
  if (!std::filesystem::exists(lut_path)) {
    throw std::runtime_error(
            "Specified LUT file was not found in config directory: " + pathToString(lut_path));
  }
  return std::filesystem::canonical(lut_path);
}

LutData loadLutFileLikeDetector(const std::filesystem::path & lut_path)
{
  cv::FileStorage fs(pathToString(lut_path), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Cannot open LUT file: " + pathToString(lut_path));
  }

  LutData lut;
  fs["ground_x_m"] >> lut.ground_x_m;
  fs["ground_y_m"] >> lut.ground_y_m;
  fs["valid_mask"] >> lut.valid_mask;
  fs["area_prior_distance_m"] >> lut.area_prior_distance_m;
  fs["area_prior_expected_area_px"] >> lut.area_prior_expected_area_px;
  fs["source_width"] >> lut.source_width;
  fs["source_height"] >> lut.source_height;
  fs["ocam_xc"] >> lut.ocam_xc;
  fs["ocam_yc"] >> lut.ocam_yc;
  fs["camera_height_m"] >> lut.camera_height_m;
  fs["ball_diameter_m"] >> lut.ball_diameter_m;
  fs.release();

  if (
    lut.ground_x_m.empty() || lut.ground_y_m.empty() || lut.valid_mask.empty() ||
    lut.ground_x_m.size() != lut.ground_y_m.size() ||
    lut.ground_x_m.size() != lut.valid_mask.size())
  {
    throw std::runtime_error("LUT file is missing required matrices: " + pathToString(lut_path));
  }
  if (lut.area_prior_distance_m.empty() || lut.area_prior_expected_area_px.empty()) {
    throw std::runtime_error(
            "LUT file is missing area_prior_distance_m / area_prior_expected_area_px. "
            "The detector now requires a regenerated LUT: " + pathToString(lut_path));
  }

  if (lut.source_width <= 0 || lut.source_height <= 0) {
    throw std::runtime_error("LUT file has invalid source dimensions: " + pathToString(lut_path));
  }

  lut.original_ground_x_type = lut.ground_x_m.type();
  lut.original_ground_y_type = lut.ground_y_m.type();
  lut.original_valid_mask_type = lut.valid_mask.type();
  lut.original_area_prior_distance_type = lut.area_prior_distance_m.type();
  lut.original_area_prior_expected_area_type = lut.area_prior_expected_area_px.type();

  if (lut.ground_x_m.type() != CV_32FC1) {
    lut.ground_x_m.convertTo(lut.ground_x_m, CV_32FC1);
  }
  if (lut.ground_y_m.type() != CV_32FC1) {
    lut.ground_y_m.convertTo(lut.ground_y_m, CV_32FC1);
  }
  if (lut.valid_mask.type() != CV_8UC1) {
    cv::Mat converted;
    lut.valid_mask.convertTo(converted, CV_8UC1);
    lut.valid_mask = converted;
  }
  if (lut.area_prior_distance_m.type() != CV_32FC1) {
    lut.area_prior_distance_m.convertTo(lut.area_prior_distance_m, CV_32FC1);
  }
  if (lut.area_prior_expected_area_px.type() != CV_32FC1) {
    lut.area_prior_expected_area_px.convertTo(lut.area_prior_expected_area_px, CV_32FC1);
  }

  return lut;
}

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void validateLutContract(const LutData & lut, const CommandLineOptions & options)
{
  require(
    lut.ground_x_m.type() == CV_32FC1,
    "ground_x_m is not CV_32FC1 after detector-style conversion.");
  require(
    lut.ground_y_m.type() == CV_32FC1,
    "ground_y_m is not CV_32FC1 after detector-style conversion.");
  require(
    lut.valid_mask.type() == CV_8UC1,
    "valid_mask is not CV_8UC1 after detector-style conversion.");
  require(
    lut.area_prior_distance_m.type() == CV_32FC1,
    "area_prior_distance_m is not CV_32FC1 after detector-style conversion.");
  require(
    lut.area_prior_expected_area_px.type() == CV_32FC1,
    "area_prior_expected_area_px is not CV_32FC1 after detector-style conversion.");

  require(
    lut.ground_x_m.cols == lut.source_width && lut.ground_x_m.rows == lut.source_height,
    "LUT matrix size does not match source_width/source_height. Detector later compares frame size "
    "against source_width/source_height.");
  require(
    (lut.area_prior_distance_m.rows == 1 || lut.area_prior_distance_m.cols == 1) &&
    (lut.area_prior_expected_area_px.rows == 1 || lut.area_prior_expected_area_px.cols == 1),
    "area_prior_distance_m and area_prior_expected_area_px must be stored as 1D vectors.");
  require(
    lut.area_prior_distance_m.total() == lut.area_prior_expected_area_px.total() &&
    lut.area_prior_distance_m.total() >= 2U,
    "Area-prior LUT vectors must have matching lengths and contain at least 2 samples.");

  require(isFinite(lut.ocam_xc) && isFinite(lut.ocam_yc), "ocam_xc/ocam_yc must be finite.");
  require(
    isFinite(lut.camera_height_m) && isFinite(lut.ball_diameter_m),
    "camera_height_m and ball_diameter_m must be finite.");

  const int valid_pixels = cv::countNonZero(lut.valid_mask);
  require(valid_pixels > 0, "valid_mask contains no valid pixels.");

  const cv::Mat area_prior_distance_flat = lut.area_prior_distance_m.reshape(1, 1);
  const cv::Mat area_prior_expected_area_flat = lut.area_prior_expected_area_px.reshape(1, 1);
  double min_area_prior_distance = std::numeric_limits<double>::infinity();
  double max_area_prior_distance = -std::numeric_limits<double>::infinity();
  double max_expected_area = -std::numeric_limits<double>::infinity();
  double min_expected_area = std::numeric_limits<double>::infinity();
  for (int i = 0; i < area_prior_distance_flat.cols; ++i) {
    const double distance_m = static_cast<double>(area_prior_distance_flat.at<float>(0, i));
    const double expected_area_px =
      static_cast<double>(area_prior_expected_area_flat.at<float>(0, i));
    require(std::isfinite(distance_m), "Area-prior distances must be finite.");
    require(
      std::isfinite(expected_area_px) && expected_area_px > 0.0,
      "Area-prior expected areas must be finite and positive.");
    if (i > 0) {
      const double prev_distance_m =
        static_cast<double>(area_prior_distance_flat.at<float>(0, i - 1));
      const double prev_expected_area_px =
        static_cast<double>(area_prior_expected_area_flat.at<float>(0, i - 1));
      require(
        distance_m > prev_distance_m,
        "area_prior_distance_m must be strictly increasing.");
      require(
        expected_area_px <= prev_expected_area_px + 1e-3,
        "area_prior_expected_area_px must be monotonic non-increasing.");
    }

    min_area_prior_distance = std::min(min_area_prior_distance, distance_m);
    max_area_prior_distance = std::max(max_area_prior_distance, distance_m);
    max_expected_area = std::max(max_expected_area, expected_area_px);
    min_expected_area = std::min(min_expected_area, expected_area_px);
  }

  std::size_t non_binary_mask_pixels = 0;
  std::size_t invalid_nonzero_ground_pixels = 0;
  std::size_t nonfinite_valid_ground_pixels = 0;
  double min_ground_x = std::numeric_limits<double>::infinity();
  double max_ground_x = -std::numeric_limits<double>::infinity();
  double min_ground_y = std::numeric_limits<double>::infinity();
  double max_ground_y = -std::numeric_limits<double>::infinity();

  for (int y = 0; y < lut.ground_x_m.rows; ++y) {
    for (int x = 0; x < lut.ground_x_m.cols; ++x) {
      const uchar mask_value = lut.valid_mask.at<uchar>(y, x);
      const float ground_x = lut.ground_x_m.at<float>(y, x);
      const float ground_y = lut.ground_y_m.at<float>(y, x);

      if (mask_value != 0 && mask_value != 1) {
        ++non_binary_mask_pixels;
      }

      if (mask_value == 0) {
        if (ground_x != 0.0F || ground_y != 0.0F) {
          ++invalid_nonzero_ground_pixels;
        }
        continue;
      }

      if (!std::isfinite(ground_x) || !std::isfinite(ground_y)) {
        ++nonfinite_valid_ground_pixels;
        continue;
      }

      min_ground_x = std::min(min_ground_x, static_cast<double>(ground_x));
      max_ground_x = std::max(max_ground_x, static_cast<double>(ground_x));
      min_ground_y = std::min(min_ground_y, static_cast<double>(ground_y));
      max_ground_y = std::max(max_ground_y, static_cast<double>(ground_y));
    }
  }

  require(
    nonfinite_valid_ground_pixels == 0,
    "Some valid_mask pixels map to non-finite ground coordinates.");

  if (options.expected_frame_width > 0) {
    require(
      options.expected_frame_width == lut.source_width &&
      options.expected_frame_height == lut.source_height,
      "Provided frame size does not match LUT source_width/source_height.");
  }

  const double valid_ratio =
    static_cast<double>(valid_pixels) /
    static_cast<double>(lut.valid_mask.rows * lut.valid_mask.cols);
  const int center_x = static_cast<int>(std::lround(lut.ocam_xc));
  const int center_y = static_cast<int>(std::lround(lut.ocam_yc));
  const bool center_in_bounds =
    center_x >= 0 && center_x < lut.valid_mask.cols &&
    center_y >= 0 && center_y < lut.valid_mask.rows;

  std::cout << "LUT check passed\n";
  std::cout << "  File: " << pathToString(options.lut_spec) << "\n";
  std::cout << "  Matrix size: " << lut.ground_x_m.cols << "x" << lut.ground_x_m.rows << "\n";
  std::cout << "  source_width/source_height: " << lut.source_width << "x" << lut.source_height << "\n";
  std::cout << "  Original matrix types: "
            << "ground_x_m=" << cvTypeToString(lut.original_ground_x_type) << ", "
            << "ground_y_m=" << cvTypeToString(lut.original_ground_y_type) << ", "
            << "valid_mask=" << cvTypeToString(lut.original_valid_mask_type) << ", "
            << "area_prior_distance_m=" << cvTypeToString(lut.original_area_prior_distance_type) << ", "
            << "area_prior_expected_area_px=" << cvTypeToString(lut.original_area_prior_expected_area_type) << "\n";
  std::cout << "  Detector runtime types: "
            << "ground_x_m=" << cvTypeToString(lut.ground_x_m.type()) << ", "
            << "ground_y_m=" << cvTypeToString(lut.ground_y_m.type()) << ", "
            << "valid_mask=" << cvTypeToString(lut.valid_mask.type()) << ", "
            << "area_prior_distance_m=" << cvTypeToString(lut.area_prior_distance_m.type()) << ", "
            << "area_prior_expected_area_px=" << cvTypeToString(lut.area_prior_expected_area_px.type()) << "\n";
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "  ocam center: (" << lut.ocam_xc << ", " << lut.ocam_yc << ")\n";
  std::cout << "  camera_height_m=" << lut.camera_height_m
            << ", ball_diameter_m=" << lut.ball_diameter_m << "\n";
  std::cout << "  Area prior distance range: [" << min_area_prior_distance
            << ", " << max_area_prior_distance << "] m\n";
  std::cout << "  Area prior expected area range: [" << min_expected_area
            << ", " << max_expected_area << "] px\n";
  std::cout << "  Valid coverage: " << valid_pixels << " / "
            << (lut.valid_mask.rows * lut.valid_mask.cols)
            << " (" << (valid_ratio * 100.0) << "%)\n";
  std::cout << "  Ground X range on valid pixels: [" << min_ground_x << ", " << max_ground_x << "]\n";
  std::cout << "  Ground Y range on valid pixels: [" << min_ground_y << ", " << max_ground_y << "]\n";

  if (center_in_bounds) {
    std::cout << "  Center sample: valid="
              << static_cast<int>(lut.valid_mask.at<uchar>(center_y, center_x))
              << ", ground_x=" << lut.ground_x_m.at<float>(center_y, center_x)
              << ", ground_y=" << lut.ground_y_m.at<float>(center_y, center_x) << "\n";
  } else {
    std::cout << "  Center sample: ocam center rounds outside matrix bounds\n";
  }

  if (non_binary_mask_pixels > 0) {
    std::cout << "  Note: valid_mask contains " << non_binary_mask_pixels
              << " non-binary nonzero values. Detector accepts this because it checks > 0.\n";
  }

  if (invalid_nonzero_ground_pixels > 0) {
    std::cout << "  Note: " << invalid_nonzero_ground_pixels
              << " invalid-mask pixels still carry nonzero ground values. Detector ignores them.\n";
  }

  if (options.expected_frame_width > 0) {
    std::cout << "  Frame size check: matched "
              << options.expected_frame_width << "x" << options.expected_frame_height << "\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const CommandLineOptions options = parseCommandLine(argc, argv);
    CommandLineOptions resolved_options = options;
    resolved_options.lut_spec = resolveLutPath(options, argv[0]);
    const LutData lut = loadLutFileLikeDetector(resolved_options.lut_spec);
    validateLutContract(lut, resolved_options);
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "LUT check failed: " << ex.what() << "\n";
    printUsage(argv[0]);
    return 1;
  }
}
