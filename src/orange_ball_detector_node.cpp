#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

constexpr int kHueMax = 179;
constexpr int kByteMax = 255;
constexpr char kInputWindowName[] = "Orange Ball Input";
constexpr char kThresholdMaskWindowName[] = "Orange Ball Threshold Mask";
constexpr char kMorphMaskWindowName[] = "Orange Ball Morph Mask";
constexpr char kOverlayWindowName[] = "Orange Ball Overlay";
constexpr char kRawMaskWindowName[] = "Orange Ball Raw Mask";
constexpr char kFilteredMaskWindowName[] = "Orange Ball Filtered Mask";
constexpr char kAreaFilterWindowName[] = "Orange Ball Area Filter";
constexpr char kAreaPriorFilterWindowName[] = "Orange Ball Area-Prior Filter";
constexpr char kAreaPriorDebugWindowName[] = "Orange Ball Area-Prior Debug";
constexpr char kAspectFilterWindowName[] = "Orange Ball Aspect Filter";
constexpr char kEdgeFilterWindowName[] = "Orange Ball Edge Filter";
constexpr char kFillFilterWindowName[] = "Orange Ball Fill Filter";
constexpr char kLutFilterWindowName[] = "Orange Ball LUT Filter";
constexpr char kTopBandFilterWindowName[] = "Orange Ball Top-Band Filter";
constexpr char kSearchDebugWindowName[] = "Orange Ball Search Debug";
constexpr char kSearchAreaPriorWindowName[] = "Orange Ball Search Area-Prior";
constexpr char kRoiWindowName[] = "Orange Ball ROI";
constexpr char kRoiMaskWindowName[] = "Orange Ball ROI Mask";
constexpr std::size_t kTimingStageCount = 8;
constexpr double kMinValidLutCoverageRatio = 0.35;
constexpr double kMinTopBandValidRatio = 0.35;
constexpr double kMinFillRatio = 0.15;

enum class TrackingMode {
  Search,
  Track,
};

enum class TimingStage : std::size_t {
  CvBridge = 0,
  Search,
  Track,
  Refine,
  Publish,
  Debug,
  CallbackTotal,
  UnaccountedOverhead,
};

constexpr std::array<const char *, kTimingStageCount> kTimingLabels = {
  "cv_bridge",
  "search",
  "track",
  "refine",
  "publish",
  "debug",
  "callback_total",
  "unaccounted_overhead",
};

long long elapsedUs(const TimePoint & start, const TimePoint & end)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

int clampToByte(int value)
{
  return std::clamp(value, 0, kByteMax);
}

int clampHue(int value)
{
  return std::clamp(value, 0, kHueMax);
}

cv::Rect clampRectToImage(const cv::Rect & rect, const cv::Size & image_size)
{
  const int x = std::clamp(rect.x, 0, image_size.width);
  const int y = std::clamp(rect.y, 0, image_size.height);
  const int max_width = std::max(0, image_size.width - x);
  const int max_height = std::max(0, image_size.height - y);
  const int width = std::clamp(rect.width, 0, max_width);
  const int height = std::clamp(rect.height, 0, max_height);
  return cv::Rect(x, y, width, height);
}

cv::Rect scaleRectAroundCenter(
  const cv::Rect & rect,
  double scale,
  const cv::Size & image_size)
{
  if (rect.width <= 0 || rect.height <= 0) {
    return cv::Rect(0, 0, image_size.width, image_size.height);
  }

  const double center_x = static_cast<double>(rect.x) + static_cast<double>(rect.width) * 0.5;
  const double center_y = static_cast<double>(rect.y) + static_cast<double>(rect.height) * 0.5;
  const int scaled_width =
    std::max(1, static_cast<int>(std::round(static_cast<double>(rect.width) * scale)));
  const int scaled_height =
    std::max(1, static_cast<int>(std::round(static_cast<double>(rect.height) * scale)));
  const int x = static_cast<int>(std::round(center_x - static_cast<double>(scaled_width) * 0.5));
  const int y = static_cast<int>(std::round(center_y - static_cast<double>(scaled_height) * 0.5));
  return clampRectToImage(cv::Rect(x, y, scaled_width, scaled_height), image_size);
}

cv::Rect buildCenteredRect(
  const cv::Point2d & center,
  int half_width,
  int half_height,
  const cv::Size & image_size)
{
  const int width = std::max(1, half_width * 2);
  const int height = std::max(1, half_height * 2);
  const int x = static_cast<int>(std::round(center.x)) - half_width;
  const int y = static_cast<int>(std::round(center.y)) - half_height;
  return clampRectToImage(cv::Rect(x, y, width, height), image_size);
}

double computeEdgeTouchRatio(const cv::Rect & bbox, const cv::Size & image_size)
{
  if (bbox.width <= 0 || bbox.height <= 0) {
    return 1.0;
  }

  double touch_length = 0.0;
  if (bbox.x <= 0) {
    touch_length += static_cast<double>(bbox.height);
  }
  if (bbox.y <= 0) {
    touch_length += static_cast<double>(bbox.width);
  }
  if (bbox.x + bbox.width >= image_size.width) {
    touch_length += static_cast<double>(bbox.height);
  }
  if (bbox.y + bbox.height >= image_size.height) {
    touch_length += static_cast<double>(bbox.width);
  }

  const double perimeter = static_cast<double>(bbox.width + bbox.height) * 2.0;
  if (perimeter <= 0.0) {
    return 1.0;
  }
  return clamp01(touch_length / perimeter);
}

cv::Size fitWithinBounds(const cv::Size & image_size, int max_width, int max_height)
{
  const int safe_max_width = std::max(1, max_width);
  const int safe_max_height = std::max(1, max_height);
  if (image_size.width <= 0 || image_size.height <= 0) {
    return cv::Size(safe_max_width, safe_max_height);
  }

  const double width_scale =
    static_cast<double>(safe_max_width) / static_cast<double>(image_size.width);
  const double height_scale =
    static_cast<double>(safe_max_height) / static_cast<double>(image_size.height);
  const double scale = std::min(1.0, std::min(width_scale, height_scale));
  return cv::Size(
    std::max(1, static_cast<int>(std::round(static_cast<double>(image_size.width) * scale))),
    std::max(1, static_cast<int>(std::round(static_cast<double>(image_size.height) * scale))));
}

void resizeWindowToFitImage(
  const std::string & window_name,
  const cv::Mat & image,
  int max_width,
  int max_height)
{
  const cv::Size fitted_size = fitWithinBounds(image.size(), max_width, max_height);
  cv::resizeWindow(window_name, fitted_size.width, fitted_size.height);
}

std::string trackingModeToString(TrackingMode mode)
{
  return mode == TrackingMode::Search ? "SEARCH" : "TRACK";
}

cv::Point clampPointToImage(const cv::Point & point, const cv::Size & image_size)
{
  return cv::Point(
    std::clamp(point.x, 0, std::max(0, image_size.width - 1)),
    std::clamp(point.y, 0, std::max(0, image_size.height - 1)));
}

std::filesystem::path resolvePath(const std::string & raw_path)
{
  if (raw_path.empty()) {
    return {};
  }

  std::filesystem::path resolved_path;
  if (raw_path.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home == nullptr || std::string(home).empty()) {
      throw std::runtime_error("HOME is not set; cannot resolve '~' in robot_mask_path");
    }

    if (raw_path.size() == 1) {
      resolved_path = std::filesystem::path(home);
    } else if (raw_path[1] == '/') {
      resolved_path = std::filesystem::path(home) / raw_path.substr(2);
    } else {
      throw std::runtime_error("Only '~' and '~/' are supported in robot_mask_path");
    }
  } else {
    resolved_path = std::filesystem::path(raw_path);
  }

  if (resolved_path.is_relative()) {
    resolved_path = std::filesystem::absolute(resolved_path);
  }

  return resolved_path.lexically_normal();
}

std::string packageConfigPath(const std::string & filename)
{
  return (std::filesystem::path(
            ament_index_cpp::get_package_share_directory("rcj_localization")) /
          "config" / filename)
    .string();
}

struct LutData
{
  cv::Mat ground_x_m;
  cv::Mat ground_y_m;
  cv::Mat valid_mask;
  cv::Mat area_prior_distance_m;
  cv::Mat area_prior_expected_area_px;
  std::vector<double> area_prior_distance_samples_m;
  std::vector<double> area_prior_expected_area_samples_px;
  int source_width = 0;
  int source_height = 0;
  double ocam_xc = 0.0;
  double ocam_yc = 0.0;
  double camera_height_m = 0.0;
  double ball_diameter_m = 0.0;
};

struct AreaPriorEvaluation
{
  bool valid = false;
  bool passed = false;
  double distance_m = 0.0;
  double expected_area_px = 0.0;
  double min_ratio = 0.0;
  double max_ratio = 0.0;
  double min_area_px = 0.0;
  double max_area_px = 0.0;
  double area_score = 0.0;
};

struct Candidate
{
  bool valid = false;
  cv::Rect bbox;
  std::vector<cv::Point> pixels;
  std::vector<cv::Point> top_band_pixels;
  cv::Point2d raw_center_px;
  cv::Point2d top_point_px;
  cv::Point2d ground_center_m;
  double area_px = 0.0;
  double aspect_ratio = 0.0;
  double fill_ratio = 0.0;
  double edge_touch_ratio = 0.0;
  double valid_lut_ratio = 0.0;
  double top_band_valid_ratio = 0.0;
  AreaPriorEvaluation area_prior;
  double score = -std::numeric_limits<double>::infinity();
  double confidence = 0.0;
};

struct DetectorOutputs
{
  Candidate candidate;
  cv::Mat threshold_debug_mask;
  cv::Mat morph_debug_mask;
  cv::Mat raw_debug_mask;
  cv::Mat filtered_debug_mask;
  cv::Mat area_debug_mask;
  cv::Mat area_prior_filter_debug_mask;
  cv::Mat aspect_debug_mask;
  cv::Mat edge_debug_mask;
  cv::Mat fill_debug_mask;
  cv::Mat lut_debug_mask;
  cv::Mat top_band_debug_mask;
  cv::Mat search_debug_image;
  cv::Mat area_prior_debug_image;
  cv::Mat search_area_prior_debug_image;
  cv::Mat roi_debug_mask;
  cv::Rect roi;
};

struct CoarseCandidate
{
  bool valid = false;
  cv::Rect bbox;
  double area_px = 0.0;
  AreaPriorEvaluation area_prior;
  cv::Point raw_reference_point;
  double score = -std::numeric_limits<double>::infinity();
};

struct DetectionDebugOptions
{
  bool capture_threshold_mask = false;
  bool capture_morph_mask = false;
  bool capture_raw_mask = false;
  bool capture_filtered_mask = false;
  bool capture_area_mask = false;
  bool capture_area_prior_filter_mask = false;
  bool capture_area_prior_debug = false;
  bool capture_aspect_mask = false;
  bool capture_edge_mask = false;
  bool capture_fill_mask = false;
  bool capture_lut_mask = false;
  bool capture_top_band_mask = false;
  bool capture_search_debug = false;
  bool capture_search_area_prior = false;
  bool capture_roi_mask = false;
};

struct DebugRequest
{
  bool publish_raw_mask = false;
  bool publish_filtered_mask = false;
  bool publish_overlay = false;
  bool show_input_image = false;
  bool show_threshold_mask = false;
  bool show_morph_mask = false;
  bool show_raw_mask = false;
  bool show_filtered_mask = false;
  bool show_area_filter = false;
  bool show_area_prior_filter = false;
  bool show_area_prior_debug = false;
  bool show_aspect_filter = false;
  bool show_edge_filter = false;
  bool show_fill_filter = false;
  bool show_lut_filter = false;
  bool show_top_band_filter = false;
  bool show_search_debug = false;
  bool show_search_area_prior = false;
  bool show_overlay_image = false;
  bool show_roi_image = false;
  bool show_roi_mask = false;

  bool needThresholdMask() const
  {
    return show_threshold_mask;
  }

  bool needMorphMask() const
  {
    return show_morph_mask || show_roi_mask;
  }

  bool needRawMask() const
  {
    return publish_raw_mask || show_raw_mask;
  }

  bool needFilteredMask() const
  {
    return publish_filtered_mask || show_filtered_mask;
  }

  bool needOverlay() const
  {
    return publish_overlay || show_overlay_image;
  }

  bool needAnyWindows() const
  {
    return show_input_image || show_threshold_mask || show_morph_mask || show_raw_mask ||
           show_filtered_mask || show_area_filter || show_area_prior_filter ||
           show_area_prior_debug || show_aspect_filter || show_edge_filter ||
           show_fill_filter || show_lut_filter || show_top_band_filter || show_search_debug ||
           show_search_area_prior || show_overlay_image || show_roi_image || show_roi_mask;
  }

  bool needAnyOutputs() const
  {
    return needThresholdMask() || needMorphMask() || needRawMask() || needFilteredMask() ||
           show_area_filter || show_area_prior_filter || show_area_prior_debug ||
           show_aspect_filter || show_edge_filter || show_fill_filter || show_lut_filter ||
           show_top_band_filter || show_search_debug || show_search_area_prior ||
           needOverlay() || show_roi_image || show_roi_mask || show_input_image;
  }
};

}  // namespace

class OrangeBallDetectorNode : public rclcpp::Node
{
public:
  OrangeBallDetectorNode()
  : Node("orange_ball_detector_node")
  {
    declareParameters();
    loadParameters();
    loadRobotMask();
    loadLutFile();
    syncImageViewState();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OrangeBallDetectorNode::imageCallback, this, std::placeholders::_1));

    ball_center_ground_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/ball_center_ground", 10);
    ball_center_raw_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "~/ball_center_raw_px", 10);
    ball_top_raw_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "~/ball_top_raw_px", 10);
    detected_pub_ = create_publisher<std_msgs::msg::Bool>("~/detected", 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>("~/confidence", 10);
    debug_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_mask", 10);
    debug_mask_filtered_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/debug_mask_filtered", 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);
    if (publish_processing_time_) {
      processing_time_pub_ =
        create_publisher<std_msgs::msg::Float32>(processing_time_topic_, 10);
    }

    RCLCPP_INFO(
      get_logger(),
      "orange_ball_detector_node started. input_topic=%s, lut_file=%s, search_scale=%.2f, "
      "roi_scale=%.2f, lost_frame_tolerance=%d, ema_alpha=%.2f, force_search_mode=%s, "
      "robot_mask_path=%s, image_view=%s, timing_log=%s",
      input_topic_.c_str(),
      lut_file_.c_str(),
      search_downsample_scale_,
      roi_scale_,
      lost_frame_tolerance_,
      ema_alpha_,
      force_search_mode_ ? "true" : "false",
      robot_mask_enabled_ ? robot_mask_path_.c_str() : "<disabled>",
      enable_image_view_ ? "true" : "false",
      enable_timing_log_ ? "true" : "false");
  }

  ~OrangeBallDetectorNode() override
  {
    destroyDebugWindows();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("input_topic", "/camera/image_raw");
    declare_parameter<std::string>("lut_file", "");
    declare_parameter<std::string>(
      "robot_mask_path", packageConfigPath("mask.png"));
    declare_parameter("orange_h_min", 5);
    declare_parameter("orange_h_max", 30);
    declare_parameter("orange_s_min", 100);
    declare_parameter("orange_v_min", 60);
    declare_parameter("enable_morph_open", true);
    declare_parameter("morph_kernel_size", 3);
    declare_parameter("search_downsample_scale", 0.5);
    declare_parameter("min_blob_area_px", 20);
    declare_parameter("max_blob_area_px", 40000);
    declare_parameter("enable_distance_aware_area_prior", true);
    declare_parameter("area_prior_near_min_ratio", 0.75);
    declare_parameter("area_prior_near_max_ratio", 1.25);
    declare_parameter("area_prior_far_min_ratio", 0.50);
    declare_parameter("area_prior_far_max_ratio", 2.00);
    declare_parameter("min_aspect_ratio", 0.35);
    declare_parameter("max_aspect_ratio", 2.8);
    declare_parameter("max_edge_touch_ratio", 0.35);
    declare_parameter("top_band_px", 4);
    declare_parameter("roi_scale", 2.0);
    declare_parameter("roi_min_half_size_px", 50);
    declare_parameter("roi_max_half_size_px", 140);
    declare_parameter("lost_frame_tolerance", 3);
    declare_parameter("ema_alpha", 0.6);
    declare_parameter("force_search_mode", false);
    declare_parameter("enable_timing_log", true);
    declare_parameter("timing_log_interval", 30);
    declare_parameter("publish_processing_time", true);
    declare_parameter<std::string>("processing_time_topic", "~/processing_time_ms");
    declare_parameter("enable_image_view", false);
    declare_parameter("show_input_image", true);
    declare_parameter("show_threshold_mask", false);
    declare_parameter("show_morph_mask", false);
    declare_parameter("show_raw_mask", true);
    declare_parameter("show_filtered_mask", true);
    declare_parameter("show_area_filter", false);
    declare_parameter("show_area_prior_filter", false);
    declare_parameter("show_area_prior_debug", false);
    declare_parameter("show_aspect_filter", false);
    declare_parameter("show_edge_filter", false);
    declare_parameter("show_fill_filter", false);
    declare_parameter("show_lut_filter", false);
    declare_parameter("show_top_band_filter", false);
    declare_parameter("show_search_debug", false);
    declare_parameter("show_search_area_prior", false);
    declare_parameter("show_overlay_image", true);
    declare_parameter("show_roi_image", true);
    declare_parameter("show_roi_mask", false);
    declare_parameter("display_max_width", 960);
    declare_parameter("display_max_height", 720);
  }

  void loadParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    lut_file_ = get_parameter("lut_file").as_string();
    robot_mask_path_ = get_parameter("robot_mask_path").as_string();
    orange_h_min_ = clampHue(static_cast<int>(get_parameter("orange_h_min").as_int()));
    orange_h_max_ = clampHue(static_cast<int>(get_parameter("orange_h_max").as_int()));
    orange_s_min_ = clampToByte(static_cast<int>(get_parameter("orange_s_min").as_int()));
    orange_v_min_ = clampToByte(static_cast<int>(get_parameter("orange_v_min").as_int()));
    enable_morph_open_ = get_parameter("enable_morph_open").as_bool();
    morph_kernel_size_ =
      std::max(1, static_cast<int>(get_parameter("morph_kernel_size").as_int()));
    search_downsample_scale_ = get_parameter("search_downsample_scale").as_double();
    min_blob_area_px_ =
      std::max(1, static_cast<int>(get_parameter("min_blob_area_px").as_int()));
    max_blob_area_px_ =
      std::max(min_blob_area_px_, static_cast<int>(get_parameter("max_blob_area_px").as_int()));
    enable_distance_aware_area_prior_ =
      get_parameter("enable_distance_aware_area_prior").as_bool();
    area_prior_near_min_ratio_ = get_parameter("area_prior_near_min_ratio").as_double();
    area_prior_near_max_ratio_ = get_parameter("area_prior_near_max_ratio").as_double();
    area_prior_far_min_ratio_ = get_parameter("area_prior_far_min_ratio").as_double();
    area_prior_far_max_ratio_ = get_parameter("area_prior_far_max_ratio").as_double();
    min_aspect_ratio_ = get_parameter("min_aspect_ratio").as_double();
    max_aspect_ratio_ = get_parameter("max_aspect_ratio").as_double();
    max_edge_touch_ratio_ = get_parameter("max_edge_touch_ratio").as_double();
    top_band_px_ = std::max(1, static_cast<int>(get_parameter("top_band_px").as_int()));
    roi_scale_ = get_parameter("roi_scale").as_double();
    roi_min_half_size_px_ =
      std::max(1, static_cast<int>(get_parameter("roi_min_half_size_px").as_int()));
    roi_max_half_size_px_ =
      std::max(
      roi_min_half_size_px_,
      static_cast<int>(get_parameter("roi_max_half_size_px").as_int()));
    lost_frame_tolerance_ =
      std::max(1, static_cast<int>(get_parameter("lost_frame_tolerance").as_int()));
    ema_alpha_ = clamp01(get_parameter("ema_alpha").as_double());
    force_search_mode_ = get_parameter("force_search_mode").as_bool();
    enable_timing_log_ = get_parameter("enable_timing_log").as_bool();
    timing_log_interval_ =
      std::max(1, static_cast<int>(get_parameter("timing_log_interval").as_int()));
    publish_processing_time_ = get_parameter("publish_processing_time").as_bool();
    processing_time_topic_ = get_parameter("processing_time_topic").as_string();
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
    show_input_image_ = get_parameter("show_input_image").as_bool();
    show_threshold_mask_ = get_parameter("show_threshold_mask").as_bool();
    show_morph_mask_ = get_parameter("show_morph_mask").as_bool();
    show_raw_mask_ = get_parameter("show_raw_mask").as_bool();
    show_filtered_mask_ = get_parameter("show_filtered_mask").as_bool();
    show_area_filter_ = get_parameter("show_area_filter").as_bool();
    show_area_prior_filter_ = get_parameter("show_area_prior_filter").as_bool();
    show_area_prior_debug_ = get_parameter("show_area_prior_debug").as_bool();
    show_aspect_filter_ = get_parameter("show_aspect_filter").as_bool();
    show_edge_filter_ = get_parameter("show_edge_filter").as_bool();
    show_fill_filter_ = get_parameter("show_fill_filter").as_bool();
    show_lut_filter_ = get_parameter("show_lut_filter").as_bool();
    show_top_band_filter_ = get_parameter("show_top_band_filter").as_bool();
    show_search_debug_ = get_parameter("show_search_debug").as_bool();
    show_search_area_prior_ = get_parameter("show_search_area_prior").as_bool();
    show_overlay_image_ = get_parameter("show_overlay_image").as_bool();
    show_roi_image_ = get_parameter("show_roi_image").as_bool();
    show_roi_mask_ = get_parameter("show_roi_mask").as_bool();
    display_max_width_ =
      std::max(1, static_cast<int>(get_parameter("display_max_width").as_int()));
    display_max_height_ =
      std::max(1, static_cast<int>(get_parameter("display_max_height").as_int()));

    if (search_downsample_scale_ <= 0.0 || search_downsample_scale_ > 1.0) {
      throw std::runtime_error(
              "Parameter 'search_downsample_scale' must be in (0, 1].");
    }
    if (min_aspect_ratio_ <= 0.0 || max_aspect_ratio_ < min_aspect_ratio_) {
      throw std::runtime_error(
              "Invalid aspect ratio bounds; require 0 < min_aspect_ratio <= max_aspect_ratio.");
    }
    if (roi_scale_ < 1.0) {
      throw std::runtime_error("Parameter 'roi_scale' must be at least 1.0.");
    }
    if (enable_distance_aware_area_prior_) {
      if (
        !std::isfinite(area_prior_near_min_ratio_) || !std::isfinite(area_prior_near_max_ratio_) ||
        !std::isfinite(area_prior_far_min_ratio_) || !std::isfinite(area_prior_far_max_ratio_))
      {
        throw std::runtime_error("Area-prior ratio parameters must be finite.");
      }
      if (
        area_prior_near_min_ratio_ <= 0.0 || area_prior_near_max_ratio_ <= 0.0 ||
        area_prior_far_min_ratio_ <= 0.0 || area_prior_far_max_ratio_ <= 0.0)
      {
        throw std::runtime_error("Area-prior ratio parameters must be positive.");
      }
      if (area_prior_near_max_ratio_ < area_prior_near_min_ratio_) {
        throw std::runtime_error(
                "Parameter 'area_prior_near_max_ratio' must be >= 'area_prior_near_min_ratio'.");
      }
      if (area_prior_far_max_ratio_ < area_prior_far_min_ratio_) {
        throw std::runtime_error(
                "Parameter 'area_prior_far_max_ratio' must be >= 'area_prior_far_min_ratio'.");
      }
    }
    if (processing_time_topic_.empty()) {
      throw std::runtime_error("Parameter 'processing_time_topic' must not be empty.");
    }
    if (lut_file_.empty()) {
      throw std::runtime_error("Parameter 'lut_file' must not be empty.");
    }
  }

  void loadRobotMask()
  {
    robot_mask_enabled_ = false;
    robot_mask_validated_ = false;
    robot_allowed_mask_.release();
    resized_robot_allowed_mask_.release();
    resized_robot_allowed_mask_size_ = cv::Size();

    if (robot_mask_path_.empty()) {
      return;
    }

    const std::filesystem::path resolved_path = resolvePath(robot_mask_path_);
    const cv::Mat loaded_mask =
      cv::imread(resolved_path.string(), cv::IMREAD_GRAYSCALE);
    if (loaded_mask.empty()) {
      throw std::runtime_error("Cannot open robot mask image: " + resolved_path.string());
    }

    cv::compare(loaded_mask, 0, robot_allowed_mask_, cv::CMP_GT);
    robot_mask_enabled_ = true;
  }

  bool validateRobotMaskForFrame(const cv::Mat & frame)
  {
    if (!robot_mask_enabled_ || robot_mask_validated_) {
      return true;
    }

    if (robot_allowed_mask_.size() != frame.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "Robot mask size %dx%d does not match raw frame size %dx%d. Shutting down.",
        robot_allowed_mask_.cols,
        robot_allowed_mask_.rows,
        frame.cols,
        frame.rows);
      image_sub_.reset();
      rclcpp::shutdown();
      return false;
    }

    robot_mask_validated_ = true;
    return true;
  }

  const cv::Mat & getRobotMaskForSize(const cv::Size & target_size)
  {
    if (!robot_mask_enabled_ || target_size == robot_allowed_mask_.size()) {
      return robot_allowed_mask_;
    }

    if (resized_robot_allowed_mask_.empty() || resized_robot_allowed_mask_size_ != target_size) {
      cv::resize(
        robot_allowed_mask_,
        resized_robot_allowed_mask_,
        target_size,
        0.0,
        0.0,
        cv::INTER_NEAREST);
      resized_robot_allowed_mask_size_ = target_size;
    }

    return resized_robot_allowed_mask_;
  }

  void applyRobotMask(cv::Mat & mask, const cv::Rect * roi = nullptr)
  {
    if (!robot_mask_enabled_ || mask.empty()) {
      return;
    }

    if (roi != nullptr) {
      cv::bitwise_and(mask, robot_allowed_mask_(*roi), mask);
      return;
    }

    cv::bitwise_and(mask, getRobotMaskForSize(mask.size()), mask);
  }

  void loadLutFile()
  {
    cv::FileStorage fs(lut_file_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      throw std::runtime_error("Cannot open LUT file: " + lut_file_);
    }

    fs["ground_x_m"] >> lut_.ground_x_m;
    fs["ground_y_m"] >> lut_.ground_y_m;
    fs["valid_mask"] >> lut_.valid_mask;
    fs["area_prior_distance_m"] >> lut_.area_prior_distance_m;
    fs["area_prior_expected_area_px"] >> lut_.area_prior_expected_area_px;
    fs["source_width"] >> lut_.source_width;
    fs["source_height"] >> lut_.source_height;
    fs["ocam_xc"] >> lut_.ocam_xc;
    fs["ocam_yc"] >> lut_.ocam_yc;
    fs["camera_height_m"] >> lut_.camera_height_m;
    fs["ball_diameter_m"] >> lut_.ball_diameter_m;
    fs.release();

    if (
      lut_.ground_x_m.empty() || lut_.ground_y_m.empty() || lut_.valid_mask.empty() ||
      lut_.ground_x_m.size() != lut_.ground_y_m.size() ||
      lut_.ground_x_m.size() != lut_.valid_mask.size())
    {
      throw std::runtime_error("LUT file is missing required matrices: " + lut_file_);
    }
    if (
      enable_distance_aware_area_prior_ &&
      (lut_.area_prior_distance_m.empty() || lut_.area_prior_expected_area_px.empty()))
    {
      throw std::runtime_error(
              "LUT file is missing the distance-aware area prior fields "
              "(area_prior_distance_m / area_prior_expected_area_px). "
              "Regenerate the raw ball LUT before launching: " + lut_file_);
    }

    if (lut_.source_width <= 0 || lut_.source_height <= 0) {
      throw std::runtime_error("LUT file has invalid source dimensions: " + lut_file_);
    }

    if (lut_.ground_x_m.type() != CV_32FC1) {
      lut_.ground_x_m.convertTo(lut_.ground_x_m, CV_32FC1);
    }
    if (lut_.ground_y_m.type() != CV_32FC1) {
      lut_.ground_y_m.convertTo(lut_.ground_y_m, CV_32FC1);
    }
    if (lut_.valid_mask.type() != CV_8UC1) {
      cv::Mat converted;
      lut_.valid_mask.convertTo(converted, CV_8UC1);
      lut_.valid_mask = converted;
    }
    lut_.area_prior_distance_samples_m.clear();
    lut_.area_prior_expected_area_samples_px.clear();
    if (enable_distance_aware_area_prior_) {
      if (lut_.area_prior_distance_m.type() != CV_32FC1) {
        lut_.area_prior_distance_m.convertTo(lut_.area_prior_distance_m, CV_32FC1);
      }
      if (lut_.area_prior_expected_area_px.type() != CV_32FC1) {
        lut_.area_prior_expected_area_px.convertTo(lut_.area_prior_expected_area_px, CV_32FC1);
      }

      if (
        (lut_.area_prior_distance_m.rows != 1 && lut_.area_prior_distance_m.cols != 1) ||
        (lut_.area_prior_expected_area_px.rows != 1 && lut_.area_prior_expected_area_px.cols != 1))
      {
        throw std::runtime_error(
                "Area prior LUT entries must be stored as 1D vectors in: " + lut_file_);
      }

      const std::size_t prior_count = lut_.area_prior_distance_m.total();
      if (prior_count != lut_.area_prior_expected_area_px.total() || prior_count < 2U) {
        throw std::runtime_error(
                "Area prior LUT vectors must be non-empty, have matching lengths, and contain at "
                "least 2 samples: " + lut_file_);
      }

      const cv::Mat distance_flat = lut_.area_prior_distance_m.reshape(1, 1);
      const cv::Mat expected_area_flat = lut_.area_prior_expected_area_px.reshape(1, 1);
      lut_.area_prior_distance_samples_m.reserve(prior_count);
      lut_.area_prior_expected_area_samples_px.reserve(prior_count);
      for (std::size_t i = 0; i < prior_count; ++i) {
        const double distance_m =
          static_cast<double>(distance_flat.at<float>(0, static_cast<int>(i)));
        const double expected_area_px =
          static_cast<double>(expected_area_flat.at<float>(0, static_cast<int>(i)));
        if (
          !std::isfinite(distance_m) || !std::isfinite(expected_area_px) ||
          expected_area_px <= 0.0)
        {
          throw std::runtime_error(
                  "Area prior LUT contains non-finite distance/area samples: " + lut_file_);
        }
        if (i > 0U) {
          if (distance_m <= lut_.area_prior_distance_samples_m.back()) {
            throw std::runtime_error(
                    "area_prior_distance_m must be strictly increasing in LUT file: " +
                    lut_file_);
          }
          if (expected_area_px > lut_.area_prior_expected_area_samples_px.back() + 1e-3) {
            throw std::runtime_error(
                    "area_prior_expected_area_px must be monotonic non-increasing in LUT file: " +
                    lut_file_);
          }
        }
        lut_.area_prior_distance_samples_m.push_back(distance_m);
        lut_.area_prior_expected_area_samples_px.push_back(expected_area_px);
      }
    }
  }

  cv::Point mapSearchPointToRaw(const cv::Point & point_small) const
  {
    const double raw_x =
      ((static_cast<double>(point_small.x) + 0.5) / search_downsample_scale_) - 0.5;
    const double raw_y =
      ((static_cast<double>(point_small.y) + 0.5) / search_downsample_scale_) - 0.5;
    return clampPointToImage(
      cv::Point(
        static_cast<int>(std::lround(raw_x)),
        static_cast<int>(std::lround(raw_y))),
      cv::Size(lut_.source_width, lut_.source_height));
  }

  bool lookupGroundDistanceAtPixel(const cv::Point & raw_pixel, double & distance_m) const
  {
    if (
      raw_pixel.x < 0 || raw_pixel.x >= lut_.source_width ||
      raw_pixel.y < 0 || raw_pixel.y >= lut_.source_height)
    {
      return false;
    }
    if (lut_.valid_mask.at<uchar>(raw_pixel) == 0) {
      return false;
    }

    const double ground_x_m = static_cast<double>(lut_.ground_x_m.at<float>(raw_pixel));
    const double ground_y_m = static_cast<double>(lut_.ground_y_m.at<float>(raw_pixel));
    if (!std::isfinite(ground_x_m) || !std::isfinite(ground_y_m)) {
      return false;
    }

    distance_m = std::hypot(ground_x_m, ground_y_m);
    return std::isfinite(distance_m);
  }

  double interpolateAreaPriorExpectedArea(double distance_m) const
  {
    const auto & distances = lut_.area_prior_distance_samples_m;
    const auto & expected_areas = lut_.area_prior_expected_area_samples_px;
    if (distances.empty() || expected_areas.empty()) {
      throw std::runtime_error("Area prior LUT samples are unavailable at runtime.");
    }

    if (distance_m <= distances.front()) {
      return expected_areas.front();
    }
    if (distance_m >= distances.back()) {
      return expected_areas.back();
    }

    const auto upper_it = std::lower_bound(distances.begin(), distances.end(), distance_m);
    const std::size_t upper_index = static_cast<std::size_t>(upper_it - distances.begin());
    const std::size_t lower_index = upper_index - 1U;
    const double lower_distance = distances[lower_index];
    const double upper_distance = distances[upper_index];
    const double lower_area = expected_areas[lower_index];
    const double upper_area = expected_areas[upper_index];
    const double span = upper_distance - lower_distance;
    const double t = span > 1e-9 ? (distance_m - lower_distance) / span : 0.0;
    return lower_area + (upper_area - lower_area) * t;
  }

  double interpolateAreaPriorRatio(
    double distance_m,
    double near_ratio,
    double far_ratio) const
  {
    const auto & distances = lut_.area_prior_distance_samples_m;
    if (distances.size() < 2U) {
      return near_ratio;
    }

    const double min_distance = distances.front();
    const double max_distance = distances.back();
    const double span = max_distance - min_distance;
    if (span <= 1e-9) {
      return near_ratio;
    }

    const double t = clamp01((distance_m - min_distance) / span);
    return near_ratio + (far_ratio - near_ratio) * t;
  }

  AreaPriorEvaluation evaluateAreaPrior(
    double distance_m,
    double actual_area_px,
    double area_scale = 1.0) const
  {
    AreaPriorEvaluation evaluation;
    evaluation.distance_m = distance_m;
    if (!std::isfinite(distance_m) || !std::isfinite(actual_area_px) || actual_area_px <= 0.0) {
      return evaluation;
    }

    const double expected_area_px =
      interpolateAreaPriorExpectedArea(distance_m) * std::max(area_scale, 1e-9);
    if (!std::isfinite(expected_area_px) || expected_area_px <= 0.0) {
      return evaluation;
    }

    const double min_ratio = interpolateAreaPriorRatio(
      distance_m,
      area_prior_near_min_ratio_,
      area_prior_far_min_ratio_);
    const double max_ratio = interpolateAreaPriorRatio(
      distance_m,
      area_prior_near_max_ratio_,
      area_prior_far_max_ratio_);
    if (
      !std::isfinite(min_ratio) || !std::isfinite(max_ratio) || min_ratio <= 0.0 ||
      max_ratio <= 0.0)
    {
      return evaluation;
    }

    const double log_error = std::abs(std::log(actual_area_px / expected_area_px));
    const double log_width =
      std::max(std::abs(std::log(min_ratio)), std::abs(std::log(max_ratio)));

    evaluation.valid = true;
    evaluation.expected_area_px = expected_area_px;
    evaluation.min_ratio = min_ratio;
    evaluation.max_ratio = max_ratio;
    evaluation.min_area_px = expected_area_px * min_ratio;
    evaluation.max_area_px = expected_area_px * max_ratio;
    evaluation.passed =
      actual_area_px >= evaluation.min_area_px && actual_area_px <= evaluation.max_area_px;
    evaluation.area_score = clamp01(1.0 - (log_error / std::max(log_width, 1e-6)));
    return evaluation;
  }

  std::vector<std::string> formatAreaPriorDebugLines(
    double actual_area_px,
    const AreaPriorEvaluation & evaluation) const
  {
    std::vector<std::string> lines;
    if (!enable_distance_aware_area_prior_) {
      lines.emplace_back("area prior disabled");
      return lines;
    }

    if (!evaluation.valid) {
      std::ostringstream oss;
      oss << "A=" << std::fixed << std::setprecision(0) << actual_area_px;
      lines.push_back(oss.str());
      lines.emplace_back("invalid");
      return lines;
    }

    std::ostringstream area_line;
    area_line << "A=" << std::fixed << std::setprecision(0) << actual_area_px;
    lines.push_back(area_line.str());

    std::ostringstream expected_line;
    expected_line << "E=" << std::fixed << std::setprecision(0) << evaluation.expected_area_px;
    lines.push_back(expected_line.str());

    std::ostringstream range_line;
    range_line << "R=[" << std::fixed << std::setprecision(0) << evaluation.min_area_px
               << ", " << evaluation.max_area_px << "]";
    lines.push_back(range_line.str());

    std::ostringstream score_line;
    score_line << "S=" << std::fixed << std::setprecision(2) << evaluation.area_score
               << " D=" << std::setprecision(2) << evaluation.distance_m;
    lines.push_back(score_line.str());
    return lines;
  }

  void drawMultilineText(
    cv::Mat & image,
    const std::vector<std::string> & lines,
    const cv::Point & origin,
    double font_scale,
    const cv::Scalar & color,
    int thickness) const
  {
    int baseline = 0;
    const cv::Size sample_size = cv::getTextSize(
      "Ag",
      cv::FONT_HERSHEY_SIMPLEX,
      font_scale,
      thickness,
      &baseline);
    const int line_step = std::max(sample_size.height + baseline + 2, 12);
    int y = origin.y;
    for (const std::string & line : lines) {
      cv::putText(
        image,
        line,
        cv::Point(origin.x, y),
        cv::FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
        cv::LINE_AA);
      y += line_step;
    }
  }

  void drawAreaPriorAnnotation(
    cv::Mat & image,
    const cv::Rect & bbox,
    double actual_area_px,
    const AreaPriorEvaluation & evaluation,
    const cv::Scalar & color,
    double font_scale = 0.45,
    int thickness = 1) const
  {
    if (image.empty()) {
      return;
    }

    cv::rectangle(image, bbox, color, 2);
    const std::vector<std::string> lines =
      formatAreaPriorDebugLines(actual_area_px, evaluation);
    const cv::Point text_origin(
      std::max(4, bbox.x),
      std::max(18, bbox.y - 6));
    drawMultilineText(image, lines, text_origin, font_scale, color, thickness);
  }

  void syncWindow(const std::string & window_name, bool should_show, bool & created)
  {
    if (should_show && !created) {
      cv::namedWindow(window_name, cv::WINDOW_NORMAL);
      created = true;
    } else if (!should_show && created) {
      cv::destroyWindow(window_name);
      created = false;
    }
  }

  void syncImageViewState()
  {
    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (enable_image_view_ && !display_available) {
      RCLCPP_WARN(
        get_logger(),
        "enable_image_view=true but no DISPLAY/WAYLAND_DISPLAY is available; "
        "disabling GUI windows.");
      enable_image_view_ = false;
    }

    const bool master_enabled = enable_image_view_;
    syncWindow(kInputWindowName, master_enabled && show_input_image_, input_window_created_);
    syncWindow(
      kThresholdMaskWindowName,
      master_enabled && show_threshold_mask_,
      threshold_mask_window_created_);
    syncWindow(
      kMorphMaskWindowName,
      master_enabled && show_morph_mask_,
      morph_mask_window_created_);
    syncWindow(kRawMaskWindowName, master_enabled && show_raw_mask_, raw_mask_window_created_);
    syncWindow(
      kFilteredMaskWindowName,
      master_enabled && show_filtered_mask_,
      filtered_mask_window_created_);
    syncWindow(
      kAreaFilterWindowName,
      master_enabled && show_area_filter_,
      area_filter_window_created_);
    syncWindow(
      kAreaPriorFilterWindowName,
      master_enabled && show_area_prior_filter_,
      area_prior_filter_window_created_);
    syncWindow(
      kAreaPriorDebugWindowName,
      master_enabled && show_area_prior_debug_,
      area_prior_debug_window_created_);
    syncWindow(
      kAspectFilterWindowName,
      master_enabled && show_aspect_filter_,
      aspect_filter_window_created_);
    syncWindow(
      kEdgeFilterWindowName,
      master_enabled && show_edge_filter_,
      edge_filter_window_created_);
    syncWindow(
      kFillFilterWindowName,
      master_enabled && show_fill_filter_,
      fill_filter_window_created_);
    syncWindow(
      kLutFilterWindowName,
      master_enabled && show_lut_filter_,
      lut_filter_window_created_);
    syncWindow(
      kTopBandFilterWindowName,
      master_enabled && show_top_band_filter_,
      top_band_filter_window_created_);
    syncWindow(
      kSearchDebugWindowName,
      master_enabled && show_search_debug_,
      search_debug_window_created_);
    syncWindow(
      kSearchAreaPriorWindowName,
      master_enabled && show_search_area_prior_,
      search_area_prior_window_created_);
    syncWindow(
      kOverlayWindowName,
      master_enabled && show_overlay_image_,
      overlay_window_created_);
    syncWindow(kRoiWindowName, master_enabled && show_roi_image_, roi_window_created_);
    syncWindow(kRoiMaskWindowName, master_enabled && show_roi_mask_, roi_mask_window_created_);
  }

  void destroyDebugWindows()
  {
    syncWindow(kInputWindowName, false, input_window_created_);
    syncWindow(kThresholdMaskWindowName, false, threshold_mask_window_created_);
    syncWindow(kMorphMaskWindowName, false, morph_mask_window_created_);
    syncWindow(kOverlayWindowName, false, overlay_window_created_);
    syncWindow(kRawMaskWindowName, false, raw_mask_window_created_);
    syncWindow(kFilteredMaskWindowName, false, filtered_mask_window_created_);
    syncWindow(kAreaFilterWindowName, false, area_filter_window_created_);
    syncWindow(kAreaPriorFilterWindowName, false, area_prior_filter_window_created_);
    syncWindow(kAreaPriorDebugWindowName, false, area_prior_debug_window_created_);
    syncWindow(kAspectFilterWindowName, false, aspect_filter_window_created_);
    syncWindow(kEdgeFilterWindowName, false, edge_filter_window_created_);
    syncWindow(kFillFilterWindowName, false, fill_filter_window_created_);
    syncWindow(kLutFilterWindowName, false, lut_filter_window_created_);
    syncWindow(kTopBandFilterWindowName, false, top_band_filter_window_created_);
    syncWindow(kSearchDebugWindowName, false, search_debug_window_created_);
    syncWindow(kSearchAreaPriorWindowName, false, search_area_prior_window_created_);
    syncWindow(kRoiWindowName, false, roi_window_created_);
    syncWindow(kRoiMaskWindowName, false, roi_mask_window_created_);
  }

  DebugRequest buildDebugRequest() const
  {
    DebugRequest request;
    request.publish_raw_mask =
      debug_mask_pub_ && debug_mask_pub_->get_subscription_count() > 0U;
    request.publish_filtered_mask =
      debug_mask_filtered_pub_ && debug_mask_filtered_pub_->get_subscription_count() > 0U;
    request.publish_overlay =
      debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0U;
    request.show_input_image = input_window_created_;
    request.show_threshold_mask = threshold_mask_window_created_;
    request.show_morph_mask = morph_mask_window_created_;
    request.show_raw_mask = raw_mask_window_created_;
    request.show_filtered_mask = filtered_mask_window_created_;
    request.show_area_filter = area_filter_window_created_;
    request.show_area_prior_filter = area_prior_filter_window_created_;
    request.show_area_prior_debug = area_prior_debug_window_created_;
    request.show_aspect_filter = aspect_filter_window_created_;
    request.show_edge_filter = edge_filter_window_created_;
    request.show_fill_filter = fill_filter_window_created_;
    request.show_lut_filter = lut_filter_window_created_;
    request.show_top_band_filter = top_band_filter_window_created_;
    request.show_search_debug = search_debug_window_created_;
    request.show_search_area_prior = search_area_prior_window_created_;
    request.show_overlay_image = overlay_window_created_;
    request.show_roi_image = roi_window_created_;
    request.show_roi_mask = roi_mask_window_created_;
    return request;
  }

  void logTimingSummary(const std::array<long long, kTimingStageCount> & stage_us)
  {
    if (!enable_timing_log_) {
      return;
    }

    ++timing_frames_in_interval_;
    for (std::size_t i = 0; i < timing_interval_totals_us_.size(); ++i) {
      timing_interval_totals_us_[i] += stage_us[i];
    }

    if (timing_frames_in_interval_ < static_cast<std::size_t>(timing_log_interval_)) {
      return;
    }

    std::ostringstream oss;
    oss << "\n=========== Orange Ball Timing Summary ===========\n";
    oss << "frames: " << timing_frames_in_interval_ << '\n';
    oss << std::left << std::setw(22) << "stage"
        << std::right << std::setw(12) << "avg ms" << '\n';
    for (std::size_t i = 0; i < kTimingLabels.size(); ++i) {
      const double avg_ms =
        static_cast<double>(timing_interval_totals_us_[i]) /
        static_cast<double>(timing_frames_in_interval_) / 1000.0;
      oss << std::left << std::setw(22) << kTimingLabels[i]
          << std::right << std::setw(12) << std::fixed << std::setprecision(3) << avg_ms
          << '\n';
    }
    oss << "=================================================";
    RCLCPP_INFO_STREAM(get_logger(), oss.str());

    timing_frames_in_interval_ = 0;
    timing_interval_totals_us_.fill(0);
  }

  void buildThresholdMask(const cv::Mat & bgr, cv::Mat & mask, const cv::Rect * roi = nullptr)
  {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    if (orange_h_min_ <= orange_h_max_) {
      cv::inRange(
        hsv,
        cv::Scalar(orange_h_min_, orange_s_min_, orange_v_min_),
        cv::Scalar(orange_h_max_, kByteMax, kByteMax),
        mask);
    } else {
      cv::Mat low_mask;
      cv::Mat high_mask;
      cv::inRange(
        hsv,
        cv::Scalar(0, orange_s_min_, orange_v_min_),
        cv::Scalar(orange_h_max_, kByteMax, kByteMax),
        low_mask);
      cv::inRange(
        hsv,
        cv::Scalar(orange_h_min_, orange_s_min_, orange_v_min_),
        cv::Scalar(kHueMax, kByteMax, kByteMax),
        high_mask);
      cv::bitwise_or(low_mask, high_mask, mask);
    }

    applyRobotMask(mask, roi);
  }

  void applyMorphOpen(cv::Mat & mask) const
  {
    if (enable_morph_open_ && morph_kernel_size_ > 1) {
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(morph_kernel_size_, morph_kernel_size_));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
  }

  void buildOrangeMask(const cv::Mat & bgr, cv::Mat & mask, const cv::Rect * roi = nullptr)
  {
    buildThresholdMask(bgr, mask, roi);
    applyMorphOpen(mask);
  }

  std::vector<CoarseCandidate> findCoarseCandidates(
    const cv::Mat & frame_small,
    cv::Mat * search_area_prior_debug_image = nullptr)
  {
    std::vector<CoarseCandidate> coarse_candidates;
    cv::Mat mask;
    buildOrangeMask(frame_small, mask);
    if (search_area_prior_debug_image != nullptr) {
      *search_area_prior_debug_image = frame_small.clone();
      if (search_area_prior_debug_image->channels() == 1) {
        cv::cvtColor(
          *search_area_prior_debug_image,
          *search_area_prior_debug_image,
          cv::COLOR_GRAY2BGR);
      }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count =
      cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    std::vector<cv::Point> farthest_pixels(static_cast<std::size_t>(label_count));
    std::vector<double> farthest_radius_sq(
      static_cast<std::size_t>(label_count),
      -std::numeric_limits<double>::infinity());
    const double ocam_x_small = lut_.ocam_xc * search_downsample_scale_;
    const double ocam_y_small = lut_.ocam_yc * search_downsample_scale_;
    for (int y = 0; y < labels.rows; ++y) {
      const int * label_row = labels.ptr<int>(y);
      for (int x = 0; x < labels.cols; ++x) {
        const int label = label_row[x];
        if (label <= 0) {
          continue;
        }

        const double dx = static_cast<double>(x) - ocam_x_small;
        const double dy = static_cast<double>(y) - ocam_y_small;
        const double radius_sq = dx * dx + dy * dy;
        if (radius_sq > farthest_radius_sq[static_cast<std::size_t>(label)]) {
          farthest_radius_sq[static_cast<std::size_t>(label)] = radius_sq;
          farthest_pixels[static_cast<std::size_t>(label)] = cv::Point(x, y);
        }
      }
    }

    const double scale_sq = search_downsample_scale_ * search_downsample_scale_;
    const int scaled_min_area = std::max(1, static_cast<int>(std::round(min_blob_area_px_ * scale_sq)));
    const int scaled_max_area = std::max(
      scaled_min_area,
      static_cast<int>(std::round(max_blob_area_px_ * scale_sq)));

    for (int label = 1; label < label_count; ++label) {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int y = stats.at<int>(label, cv::CC_STAT_TOP);
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      if (area < scaled_min_area || area > scaled_max_area || width <= 0 || height <= 0) {
        continue;
      }

      const double aspect = static_cast<double>(width) / static_cast<double>(height);
      if (aspect < min_aspect_ratio_ || aspect > max_aspect_ratio_) {
        continue;
      }

      const cv::Rect bbox(x, y, width, height);
      const double edge_touch_ratio = computeEdgeTouchRatio(bbox, frame_small.size());
      if (edge_touch_ratio > max_edge_touch_ratio_) {
        continue;
      }

      const double fill_ratio =
        static_cast<double>(area) /
        std::max(1.0, static_cast<double>(width) * static_cast<double>(height));
      if (fill_ratio < kMinFillRatio) {
        continue;
      }

      const double aspect_score =
        clamp01(1.0 - (std::abs(std::log(aspect)) / std::log(std::max(1.001, max_aspect_ratio_))));
      const double fill_score = clamp01(fill_ratio / 0.85);
      double area_score =
        clamp01(static_cast<double>(area) / static_cast<double>(scaled_max_area));
      AreaPriorEvaluation area_prior;
      bool area_prior_passed = true;
      const cv::Point raw_reference_point =
        mapSearchPointToRaw(farthest_pixels[static_cast<std::size_t>(label)]);
      if (enable_distance_aware_area_prior_) {
        double distance_m = 0.0;
        if (!lookupGroundDistanceAtPixel(raw_reference_point, distance_m)) {
          continue;
        }

        area_prior = evaluateAreaPrior(
          distance_m,
          static_cast<double>(area),
          scale_sq);
        area_prior_passed = area_prior.valid && area_prior.passed;
        if (search_area_prior_debug_image != nullptr) {
          drawAreaPriorAnnotation(
            *search_area_prior_debug_image,
            bbox,
            static_cast<double>(area),
            area_prior,
            area_prior_passed ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
            0.30,
            1);
          cv::circle(
            *search_area_prior_debug_image,
            farthest_pixels[static_cast<std::size_t>(label)],
            3,
            cv::Scalar(255, 255, 0),
            -1);
        }
        if (!area_prior_passed) {
          continue;
        }
        area_score = area_prior.area_score;
      } else if (search_area_prior_debug_image != nullptr) {
        drawAreaPriorAnnotation(
          *search_area_prior_debug_image,
          bbox,
          static_cast<double>(area),
          area_prior,
          cv::Scalar(0, 255, 255),
          0.30,
          1);
      }
      const double score = 0.45 * aspect_score + 0.30 * fill_score + 0.25 * area_score;

      CoarseCandidate coarse_candidate;
      coarse_candidate.valid = true;
      coarse_candidate.bbox = bbox;
      coarse_candidate.area_px = static_cast<double>(area);
      coarse_candidate.area_prior = area_prior;
      coarse_candidate.raw_reference_point = raw_reference_point;
      coarse_candidate.score = score;
      coarse_candidates.push_back(coarse_candidate);
    }

    std::sort(
      coarse_candidates.begin(),
      coarse_candidates.end(),
      [](const CoarseCandidate & lhs, const CoarseCandidate & rhs) {
        return lhs.score > rhs.score;
      });
    return coarse_candidates;
  }

  DetectorOutputs detectCandidateInRoi(
    const cv::Mat & frame,
    const cv::Rect & roi,
    TrackingMode mode,
    const DetectionDebugOptions & debug_options)
  {
    DetectorOutputs outputs;
    outputs.roi = roi;

    if (roi.width <= 0 || roi.height <= 0) {
      return outputs;
    }

    const cv::Mat frame_roi = frame(roi);
    cv::Mat threshold_roi_mask;
    buildThresholdMask(frame_roi, threshold_roi_mask, &roi);
    if (debug_options.capture_threshold_mask || debug_options.capture_raw_mask) {
      outputs.threshold_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
      threshold_roi_mask.copyTo(outputs.threshold_debug_mask(roi));
    }

    cv::Mat mask_roi = threshold_roi_mask.clone();
    applyMorphOpen(mask_roi);
    if (debug_options.capture_morph_mask || debug_options.capture_roi_mask) {
      outputs.morph_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
      mask_roi.copyTo(outputs.morph_debug_mask(roi));
    }
    if (debug_options.capture_raw_mask) {
      if (outputs.morph_debug_mask.empty()) {
        outputs.raw_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        mask_roi.copyTo(outputs.raw_debug_mask(roi));
      } else {
        outputs.raw_debug_mask = outputs.morph_debug_mask;
      }
    }
    if (debug_options.capture_roi_mask) {
      outputs.roi_debug_mask = mask_roi.clone();
    }
    if (debug_options.capture_filtered_mask) {
      outputs.filtered_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    }
    if (debug_options.capture_area_prior_filter_mask) {
      outputs.area_prior_filter_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    }
    if (debug_options.capture_area_prior_debug) {
      outputs.area_prior_debug_image = frame.clone();
      cv::rectangle(outputs.area_prior_debug_image, roi, cv::Scalar(255, 255, 0), 1);
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count =
      cv::connectedComponentsWithStats(mask_roi, labels, stats, centroids, 8, CV_32S);

    if (label_count <= 1) {
      return outputs;
    }

    std::vector<std::vector<cv::Point>> label_pixels(static_cast<std::size_t>(label_count));
    for (int y = 0; y < labels.rows; ++y) {
      const int * label_row = labels.ptr<int>(y);
      for (int x = 0; x < labels.cols; ++x) {
        const int label = label_row[x];
        if (label <= 0) {
          continue;
        }
        label_pixels[static_cast<std::size_t>(label)].emplace_back(x + roi.x, y + roi.y);
      }
    }

    for (int label = 1; label < label_count; ++label) {
      Candidate candidate;
      candidate.area_px = static_cast<double>(stats.at<int>(label, cv::CC_STAT_AREA));
      const std::vector<cv::Point> & component_pixels =
        label_pixels[static_cast<std::size_t>(label)];
      candidate.bbox = cv::Rect(
        stats.at<int>(label, cv::CC_STAT_LEFT) + roi.x,
        stats.at<int>(label, cv::CC_STAT_TOP) + roi.y,
        stats.at<int>(label, cv::CC_STAT_WIDTH),
        stats.at<int>(label, cv::CC_STAT_HEIGHT));

      if (
        candidate.area_px < static_cast<double>(min_blob_area_px_) ||
        candidate.area_px > static_cast<double>(max_blob_area_px_))
      {
        continue;
      }
      if (debug_options.capture_area_mask) {
        if (outputs.area_debug_mask.empty()) {
          outputs.area_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.area_debug_mask.at<uchar>(pixel) = 255;
        }
      }
      if (candidate.bbox.width <= 0 || candidate.bbox.height <= 0) {
        continue;
      }

      candidate.aspect_ratio =
        static_cast<double>(candidate.bbox.width) / static_cast<double>(candidate.bbox.height);
      if (
        candidate.aspect_ratio < min_aspect_ratio_ ||
        candidate.aspect_ratio > max_aspect_ratio_)
      {
        continue;
      }
      if (debug_options.capture_aspect_mask) {
        if (outputs.aspect_debug_mask.empty()) {
          outputs.aspect_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.aspect_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      candidate.edge_touch_ratio = computeEdgeTouchRatio(candidate.bbox, frame.size());
      if (candidate.edge_touch_ratio > max_edge_touch_ratio_) {
        continue;
      }
      if (debug_options.capture_edge_mask) {
        if (outputs.edge_debug_mask.empty()) {
          outputs.edge_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.edge_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      candidate.fill_ratio =
        candidate.area_px /
        std::max(
        1.0,
        static_cast<double>(candidate.bbox.width) * static_cast<double>(candidate.bbox.height));
      if (candidate.fill_ratio < kMinFillRatio) {
        continue;
      }
      if (debug_options.capture_fill_mask) {
        if (outputs.fill_debug_mask.empty()) {
          outputs.fill_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.fill_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      candidate.pixels = component_pixels;
      if (candidate.pixels.empty()) {
        continue;
      }

      const double centroid_x = centroids.at<double>(label, 0) + static_cast<double>(roi.x);
      const double centroid_y = centroids.at<double>(label, 1) + static_cast<double>(roi.y);
      candidate.raw_center_px = cv::Point2d(centroid_x, centroid_y);

      int valid_lut_pixels = 0;
      double max_radius = -std::numeric_limits<double>::infinity();
      std::vector<double> radii(candidate.pixels.size(), 0.0);
      for (std::size_t i = 0; i < candidate.pixels.size(); ++i) {
        const cv::Point & pixel = candidate.pixels[i];
        const double dx = static_cast<double>(pixel.x) - lut_.ocam_xc;
        const double dy = static_cast<double>(pixel.y) - lut_.ocam_yc;
        const double radius = std::hypot(dx, dy);
        radii[i] = radius;
        max_radius = std::max(max_radius, radius);
        if (lut_.valid_mask.at<uchar>(pixel) > 0) {
          ++valid_lut_pixels;
        }
      }
      candidate.valid_lut_ratio =
        static_cast<double>(valid_lut_pixels) / static_cast<double>(candidate.pixels.size());
      if (candidate.valid_lut_ratio < kMinValidLutCoverageRatio) {
        continue;
      }
      if (debug_options.capture_lut_mask) {
        if (outputs.lut_debug_mask.empty()) {
          outputs.lut_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.lut_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      const double band_threshold = max_radius - static_cast<double>(top_band_px_);
      double top_weight_sum = 0.0;
      double top_x_sum = 0.0;
      double top_y_sum = 0.0;
      double ground_x_sum = 0.0;
      double ground_y_sum = 0.0;
      double ground_weight_sum = 0.0;
      int valid_top_band_pixels = 0;
      for (std::size_t i = 0; i < candidate.pixels.size(); ++i) {
        if (radii[i] < band_threshold) {
          continue;
        }

        const cv::Point & pixel = candidate.pixels[i];
        const double weight = 1.0 + std::max(0.0, radii[i] - band_threshold);
        candidate.top_band_pixels.push_back(pixel);
        top_weight_sum += weight;
        top_x_sum += static_cast<double>(pixel.x) * weight;
        top_y_sum += static_cast<double>(pixel.y) * weight;
        if (lut_.valid_mask.at<uchar>(pixel) == 0) {
          continue;
        }

        ground_x_sum += static_cast<double>(lut_.ground_x_m.at<float>(pixel)) * weight;
        ground_y_sum += static_cast<double>(lut_.ground_y_m.at<float>(pixel)) * weight;
        ground_weight_sum += weight;
        ++valid_top_band_pixels;
      }

      if (candidate.top_band_pixels.empty() || top_weight_sum <= 0.0 || ground_weight_sum <= 0.0) {
        continue;
      }

      candidate.top_band_valid_ratio =
        static_cast<double>(valid_top_band_pixels) /
        static_cast<double>(candidate.top_band_pixels.size());
      if (candidate.top_band_valid_ratio < kMinTopBandValidRatio) {
        continue;
      }
      if (debug_options.capture_top_band_mask) {
        if (outputs.top_band_debug_mask.empty()) {
          outputs.top_band_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        }
        for (const cv::Point & pixel : component_pixels) {
          outputs.top_band_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      candidate.top_point_px = cv::Point2d(top_x_sum / top_weight_sum, top_y_sum / top_weight_sum);
      candidate.ground_center_m = cv::Point2d(
        ground_x_sum / ground_weight_sum,
        ground_y_sum / ground_weight_sum);

      const double aspect_score =
        clamp01(
        1.0 - (std::abs(std::log(candidate.aspect_ratio)) /
        std::log(std::max(1.001, max_aspect_ratio_))));
      const double fill_score = clamp01(candidate.fill_ratio / 0.85);
      const double valid_score = clamp01(candidate.top_band_valid_ratio);
      double area_score =
        clamp01(candidate.area_px / static_cast<double>(max_blob_area_px_));
      if (enable_distance_aware_area_prior_) {
        const double distance_m =
          std::hypot(candidate.ground_center_m.x, candidate.ground_center_m.y);
        candidate.area_prior = evaluateAreaPrior(distance_m, candidate.area_px);
        if (debug_options.capture_area_prior_debug) {
          drawAreaPriorAnnotation(
            outputs.area_prior_debug_image,
            candidate.bbox,
            candidate.area_px,
            candidate.area_prior,
            (candidate.area_prior.valid && candidate.area_prior.passed) ?
            cv::Scalar(0, 255, 0) :
            cv::Scalar(0, 0, 255));
        }
        if (!candidate.area_prior.valid || !candidate.area_prior.passed) {
          continue;
        }
        if (debug_options.capture_area_prior_filter_mask) {
          for (const cv::Point & pixel : component_pixels) {
            outputs.area_prior_filter_debug_mask.at<uchar>(pixel) = 255;
          }
        }
        area_score = candidate.area_prior.area_score;
      } else if (debug_options.capture_area_prior_debug) {
        drawAreaPriorAnnotation(
          outputs.area_prior_debug_image,
          candidate.bbox,
          candidate.area_px,
          candidate.area_prior,
          cv::Scalar(0, 255, 255));
      }

      if (mode == TrackingMode::Track && has_filtered_state_) {
        const double distance =
          cv::norm(candidate.raw_center_px - filtered_raw_center_px_);
        const double distance_norm = distance /
          std::max(1.0, std::hypot(
            static_cast<double>(outputs.roi.width),
            static_cast<double>(outputs.roi.height)));
        const double proximity_score = clamp01(1.0 - distance_norm);
        if (enable_distance_aware_area_prior_) {
          candidate.score =
            0.40 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score +
            0.05 * fill_score + 0.10 * area_score;
          candidate.confidence =
            clamp01(
            0.30 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score +
            0.10 * fill_score + 0.15 * area_score);
        } else {
          candidate.score =
            0.45 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score +
            0.10 * fill_score;
          candidate.confidence =
            clamp01(
            0.35 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score +
            0.20 * fill_score);
        }
      } else {
        candidate.score =
          0.40 * aspect_score + 0.25 * fill_score + 0.20 * valid_score + 0.15 * area_score;
        candidate.confidence =
          clamp01(
          0.30 * aspect_score + 0.20 * fill_score + 0.30 * valid_score + 0.20 * area_score);
      }

      if (debug_options.capture_filtered_mask) {
        for (const cv::Point & pixel : candidate.pixels) {
          outputs.filtered_debug_mask.at<uchar>(pixel) = 255;
        }
      }

      if (!outputs.candidate.valid || candidate.score > outputs.candidate.score) {
        candidate.valid = true;
        outputs.candidate = std::move(candidate);
      }
    }

    return outputs;
  }

  DetectorOutputs runSearch(
    const cv::Mat & frame,
    const DetectionDebugOptions & debug_options)
  {
    if (search_downsample_scale_ >= 0.999) {
      return detectCandidateInRoi(
        frame,
        cv::Rect(0, 0, frame.cols, frame.rows),
        TrackingMode::Search,
        debug_options);
    }

    cv::Mat frame_small;
    cv::resize(
      frame,
      frame_small,
      cv::Size(),
      search_downsample_scale_,
      search_downsample_scale_,
      cv::INTER_LINEAR);
    cv::Mat search_mask_small;
    if (debug_options.capture_search_debug) {
      buildOrangeMask(frame_small, search_mask_small);
    }
    cv::Mat search_area_prior_small;
    const std::vector<CoarseCandidate> coarse_candidates = findCoarseCandidates(
      frame_small,
      debug_options.capture_search_area_prior ? &search_area_prior_small : nullptr);

    DetectorOutputs fallback_outputs;
    if (debug_options.capture_threshold_mask || debug_options.capture_raw_mask) {
      buildThresholdMask(frame, fallback_outputs.threshold_debug_mask);
    }
    if (debug_options.capture_morph_mask) {
      if (fallback_outputs.threshold_debug_mask.empty()) {
        buildThresholdMask(frame, fallback_outputs.threshold_debug_mask);
      }
      fallback_outputs.morph_debug_mask = fallback_outputs.threshold_debug_mask.clone();
      applyMorphOpen(fallback_outputs.morph_debug_mask);
    }
    if (debug_options.capture_raw_mask) {
      if (fallback_outputs.morph_debug_mask.empty()) {
        if (fallback_outputs.threshold_debug_mask.empty()) {
          buildThresholdMask(frame, fallback_outputs.threshold_debug_mask);
        }
        fallback_outputs.raw_debug_mask = fallback_outputs.threshold_debug_mask.clone();
        applyMorphOpen(fallback_outputs.raw_debug_mask);
      } else {
        fallback_outputs.raw_debug_mask = fallback_outputs.morph_debug_mask;
      }
    }
    if (debug_options.capture_search_debug) {
      cv::Mat search_overlay = frame_small.clone();
      if (search_overlay.channels() == 1) {
        cv::cvtColor(search_overlay, search_overlay, cv::COLOR_GRAY2BGR);
      }
      if (!search_mask_small.empty()) {
        for (const CoarseCandidate & coarse_candidate : coarse_candidates) {
          cv::rectangle(search_overlay, coarse_candidate.bbox, cv::Scalar(0, 255, 255), 1);
        }
      }
      cv::resize(
        search_overlay,
        fallback_outputs.search_debug_image,
        frame.size(),
        0.0,
        0.0,
        cv::INTER_NEAREST);
    }
    if (debug_options.capture_search_area_prior) {
      if (search_area_prior_small.empty()) {
        search_area_prior_small = frame_small.clone();
      }
      cv::resize(
        search_area_prior_small,
        fallback_outputs.search_area_prior_debug_image,
        frame.size(),
        0.0,
        0.0,
        cv::INTER_NEAREST);
    }
    if (coarse_candidates.empty()) {
      return fallback_outputs;
    }

    const double inv_scale = 1.0 / search_downsample_scale_;
    DetectionDebugOptions roi_debug_options = debug_options;
    roi_debug_options.capture_raw_mask = false;
    for (const CoarseCandidate & coarse_candidate : coarse_candidates) {
      const cv::Rect coarse_raw_bbox(
        static_cast<int>(std::floor(static_cast<double>(coarse_candidate.bbox.x) * inv_scale)),
        static_cast<int>(std::floor(static_cast<double>(coarse_candidate.bbox.y) * inv_scale)),
        std::max(
          1,
          static_cast<int>(std::ceil(static_cast<double>(coarse_candidate.bbox.width) * inv_scale))),
        std::max(
          1,
          static_cast<int>(
            std::ceil(static_cast<double>(coarse_candidate.bbox.height) * inv_scale))));
      const cv::Rect refine_roi = scaleRectAroundCenter(coarse_raw_bbox, 1.6, frame.size());
      DetectorOutputs trial_outputs = detectCandidateInRoi(
        frame,
        refine_roi,
        TrackingMode::Search,
        roi_debug_options);
      trial_outputs.threshold_debug_mask = fallback_outputs.threshold_debug_mask;
      trial_outputs.morph_debug_mask = fallback_outputs.morph_debug_mask;
      trial_outputs.raw_debug_mask = fallback_outputs.raw_debug_mask;
      trial_outputs.search_debug_image = fallback_outputs.search_debug_image;
      trial_outputs.search_area_prior_debug_image = fallback_outputs.search_area_prior_debug_image;
      fallback_outputs.roi = trial_outputs.roi;
      fallback_outputs.filtered_debug_mask = trial_outputs.filtered_debug_mask;
      fallback_outputs.area_debug_mask = trial_outputs.area_debug_mask;
      fallback_outputs.area_prior_filter_debug_mask = trial_outputs.area_prior_filter_debug_mask;
      fallback_outputs.area_prior_debug_image = trial_outputs.area_prior_debug_image;
      fallback_outputs.aspect_debug_mask = trial_outputs.aspect_debug_mask;
      fallback_outputs.edge_debug_mask = trial_outputs.edge_debug_mask;
      fallback_outputs.fill_debug_mask = trial_outputs.fill_debug_mask;
      fallback_outputs.lut_debug_mask = trial_outputs.lut_debug_mask;
      fallback_outputs.top_band_debug_mask = trial_outputs.top_band_debug_mask;
      fallback_outputs.roi_debug_mask = trial_outputs.roi_debug_mask;
      if (trial_outputs.candidate.valid) {
        return trial_outputs;
      }
    }

    return fallback_outputs;
  }

  cv::Rect buildTrackRoi(const cv::Size & image_size) const
  {
    if (!has_filtered_state_ || last_bbox_.width <= 0 || last_bbox_.height <= 0) {
      return cv::Rect(0, 0, image_size.width, image_size.height);
    }

    const double miss_scale = std::pow(1.5, static_cast<double>(lost_frame_count_));
    const double combined_scale = roi_scale_ * miss_scale;
    cv::Rect scaled = scaleRectAroundCenter(last_bbox_, combined_scale, image_size);

    const int half_width =
      std::clamp(scaled.width / 2, roi_min_half_size_px_, roi_max_half_size_px_);
    const int half_height =
      std::clamp(scaled.height / 2, roi_min_half_size_px_, roi_max_half_size_px_);
    return buildCenteredRect(filtered_raw_center_px_, half_width, half_height, image_size);
  }

  void publishStatus(bool detected, double confidence)
  {
    std_msgs::msg::Bool detected_msg;
    detected_msg.data = detected;
    detected_pub_->publish(detected_msg);

    std_msgs::msg::Float32 confidence_msg;
    confidence_msg.data = static_cast<float>(confidence);
    confidence_pub_->publish(confidence_msg);
  }

  void publishDetection(
    const std_msgs::msg::Header & header,
    const Candidate & candidate,
    const cv::Point2d & filtered_raw_center_px,
    const cv::Point2d & filtered_ground_center_m)
  {
    // `ball_center_ground` is the physical ball-center estimate in `base_link`.
    // `ball_center_raw_px` is the tracked image-space representative center after EMA.
    // `ball_top_raw_px` is the raw-image representative point of the top-band envelope.
    geometry_msgs::msg::PoseStamped ground_msg;
    ground_msg.header = header;
    ground_msg.header.frame_id = "base_link";
    ground_msg.pose.position.x = filtered_ground_center_m.x;
    ground_msg.pose.position.y = filtered_ground_center_m.y;
    ground_msg.pose.position.z = lut_.ball_diameter_m * 0.5;
    ground_msg.pose.orientation.w = 1.0;
    ball_center_ground_pub_->publish(ground_msg);

    geometry_msgs::msg::PointStamped raw_center_msg;
    raw_center_msg.header = header;
    raw_center_msg.header.frame_id = "camera_raw_px";
    raw_center_msg.point.x = filtered_raw_center_px.x;
    raw_center_msg.point.y = filtered_raw_center_px.y;
    raw_center_msg.point.z = 0.0;
    ball_center_raw_pub_->publish(raw_center_msg);

    geometry_msgs::msg::PointStamped top_msg;
    top_msg.header = header;
    top_msg.header.frame_id = "camera_raw_px";
    top_msg.point.x = candidate.top_point_px.x;
    top_msg.point.y = candidate.top_point_px.y;
    top_msg.point.z = 0.0;
    ball_top_raw_pub_->publish(top_msg);
  }

  void publishDebugImages(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const cv::Mat & frame,
    const DetectorOutputs & outputs,
    TrackingMode mode,
    const DebugRequest & debug_request)
  {
    if (!debug_request.needAnyOutputs()) {
      return;
    }

    cv::Mat threshold_debug_mask;
    if (debug_request.show_threshold_mask) {
      threshold_debug_mask = outputs.threshold_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.threshold_debug_mask;
    }

    cv::Mat morph_debug_mask;
    if (debug_request.show_morph_mask || debug_request.show_roi_mask) {
      morph_debug_mask = outputs.morph_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.morph_debug_mask;
    }

    cv::Mat raw_debug_mask;
    if (debug_request.publish_raw_mask) {
      raw_debug_mask = outputs.raw_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.raw_debug_mask;
      debug_mask_pub_->publish(
        *cv_bridge::CvImage(msg->header, "mono8", raw_debug_mask).toImageMsg());
    }

    cv::Mat filtered_debug_mask;
    if (debug_request.publish_filtered_mask) {
      filtered_debug_mask = outputs.filtered_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.filtered_debug_mask;
      debug_mask_filtered_pub_->publish(
        *cv_bridge::CvImage(msg->header, "mono8", filtered_debug_mask).toImageMsg());
    }

    const auto maskOrZeros = [&frame](const cv::Mat & mask) {
        return mask.empty() ? cv::Mat::zeros(frame.size(), CV_8UC1) : mask;
      };

    cv::Mat overlay;
    if (debug_request.needOverlay()) {
      overlay = frame.clone();
      if (outputs.roi.width > 0 && outputs.roi.height > 0) {
        cv::rectangle(overlay, outputs.roi, cv::Scalar(255, 255, 0), 1);
      }
      if (outputs.candidate.valid) {
        cv::rectangle(overlay, outputs.candidate.bbox, cv::Scalar(0, 255, 255), 2);
        for (const cv::Point & pixel : outputs.candidate.top_band_pixels) {
          overlay.at<cv::Vec3b>(pixel) = cv::Vec3b(0, 255, 255);
        }
        cv::circle(
          overlay,
          cv::Point(
            static_cast<int>(std::round(outputs.candidate.top_point_px.x)),
            static_cast<int>(std::round(outputs.candidate.top_point_px.y))),
          5,
          cv::Scalar(0, 0, 255),
          2);
        cv::circle(
          overlay,
          cv::Point(
            static_cast<int>(std::round(filtered_raw_center_px_.x)),
            static_cast<int>(std::round(filtered_raw_center_px_.y))),
          5,
          cv::Scalar(255, 0, 0),
          2);
        std::ostringstream oss;
        oss << trackingModeToString(mode) << " conf="
            << std::fixed << std::setprecision(2) << outputs.candidate.confidence
            << " lost=" << lost_frame_count_;
        cv::putText(
          overlay,
          oss.str(),
          cv::Point(10, 30),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 255, 0),
          2);
        if (enable_distance_aware_area_prior_) {
          drawMultilineText(
            overlay,
            formatAreaPriorDebugLines(outputs.candidate.area_px, outputs.candidate.area_prior),
            cv::Point(10, 58),
            0.5,
            cv::Scalar(0, 255, 255),
            1);
        }
      } else {
        const std::string label = trackingModeToString(mode) + " no detection";
        cv::putText(
          overlay,
          label,
          cv::Point(10, 30),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar(0, 0, 255),
          2);
      }
    }

    if (debug_request.publish_overlay) {
      debug_image_pub_->publish(
        *cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg());
    }

    if (debug_request.show_input_image) {
      cv::imshow(kInputWindowName, frame);
      resizeWindowToFitImage(kInputWindowName, frame, display_max_width_, display_max_height_);
    }
    if (debug_request.show_threshold_mask) {
      cv::imshow(kThresholdMaskWindowName, threshold_debug_mask);
      resizeWindowToFitImage(
        kThresholdMaskWindowName,
        threshold_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_morph_mask) {
      cv::imshow(kMorphMaskWindowName, morph_debug_mask);
      resizeWindowToFitImage(
        kMorphMaskWindowName,
        morph_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_raw_mask) {
      if (raw_debug_mask.empty()) {
        raw_debug_mask = outputs.raw_debug_mask.empty() ?
          cv::Mat::zeros(frame.size(), CV_8UC1) :
          outputs.raw_debug_mask;
      }
      cv::imshow(kRawMaskWindowName, raw_debug_mask);
      resizeWindowToFitImage(
        kRawMaskWindowName,
        raw_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_filtered_mask) {
      if (filtered_debug_mask.empty()) {
        filtered_debug_mask = outputs.filtered_debug_mask.empty() ?
          cv::Mat::zeros(frame.size(), CV_8UC1) :
          outputs.filtered_debug_mask;
      }
      cv::imshow(kFilteredMaskWindowName, filtered_debug_mask);
      resizeWindowToFitImage(
        kFilteredMaskWindowName,
        filtered_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_area_filter) {
      const cv::Mat area_debug_mask = maskOrZeros(outputs.area_debug_mask);
      cv::imshow(kAreaFilterWindowName, area_debug_mask);
      resizeWindowToFitImage(
        kAreaFilterWindowName,
        area_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_area_prior_filter) {
      const cv::Mat area_prior_filter_debug_mask = maskOrZeros(outputs.area_prior_filter_debug_mask);
      cv::imshow(kAreaPriorFilterWindowName, area_prior_filter_debug_mask);
      resizeWindowToFitImage(
        kAreaPriorFilterWindowName,
        area_prior_filter_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_area_prior_debug) {
      const cv::Mat area_prior_debug_image = outputs.area_prior_debug_image.empty() ?
        frame.clone() :
        outputs.area_prior_debug_image;
      cv::imshow(kAreaPriorDebugWindowName, area_prior_debug_image);
      resizeWindowToFitImage(
        kAreaPriorDebugWindowName,
        area_prior_debug_image,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_aspect_filter) {
      const cv::Mat aspect_debug_mask = maskOrZeros(outputs.aspect_debug_mask);
      cv::imshow(kAspectFilterWindowName, aspect_debug_mask);
      resizeWindowToFitImage(
        kAspectFilterWindowName,
        aspect_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_edge_filter) {
      const cv::Mat edge_debug_mask = maskOrZeros(outputs.edge_debug_mask);
      cv::imshow(kEdgeFilterWindowName, edge_debug_mask);
      resizeWindowToFitImage(
        kEdgeFilterWindowName,
        edge_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_fill_filter) {
      const cv::Mat fill_debug_mask = maskOrZeros(outputs.fill_debug_mask);
      cv::imshow(kFillFilterWindowName, fill_debug_mask);
      resizeWindowToFitImage(
        kFillFilterWindowName,
        fill_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_lut_filter) {
      const cv::Mat lut_debug_mask = maskOrZeros(outputs.lut_debug_mask);
      cv::imshow(kLutFilterWindowName, lut_debug_mask);
      resizeWindowToFitImage(
        kLutFilterWindowName,
        lut_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_top_band_filter) {
      const cv::Mat top_band_debug_mask = maskOrZeros(outputs.top_band_debug_mask);
      cv::imshow(kTopBandFilterWindowName, top_band_debug_mask);
      resizeWindowToFitImage(
        kTopBandFilterWindowName,
        top_band_debug_mask,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_search_debug) {
      const cv::Mat search_debug_image = outputs.search_debug_image.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC3) :
        outputs.search_debug_image;
      cv::imshow(kSearchDebugWindowName, search_debug_image);
      resizeWindowToFitImage(
        kSearchDebugWindowName,
        search_debug_image,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_search_area_prior) {
      const cv::Mat search_area_prior_debug_image = outputs.search_area_prior_debug_image.empty() ?
        frame.clone() :
        outputs.search_area_prior_debug_image;
      cv::imshow(kSearchAreaPriorWindowName, search_area_prior_debug_image);
      resizeWindowToFitImage(
        kSearchAreaPriorWindowName,
        search_area_prior_debug_image,
        display_max_width_,
        display_max_height_);
    }
    if (debug_request.show_overlay_image) {
      cv::imshow(kOverlayWindowName, overlay);
      resizeWindowToFitImage(kOverlayWindowName, overlay, display_max_width_, display_max_height_);
    }
    if (debug_request.show_roi_image) {
      cv::Mat roi_view;
      if (outputs.roi.width > 0 && outputs.roi.height > 0) {
        roi_view = frame(outputs.roi).clone();
      } else {
        roi_view = frame.clone();
      }
      cv::imshow(kRoiWindowName, roi_view);
      resizeWindowToFitImage(kRoiWindowName, roi_view, display_max_width_, display_max_height_);
    }
    if (debug_request.show_roi_mask) {
      cv::Mat roi_mask_view;
      if (!outputs.roi_debug_mask.empty()) {
        roi_mask_view = outputs.roi_debug_mask;
      } else if (outputs.roi.width > 0 && outputs.roi.height > 0) {
        roi_mask_view = cv::Mat::zeros(outputs.roi.size(), CV_8UC1);
      } else {
        roi_mask_view = cv::Mat::zeros(frame.size(), CV_8UC1);
      }
      cv::imshow(kRoiMaskWindowName, roi_mask_view);
      resizeWindowToFitImage(
        kRoiMaskWindowName,
        roi_mask_view,
        display_max_width_,
        display_max_height_);
    }

    if (debug_request.needAnyWindows()) {
      cv::waitKey(1);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::array<long long, kTimingStageCount> stage_us{};
    const TimePoint callback_start = SteadyClock::now();

    cv::Mat frame;
    TimePoint stage_start = SteadyClock::now();
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge failed: %s",
        e.what());
      return;
    }
    stage_us[static_cast<std::size_t>(TimingStage::CvBridge)] =
      elapsedUs(stage_start, SteadyClock::now());

    if (frame.cols != lut_.source_width || frame.rows != lut_.source_height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Input image size %dx%d does not match LUT source size %dx%d. Frame dropped.",
        frame.cols,
        frame.rows,
        lut_.source_width,
        lut_.source_height);
      return;
    }

    if (!validateRobotMaskForFrame(frame)) {
      return;
    }

    const TrackingMode mode =
      (!force_search_mode_ && has_filtered_state_ && lost_frame_count_ < lost_frame_tolerance_) ?
      TrackingMode::Track :
      TrackingMode::Search;
    const DebugRequest debug_request = buildDebugRequest();
    DetectionDebugOptions debug_options;
    debug_options.capture_threshold_mask = debug_request.show_threshold_mask;
    debug_options.capture_morph_mask = debug_request.show_morph_mask;
    debug_options.capture_raw_mask = debug_request.needRawMask();
    debug_options.capture_filtered_mask = debug_request.needFilteredMask();
    debug_options.capture_area_mask = debug_request.show_area_filter;
    debug_options.capture_area_prior_filter_mask = debug_request.show_area_prior_filter;
    debug_options.capture_area_prior_debug = debug_request.show_area_prior_debug;
    debug_options.capture_aspect_mask = debug_request.show_aspect_filter;
    debug_options.capture_edge_mask = debug_request.show_edge_filter;
    debug_options.capture_fill_mask = debug_request.show_fill_filter;
    debug_options.capture_lut_mask = debug_request.show_lut_filter;
    debug_options.capture_top_band_mask = debug_request.show_top_band_filter;
    debug_options.capture_search_debug = debug_request.show_search_debug;
    debug_options.capture_search_area_prior = debug_request.show_search_area_prior;
    debug_options.capture_roi_mask = debug_request.show_roi_mask;

    DetectorOutputs outputs;
    if (mode == TrackingMode::Search) {
      stage_start = SteadyClock::now();
      outputs = runSearch(frame, debug_options);
      stage_us[static_cast<std::size_t>(TimingStage::Search)] =
        elapsedUs(stage_start, SteadyClock::now());
    } else {
      stage_start = SteadyClock::now();
      outputs = detectCandidateInRoi(
        frame,
        buildTrackRoi(frame.size()),
        TrackingMode::Track,
        debug_options);
      stage_us[static_cast<std::size_t>(TimingStage::Track)] =
        elapsedUs(stage_start, SteadyClock::now());
    }

    stage_start = SteadyClock::now();
    if (outputs.candidate.valid) {
      if (!has_filtered_state_) {
        filtered_raw_center_px_ = outputs.candidate.raw_center_px;
        filtered_ground_center_m_ = outputs.candidate.ground_center_m;
      } else {
        filtered_raw_center_px_ =
          outputs.candidate.raw_center_px * ema_alpha_ +
          filtered_raw_center_px_ * (1.0 - ema_alpha_);
        filtered_ground_center_m_ =
          outputs.candidate.ground_center_m * ema_alpha_ +
          filtered_ground_center_m_ * (1.0 - ema_alpha_);
      }

      last_bbox_ = outputs.candidate.bbox;
      has_filtered_state_ = true;
      lost_frame_count_ = 0;
      publishStatus(true, outputs.candidate.confidence);
      publishDetection(msg->header, outputs.candidate, filtered_raw_center_px_, filtered_ground_center_m_);
    } else {
      ++lost_frame_count_;
      if (lost_frame_count_ >= lost_frame_tolerance_) {
        has_filtered_state_ = false;
      }
      publishStatus(false, 0.0);
    }
    stage_us[static_cast<std::size_t>(TimingStage::Publish)] =
      elapsedUs(stage_start, SteadyClock::now());

    stage_start = SteadyClock::now();
    publishDebugImages(msg, frame, outputs, mode, debug_request);
    stage_us[static_cast<std::size_t>(TimingStage::Debug)] =
      elapsedUs(stage_start, SteadyClock::now());

    const TimePoint callback_end = SteadyClock::now();
    stage_us[static_cast<std::size_t>(TimingStage::CallbackTotal)] =
      elapsedUs(callback_start, callback_end);
    long long accounted = 0;
    for (std::size_t i = 0; i < stage_us.size(); ++i) {
      if (
        i == static_cast<std::size_t>(TimingStage::CallbackTotal) ||
        i == static_cast<std::size_t>(TimingStage::UnaccountedOverhead))
      {
        continue;
      }
      accounted += stage_us[i];
    }
    stage_us[static_cast<std::size_t>(TimingStage::UnaccountedOverhead)] =
      std::max(
      0LL,
      stage_us[static_cast<std::size_t>(TimingStage::CallbackTotal)] - accounted);

    if (publish_processing_time_ && processing_time_pub_) {
      std_msgs::msg::Float32 processing_time_msg;
      processing_time_msg.data = static_cast<float>(
        stage_us[static_cast<std::size_t>(TimingStage::CallbackTotal)] / 1000.0);
      processing_time_pub_->publish(processing_time_msg);
    }

    logTimingSummary(stage_us);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_center_ground_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ball_center_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ball_top_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_pub_;

  std::string input_topic_;
  std::string lut_file_;
  std::string robot_mask_path_;
  int orange_h_min_ = 5;
  int orange_h_max_ = 30;
  int orange_s_min_ = 100;
  int orange_v_min_ = 60;
  bool enable_morph_open_ = true;
  int morph_kernel_size_ = 3;
  double search_downsample_scale_ = 0.5;
  int min_blob_area_px_ = 20;
  int max_blob_area_px_ = 40000;
  bool enable_distance_aware_area_prior_ = true;
  double area_prior_near_min_ratio_ = 0.75;
  double area_prior_near_max_ratio_ = 1.25;
  double area_prior_far_min_ratio_ = 0.50;
  double area_prior_far_max_ratio_ = 2.00;
  double min_aspect_ratio_ = 0.35;
  double max_aspect_ratio_ = 2.8;
  double max_edge_touch_ratio_ = 0.35;
  int top_band_px_ = 4;
  double roi_scale_ = 2.0;
  int roi_min_half_size_px_ = 50;
  int roi_max_half_size_px_ = 140;
  int lost_frame_tolerance_ = 3;
  double ema_alpha_ = 0.6;
  bool force_search_mode_ = false;
  bool enable_timing_log_ = true;
  int timing_log_interval_ = 30;
  bool publish_processing_time_ = true;
  std::string processing_time_topic_;
  bool enable_image_view_ = false;
  bool show_input_image_ = true;
  bool show_threshold_mask_ = false;
  bool show_morph_mask_ = false;
  bool show_raw_mask_ = true;
  bool show_filtered_mask_ = true;
  bool show_area_filter_ = false;
  bool show_area_prior_filter_ = false;
  bool show_area_prior_debug_ = false;
  bool show_aspect_filter_ = false;
  bool show_edge_filter_ = false;
  bool show_fill_filter_ = false;
  bool show_lut_filter_ = false;
  bool show_top_band_filter_ = false;
  bool show_search_debug_ = false;
  bool show_search_area_prior_ = false;
  bool show_overlay_image_ = true;
  bool show_roi_image_ = true;
  bool show_roi_mask_ = false;
  int display_max_width_ = 960;
  int display_max_height_ = 720;

  LutData lut_;
  bool robot_mask_enabled_ = false;
  bool robot_mask_validated_ = false;
  cv::Mat robot_allowed_mask_;
  cv::Mat resized_robot_allowed_mask_;
  cv::Size resized_robot_allowed_mask_size_;
  bool has_filtered_state_ = false;
  int lost_frame_count_ = 0;
  cv::Point2d filtered_raw_center_px_;
  cv::Point2d filtered_ground_center_m_;
  cv::Rect last_bbox_;

  bool input_window_created_ = false;
  bool threshold_mask_window_created_ = false;
  bool morph_mask_window_created_ = false;
  bool overlay_window_created_ = false;
  bool raw_mask_window_created_ = false;
  bool filtered_mask_window_created_ = false;
  bool area_filter_window_created_ = false;
  bool area_prior_filter_window_created_ = false;
  bool area_prior_debug_window_created_ = false;
  bool aspect_filter_window_created_ = false;
  bool edge_filter_window_created_ = false;
  bool fill_filter_window_created_ = false;
  bool lut_filter_window_created_ = false;
  bool top_band_filter_window_created_ = false;
  bool search_debug_window_created_ = false;
  bool search_area_prior_window_created_ = false;
  bool roi_window_created_ = false;
  bool roi_mask_window_created_ = false;

  std::size_t timing_frames_in_interval_ = 0;
  std::array<long long, kTimingStageCount> timing_interval_totals_us_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrangeBallDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
