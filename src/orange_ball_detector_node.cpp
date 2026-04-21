#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
#include <opencv2/imgproc.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

constexpr int kHueMax = 179;
constexpr int kByteMax = 255;
constexpr char kOverlayWindowName[] = "Orange Ball Overlay";
constexpr char kMaskWindowName[] = "Orange Ball Mask";
constexpr char kRoiWindowName[] = "Orange Ball ROI";
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

struct LutData
{
  cv::Mat ground_x_m;
  cv::Mat ground_y_m;
  cv::Mat valid_mask;
  int source_width = 0;
  int source_height = 0;
  double ocam_xc = 0.0;
  double ocam_yc = 0.0;
  double camera_height_m = 0.0;
  double ball_diameter_m = 0.0;
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
  double score = -std::numeric_limits<double>::infinity();
  double confidence = 0.0;
};

struct DetectorOutputs
{
  Candidate candidate;
  cv::Mat raw_debug_mask;
  cv::Mat filtered_debug_mask;
  cv::Rect roi;
};

struct CoarseCandidate
{
  bool valid = false;
  cv::Rect bbox;
  double score = -std::numeric_limits<double>::infinity();
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
      "roi_scale=%.2f, lost_frame_tolerance=%d, ema_alpha=%.2f, image_view=%s, timing_log=%s",
      input_topic_.c_str(),
      lut_file_.c_str(),
      search_downsample_scale_,
      roi_scale_,
      lost_frame_tolerance_,
      ema_alpha_,
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
    declare_parameter("orange_h_min", 5);
    declare_parameter("orange_h_max", 30);
    declare_parameter("orange_s_min", 100);
    declare_parameter("orange_v_min", 60);
    declare_parameter("enable_morph_open", true);
    declare_parameter("morph_kernel_size", 3);
    declare_parameter("search_downsample_scale", 0.5);
    declare_parameter("min_blob_area_px", 20);
    declare_parameter("max_blob_area_px", 40000);
    declare_parameter("min_aspect_ratio", 0.35);
    declare_parameter("max_aspect_ratio", 2.8);
    declare_parameter("max_edge_touch_ratio", 0.35);
    declare_parameter("top_band_px", 4);
    declare_parameter("roi_scale", 2.0);
    declare_parameter("roi_min_half_size_px", 50);
    declare_parameter("roi_max_half_size_px", 140);
    declare_parameter("lost_frame_tolerance", 3);
    declare_parameter("ema_alpha", 0.6);
    declare_parameter("enable_timing_log", true);
    declare_parameter("timing_log_interval", 30);
    declare_parameter("publish_processing_time", true);
    declare_parameter<std::string>("processing_time_topic", "~/processing_time_ms");
    declare_parameter("enable_image_view", false);
    declare_parameter("display_max_width", 960);
    declare_parameter("display_max_height", 720);
  }

  void loadParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    lut_file_ = get_parameter("lut_file").as_string();
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
    enable_timing_log_ = get_parameter("enable_timing_log").as_bool();
    timing_log_interval_ =
      std::max(1, static_cast<int>(get_parameter("timing_log_interval").as_int()));
    publish_processing_time_ = get_parameter("publish_processing_time").as_bool();
    processing_time_topic_ = get_parameter("processing_time_topic").as_string();
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
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
    if (processing_time_topic_.empty()) {
      throw std::runtime_error("Parameter 'processing_time_topic' must not be empty.");
    }
    if (lut_file_.empty()) {
      throw std::runtime_error("Parameter 'lut_file' must not be empty.");
    }
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

    syncWindow(kOverlayWindowName, enable_image_view_, overlay_window_created_);
    syncWindow(kMaskWindowName, enable_image_view_, mask_window_created_);
    syncWindow(kRoiWindowName, enable_image_view_, roi_window_created_);
  }

  void destroyDebugWindows()
  {
    syncWindow(kOverlayWindowName, false, overlay_window_created_);
    syncWindow(kMaskWindowName, false, mask_window_created_);
    syncWindow(kRoiWindowName, false, roi_window_created_);
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

  void buildOrangeMask(const cv::Mat & bgr, cv::Mat & mask) const
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

    if (enable_morph_open_ && morph_kernel_size_ > 1) {
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(morph_kernel_size_, morph_kernel_size_));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
  }

  std::vector<CoarseCandidate> findCoarseCandidates(const cv::Mat & frame_small)
  {
    std::vector<CoarseCandidate> coarse_candidates;
    cv::Mat mask;
    buildOrangeMask(frame_small, mask);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count =
      cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

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
      const double area_score =
        clamp01(static_cast<double>(area) / static_cast<double>(scaled_max_area));
      const double score = 0.45 * aspect_score + 0.30 * fill_score + 0.25 * area_score;

      CoarseCandidate coarse_candidate;
      coarse_candidate.valid = true;
      coarse_candidate.bbox = bbox;
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
    TrackingMode mode)
  {
    DetectorOutputs outputs;
    outputs.roi = roi;
    outputs.raw_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    outputs.filtered_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    if (roi.width <= 0 || roi.height <= 0) {
      return outputs;
    }

    const cv::Mat frame_roi = frame(roi);
    cv::Mat mask_roi;
    buildOrangeMask(frame_roi, mask_roi);
    mask_roi.copyTo(outputs.raw_debug_mask(roi));

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

      candidate.edge_touch_ratio = computeEdgeTouchRatio(candidate.bbox, frame.size());
      if (candidate.edge_touch_ratio > max_edge_touch_ratio_) {
        continue;
      }

      candidate.fill_ratio =
        candidate.area_px /
        std::max(
        1.0,
        static_cast<double>(candidate.bbox.width) * static_cast<double>(candidate.bbox.height));
      if (candidate.fill_ratio < kMinFillRatio) {
        continue;
      }

      candidate.pixels = std::move(label_pixels[static_cast<std::size_t>(label)]);
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
      const double area_score =
        clamp01(candidate.area_px / static_cast<double>(max_blob_area_px_));

      if (mode == TrackingMode::Track && has_filtered_state_) {
        const double distance =
          cv::norm(candidate.raw_center_px - filtered_raw_center_px_);
        const double distance_norm = distance /
          std::max(1.0, std::hypot(
            static_cast<double>(outputs.roi.width),
            static_cast<double>(outputs.roi.height)));
        const double proximity_score = clamp01(1.0 - distance_norm);
        candidate.score =
          0.45 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score + 0.10 * fill_score;
        candidate.confidence =
          clamp01(
          0.35 * proximity_score + 0.25 * aspect_score + 0.20 * valid_score +
          0.20 * fill_score);
      } else {
        candidate.score =
          0.40 * aspect_score + 0.25 * fill_score + 0.20 * valid_score + 0.15 * area_score;
        candidate.confidence =
          clamp01(
          0.30 * aspect_score + 0.20 * fill_score + 0.30 * valid_score + 0.20 * area_score);
      }

      for (const cv::Point & pixel : candidate.pixels) {
        outputs.filtered_debug_mask.at<uchar>(pixel) = 255;
      }

      if (!outputs.candidate.valid || candidate.score > outputs.candidate.score) {
        candidate.valid = true;
        outputs.candidate = std::move(candidate);
      }
    }

    return outputs;
  }

  DetectorOutputs runSearch(const cv::Mat & frame)
  {
    if (search_downsample_scale_ >= 0.999) {
      return detectCandidateInRoi(
        frame,
        cv::Rect(0, 0, frame.cols, frame.rows),
        TrackingMode::Search);
    }

    cv::Mat frame_small;
    cv::resize(
      frame,
      frame_small,
      cv::Size(),
      search_downsample_scale_,
      search_downsample_scale_,
      cv::INTER_LINEAR);
    const std::vector<CoarseCandidate> coarse_candidates = findCoarseCandidates(frame_small);

    DetectorOutputs fallback_outputs;
    fallback_outputs.raw_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    fallback_outputs.filtered_debug_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    buildOrangeMask(frame, fallback_outputs.raw_debug_mask);
    if (coarse_candidates.empty()) {
      return fallback_outputs;
    }

    const double inv_scale = 1.0 / search_downsample_scale_;
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
      DetectorOutputs trial_outputs = detectCandidateInRoi(frame, refine_roi, TrackingMode::Search);
      trial_outputs.raw_debug_mask = fallback_outputs.raw_debug_mask;
      fallback_outputs.roi = trial_outputs.roi;
      fallback_outputs.filtered_debug_mask = std::move(trial_outputs.filtered_debug_mask);
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
    TrackingMode mode)
  {
    const bool need_debug_mask =
      debug_mask_pub_->get_subscription_count() > 0U || mask_window_created_;
    const bool need_debug_mask_filtered =
      debug_mask_filtered_pub_->get_subscription_count() > 0U;
    const bool need_debug_image =
      debug_image_pub_->get_subscription_count() > 0U || overlay_window_created_ || roi_window_created_;

    cv::Mat debug_mask;
    if (need_debug_mask) {
      debug_mask = outputs.raw_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.raw_debug_mask.clone();
      debug_mask_pub_->publish(
        *cv_bridge::CvImage(msg->header, "mono8", debug_mask).toImageMsg());
    }

    if (need_debug_mask_filtered) {
      const cv::Mat debug_mask_filtered = outputs.filtered_debug_mask.empty() ?
        cv::Mat::zeros(frame.size(), CV_8UC1) :
        outputs.filtered_debug_mask;
      debug_mask_filtered_pub_->publish(
        *cv_bridge::CvImage(msg->header, "mono8", debug_mask_filtered).toImageMsg());
    }

    if (!need_debug_image) {
      return;
    }

    cv::Mat overlay = frame.clone();
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

    debug_image_pub_->publish(
      *cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg());

    if (overlay_window_created_) {
      cv::imshow(kOverlayWindowName, overlay);
      resizeWindowToFitImage(kOverlayWindowName, overlay, display_max_width_, display_max_height_);
    }
    if (mask_window_created_) {
      const cv::Mat & mask_to_show = debug_mask.empty() ?
        outputs.raw_debug_mask :
        debug_mask;
      cv::imshow(kMaskWindowName, mask_to_show);
      resizeWindowToFitImage(kMaskWindowName, mask_to_show, display_max_width_, display_max_height_);
    }
    if (roi_window_created_) {
      cv::Mat roi_view;
      if (outputs.roi.width > 0 && outputs.roi.height > 0) {
        roi_view = frame(outputs.roi).clone();
      } else {
        roi_view = frame.clone();
      }
      cv::imshow(kRoiWindowName, roi_view);
      resizeWindowToFitImage(kRoiWindowName, roi_view, display_max_width_, display_max_height_);
    }
    if (overlay_window_created_ || mask_window_created_ || roi_window_created_) {
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

    const TrackingMode mode =
      (has_filtered_state_ && lost_frame_count_ < lost_frame_tolerance_) ?
      TrackingMode::Track :
      TrackingMode::Search;

    DetectorOutputs outputs;
    if (mode == TrackingMode::Search) {
      stage_start = SteadyClock::now();
      outputs = runSearch(frame);
      stage_us[static_cast<std::size_t>(TimingStage::Search)] =
        elapsedUs(stage_start, SteadyClock::now());
    } else {
      stage_start = SteadyClock::now();
      outputs = detectCandidateInRoi(frame, buildTrackRoi(frame.size()), TrackingMode::Track);
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
    publishDebugImages(msg, frame, outputs, mode);
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
  int orange_h_min_ = 5;
  int orange_h_max_ = 30;
  int orange_s_min_ = 100;
  int orange_v_min_ = 60;
  bool enable_morph_open_ = true;
  int morph_kernel_size_ = 3;
  double search_downsample_scale_ = 0.5;
  int min_blob_area_px_ = 20;
  int max_blob_area_px_ = 40000;
  double min_aspect_ratio_ = 0.35;
  double max_aspect_ratio_ = 2.8;
  double max_edge_touch_ratio_ = 0.35;
  int top_band_px_ = 4;
  double roi_scale_ = 2.0;
  int roi_min_half_size_px_ = 50;
  int roi_max_half_size_px_ = 140;
  int lost_frame_tolerance_ = 3;
  double ema_alpha_ = 0.6;
  bool enable_timing_log_ = true;
  int timing_log_interval_ = 30;
  bool publish_processing_time_ = true;
  std::string processing_time_topic_;
  bool enable_image_view_ = false;
  int display_max_width_ = 960;
  int display_max_height_ = 720;

  LutData lut_;
  bool has_filtered_state_ = false;
  int lost_frame_count_ = 0;
  cv::Point2d filtered_raw_center_px_;
  cv::Point2d filtered_ground_center_m_;
  cv::Rect last_bbox_;

  bool overlay_window_created_ = false;
  bool mask_window_created_ = false;
  bool roi_window_created_ = false;

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
