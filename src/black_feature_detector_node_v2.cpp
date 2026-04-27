#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rcj_localization/msg/black_feature_detection.hpp"
#include "rcj_localization/msg/black_feature_detection_array.hpp"

namespace {

constexpr char kInputWindowName[] = "Black Feature Detector Input";
constexpr char kInputBlackMaskWindowName[] = "Black Feature Detector Input Black Mask";
constexpr char kBinaryNormalizedMaskWindowName[] = "Black Feature Detector Binary Normalized Mask";
constexpr char kMorphOpenMaskWindowName[] = "Black Feature Detector Morph Open Mask";
constexpr char kMorphCloseMaskWindowName[] = "Black Feature Detector Morph Close Mask";
constexpr char kBorderFilteredMaskWindowName[] = "Black Feature Detector Border Filtered Mask";
constexpr char kCleanMaskWindowName[] = "Black Feature Detector Clean Mask";
constexpr char kDotMaskWindowName[] = "Black Feature Detector Dot Mask";
constexpr char kCircleMaskWindowName[] = "Black Feature Detector Circle Mask";
constexpr char kCircleCandidateMaskWindowName[] = "Black Feature Detector Circle Candidate Mask";
constexpr char kCircleRejectedComponentMaskWindowName[] =
  "Black Feature Detector Circle Rejected Component Mask";
constexpr char kCircleRejectedArcMaskWindowName[] =
  "Black Feature Detector Circle Rejected Arc Mask";
constexpr char kCircleRadiusRejectedMaskWindowName[] =
  "Black Feature Detector Circle Radius Rejected Mask";
constexpr char kCircleTooFewPixelsRejectedMaskWindowName[] =
  "Black Feature Detector Circle Too Few Pixels Rejected Mask";
constexpr char kCircleRadialFitRejectedMaskWindowName[] =
  "Black Feature Detector Circle Radial Fit Rejected Mask";
constexpr char kCircleNoSegmentsRejectedMaskWindowName[] =
  "Black Feature Detector Circle No Segments Rejected Mask";
constexpr char kFinalMaskWindowName[] = "Black Feature Detector Final Mask";
constexpr char kDebugWindowName[] = "Black Feature Detector Debug";
constexpr double kPi = 3.14159265358979323846;

enum class CircleRejectReason
{
  kNone,
  kRadiusOutOfRange,
  kTooFewPixels,
  kBadRadialFit,
  kNoSegments,
};

struct CircleArcCandidate
{
  cv::Mat mask;
  double start_deg = 0.0;
  double end_deg = 0.0;
  double ring_ratio = 0.0;
};

struct RejectedCircleArcCandidate
{
  cv::Mat mask;
  double start_deg = 0.0;
  double end_deg = 0.0;
  bool rejected_for_span = false;
  bool rejected_for_ring_ratio = false;
  bool rejected_for_inner_ratio = false;
  bool rejected_for_outer_ratio = false;
};

struct CircleEvaluationResult
{
  bool accepted = false;
  cv::Point2f center;
  float radius = 0.0f;
  double coverage_deg = 0.0;
  double mean_ring_ratio = 0.0;
  CircleRejectReason reject_reason = CircleRejectReason::kNone;
  std::vector<CircleArcCandidate> arcs;
  std::vector<RejectedCircleArcCandidate> rejected_arcs;
};

struct BinSegment
{
  int start = 0;
  int end = 0;
};

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

int makeOddKernel(int value, int minimum_value = 1)
{
  const int clamped = std::max(minimum_value, value);
  return clamped % 2 == 0 ? clamped + 1 : clamped;
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
    std::max(1, static_cast<int>(std::round(image_size.width * scale))),
    std::max(1, static_cast<int>(std::round(image_size.height * scale))));
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

double normalizeAngleDeg(double angle_deg)
{
  double normalized = std::fmod(angle_deg, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

bool angleInRange(double angle_deg, double start_deg, double end_deg)
{
  const double angle = normalizeAngleDeg(angle_deg);
  const double start = normalizeAngleDeg(start_deg);
  const double end = normalizeAngleDeg(end_deg);

  if (std::abs(start - end) < 1e-6) {
    return true;
  }
  if (start < end) {
    return angle >= start && angle < end;
  }
  return angle >= start || angle < end;
}

double computeMedian(std::vector<float> values)
{
  if (values.empty()) {
    return 0.0;
  }

  const auto mid_it = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid_it, values.end());
  const double upper = *mid_it;
  if (values.size() % 2 == 1) {
    return upper;
  }

  const auto lower_it = std::max_element(values.begin(), mid_it);
  return 0.5 * (upper + *lower_it);
}

std::vector<BinSegment> collectOccupiedSegments(
  const std::vector<int> & bin_counts,
  int max_gap_bins)
{
  std::vector<BinSegment> segments;
  const int total_bins = static_cast<int>(bin_counts.size());
  int idx = 0;
  while (idx < total_bins) {
    while (idx < total_bins && bin_counts[static_cast<std::size_t>(idx)] == 0) {
      ++idx;
    }
    if (idx >= total_bins) {
      break;
    }
    const int start = idx;
    while (idx < total_bins && bin_counts[static_cast<std::size_t>(idx)] > 0) {
      ++idx;
    }
    segments.push_back({start, idx - 1});
  }

  if (segments.size() > 1 && bin_counts.front() > 0 && bin_counts.back() > 0) {
    const BinSegment first = segments.front();
    const BinSegment last = segments.back();
    segments.front() = {last.start, first.end + total_bins};
    segments.pop_back();
  }

  if (segments.size() <= 1 || max_gap_bins <= 0) {
    return segments;
  }

  std::vector<BinSegment> merged_segments;
  merged_segments.reserve(segments.size());
  merged_segments.push_back(segments.front());
  for (std::size_t i = 1; i < segments.size(); ++i) {
    BinSegment & current = merged_segments.back();
    const BinSegment & next = segments[i];
    const int gap_bins = next.start - current.end - 1;
    if (gap_bins <= max_gap_bins) {
      current.end = next.end;
    } else {
      merged_segments.push_back(next);
    }
  }

  if (
    merged_segments.size() > 1 &&
    (merged_segments.front().start + total_bins - merged_segments.back().end - 1) <= max_gap_bins)
  {
    BinSegment first = merged_segments.front();
    merged_segments.front() = {merged_segments.back().start, first.end + total_bins};
    merged_segments.pop_back();
  }

  return merged_segments;
}

bool contourTouchesBorder(const std::vector<cv::Point> & contour, const cv::Size & image_size)
{
  for (const auto & point : contour) {
    if (
      point.x <= 0 || point.y <= 0 ||
      point.x >= image_size.width - 1 || point.y >= image_size.height - 1)
    {
      return true;
    }
  }
  return false;
}

double contourCircularity(const std::vector<cv::Point> & contour, double area)
{
  const double perimeter = cv::arcLength(contour, true);
  if (perimeter <= 1e-6) {
    return 0.0;
  }
  return (4.0 * kPi * area) / (perimeter * perimeter);
}

double contourSolidity(const std::vector<cv::Point> & contour, double area)
{
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull);
  const double hull_area = cv::contourArea(hull);
  if (hull_area <= 1e-6) {
    return 0.0;
  }
  return area / hull_area;
}

double contourAspectRatio(const std::vector<cv::Point> & contour)
{
  const cv::Rect bbox = cv::boundingRect(contour);
  const int min_side = std::max(1, std::min(bbox.width, bbox.height));
  const int max_side = std::max(bbox.width, bbox.height);
  return static_cast<double>(max_side) / static_cast<double>(min_side);
}

cv::Mat normalizeBinaryMask(const cv::Mat & mask)
{
  cv::Mat normalized;
  cv::threshold(mask, normalized, 0, 255, cv::THRESH_BINARY);
  return normalized;
}

cv::Mat removeBorderTouchingComponents(const cv::Mat & mask)
{
  cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (const auto & contour : contours) {
    if (contourTouchesBorder(contour, mask.size())) {
      continue;
    }
    cv::drawContours(
      filtered,
      std::vector<std::vector<cv::Point>>{contour},
      -1,
      cv::Scalar(255),
      cv::FILLED);
  }
  return filtered;
}

cv::Mat createMaskOverlay(
  const cv::Mat & image,
  const cv::Mat & mask,
  const cv::Scalar & color,
  double alpha)
{
  cv::Mat overlay = image.clone();
  cv::Mat color_image(image.size(), CV_8UC3, color);
  cv::Mat blended;
  cv::addWeighted(image, 1.0 - alpha, color_image, alpha, 0.0, blended);
  blended.copyTo(overlay, mask);
  return overlay;
}

std::string describeRejectedCircleArc(const RejectedCircleArcCandidate & rejected_arc)
{
  std::string label;
  const auto append_reason = [&label](const char * reason) {
    if (!label.empty()) {
      label += "+";
    }
    label += reason;
  };

  if (rejected_arc.rejected_for_span) {
    append_reason("short");
  }
  if (rejected_arc.rejected_for_ring_ratio) {
    append_reason("low_ring");
  }
  if (rejected_arc.rejected_for_inner_ratio) {
    append_reason("high_inner");
  }
  if (rejected_arc.rejected_for_outer_ratio) {
    append_reason("high_outer");
  }

  if (label.empty()) {
    label = "rejected_arc";
  }
  return label;
}

bool computeMaskCentroid(const cv::Mat & mask, cv::Point & centroid)
{
  const cv::Moments moments = cv::moments(mask, true);
  if (moments.m00 > 1e-6) {
    centroid.x = static_cast<int>(std::round(moments.m10 / moments.m00));
    centroid.y = static_cast<int>(std::round(moments.m01 / moments.m00));
    return true;
  }

  std::vector<cv::Point> points;
  cv::findNonZero(mask, points);
  if (points.empty()) {
    return false;
  }

  centroid = points[points.size() / 2U];
  return true;
}

void drawDebugLabel(
  cv::Mat & image,
  const std::string & label,
  cv::Point anchor,
  const cv::Scalar & color)
{
  int baseline = 0;
  constexpr int kFontFace = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kFontScale = 0.45;
  constexpr int kTextThickness = 1;
  const cv::Size text_size =
    cv::getTextSize(label, kFontFace, kFontScale, kTextThickness, &baseline);

  const int min_x = 2;
  const int max_x = std::max(min_x, image.cols - text_size.width - 2);
  const int min_y = text_size.height + 2;
  const int max_y = std::max(min_y, image.rows - baseline - 2);
  anchor.x = std::clamp(anchor.x, min_x, max_x);
  anchor.y = std::clamp(anchor.y, min_y, max_y);

  cv::putText(
    image,
    label,
    anchor,
    kFontFace,
    kFontScale,
    cv::Scalar(0, 0, 0),
    kTextThickness + 2,
    cv::LINE_AA);
  cv::putText(
    image,
    label,
    anchor,
    kFontFace,
    kFontScale,
    color,
    kTextThickness,
    cv::LINE_AA);
}

}  // namespace

class BlackFeatureDetectorNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using ExactSyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;

  BlackFeatureDetectorNode()
  : Node("black_feature_detector_node")
  {
    declare_parameter<std::string>(
      "input_topic", "/black_feature_input_remap_node/image_remapped");
    declare_parameter<std::string>("black_mask_topic", "/white_line_hsv_white_node/black_mask");
    declare_parameter("enable_image_view", true);
    declare_parameter("publish_debug_image", true);
    declare_parameter("show_input_image", false);
    declare_parameter("show_input_black_mask", true);
    declare_parameter("show_binary_normalized_mask", true);
    declare_parameter("show_morph_open_mask", true);
    declare_parameter("show_morph_close_mask", true);
    declare_parameter("show_border_filtered_mask", true);
    declare_parameter("show_clean_mask", false);
    declare_parameter("show_dot_mask", false);
    declare_parameter("show_circle_mask", false);
    declare_parameter("show_circle_candidate_mask", false);
    declare_parameter("show_circle_rejected_component_mask", false);
    declare_parameter("show_circle_rejected_arc_mask", false);
    declare_parameter("show_circle_radius_rejected_mask", false);
    declare_parameter("show_circle_too_few_pixels_rejected_mask", false);
    declare_parameter("show_circle_radial_fit_rejected_mask", false);
    declare_parameter("show_circle_no_segments_rejected_mask", false);
    declare_parameter("show_black_final_mask", true);
    declare_parameter("show_debug_image", true);
    declare_parameter("display_max_width", 960);
    declare_parameter("display_max_height", 720);

    declare_parameter("open_kernel", 3);
    declare_parameter("close_kernel", 5);

    declare_parameter("dot_min_area_px", 8);
    declare_parameter("dot_max_area_px", 220);
    declare_parameter("dot_min_circularity", 0.45);
    declare_parameter("dot_min_solidity", 0.75);

    declare_parameter("circle_min_radius_px", 0.0);
    declare_parameter("circle_max_radius_px", 500.0);
    declare_parameter("circle_ring_width_px", 8.0);
    declare_parameter("circle_inner_guard_px", 8.0);
    declare_parameter("circle_outer_guard_px", 8.0);
    declare_parameter("circle_min_ring_black_ratio", 0.18);
    declare_parameter("circle_max_inner_black_ratio", 0.08);
    declare_parameter("circle_max_outer_black_ratio", 0.12);
    declare_parameter("circle_max_radial_residual_px", 5.5);
    declare_parameter("circle_min_arc_span_deg", 45.0);
    declare_parameter("circle_angle_bin_deg", 6.0);
    declare_parameter("circle_max_gap_bins", 2);

    syncImageViewState();

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto black_mask_topic = get_parameter("black_mask_topic").as_string();

    input_sub_.subscribe(this, input_topic, rmw_qos_profile_sensor_data);
    black_mask_sub_.subscribe(this, black_mask_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
      ExactSyncPolicy(10),
      input_sub_,
      black_mask_sub_);
    sync_->registerCallback(
      std::bind(
        &BlackFeatureDetectorNode::synchronizedCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    black_final_mask_pub_ = create_publisher<Image>("~/black_final_mask", 10);
    debug_image_pub_ = create_publisher<Image>("~/debug_image", 10);
    detections_pub_ = create_publisher<rcj_localization::msg::BlackFeatureDetectionArray>(
      "~/detections",
      10);

    RCLCPP_INFO(
      get_logger(),
      "black_feature_detector_node started. input_topic='%s', black_mask_topic='%s', "
      "publish_debug_image=%s, enable_image_view=%s",
      input_topic.c_str(),
      black_mask_topic.c_str(),
      get_parameter("publish_debug_image").as_bool() ? "true" : "false",
      enable_image_view_ ? "true" : "false");
  }

  ~BlackFeatureDetectorNode() override
  {
    destroyDebugWindows();
  }

private:
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

  void destroyDebugWindows()
  {
    syncWindow(kInputWindowName, false, input_window_created_);
    syncWindow(kInputBlackMaskWindowName, false, input_black_mask_window_created_);
    syncWindow(kBinaryNormalizedMaskWindowName, false, binary_normalized_mask_window_created_);
    syncWindow(kMorphOpenMaskWindowName, false, morph_open_mask_window_created_);
    syncWindow(kMorphCloseMaskWindowName, false, morph_close_mask_window_created_);
    syncWindow(kBorderFilteredMaskWindowName, false, border_filtered_mask_window_created_);
    syncWindow(kCleanMaskWindowName, false, clean_mask_window_created_);
    syncWindow(kDotMaskWindowName, false, dot_mask_window_created_);
    syncWindow(kCircleMaskWindowName, false, circle_mask_window_created_);
    syncWindow(
      kCircleCandidateMaskWindowName,
      false,
      circle_candidate_mask_window_created_);
    syncWindow(
      kCircleRejectedComponentMaskWindowName,
      false,
      circle_rejected_component_mask_window_created_);
    syncWindow(
      kCircleRejectedArcMaskWindowName,
      false,
      circle_rejected_arc_mask_window_created_);
    syncWindow(
      kCircleRadiusRejectedMaskWindowName,
      false,
      circle_radius_rejected_mask_window_created_);
    syncWindow(
      kCircleTooFewPixelsRejectedMaskWindowName,
      false,
      circle_too_few_pixels_rejected_mask_window_created_);
    syncWindow(
      kCircleRadialFitRejectedMaskWindowName,
      false,
      circle_radial_fit_rejected_mask_window_created_);
    syncWindow(
      kCircleNoSegmentsRejectedMaskWindowName,
      false,
      circle_no_segments_rejected_mask_window_created_);
    syncWindow(kFinalMaskWindowName, false, final_mask_window_created_);
    syncWindow(kDebugWindowName, false, debug_window_created_);
  }

  void syncImageViewState()
  {
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    show_input_image_ = get_parameter("show_input_image").as_bool();
    show_input_black_mask_ = get_parameter("show_input_black_mask").as_bool();
    show_binary_normalized_mask_ = get_parameter("show_binary_normalized_mask").as_bool();
    show_morph_open_mask_ = get_parameter("show_morph_open_mask").as_bool();
    show_morph_close_mask_ = get_parameter("show_morph_close_mask").as_bool();
    show_border_filtered_mask_ = get_parameter("show_border_filtered_mask").as_bool();
    show_clean_mask_ = get_parameter("show_clean_mask").as_bool();
    show_dot_mask_ = get_parameter("show_dot_mask").as_bool();
    show_circle_mask_ = get_parameter("show_circle_mask").as_bool();
    show_circle_candidate_mask_ = get_parameter("show_circle_candidate_mask").as_bool();
    show_circle_rejected_component_mask_ =
      get_parameter("show_circle_rejected_component_mask").as_bool();
    show_circle_rejected_arc_mask_ = get_parameter("show_circle_rejected_arc_mask").as_bool();
    show_circle_radius_rejected_mask_ =
      get_parameter("show_circle_radius_rejected_mask").as_bool();
    show_circle_too_few_pixels_rejected_mask_ =
      get_parameter("show_circle_too_few_pixels_rejected_mask").as_bool();
    show_circle_radial_fit_rejected_mask_ =
      get_parameter("show_circle_radial_fit_rejected_mask").as_bool();
    show_circle_no_segments_rejected_mask_ =
      get_parameter("show_circle_no_segments_rejected_mask").as_bool();
    show_black_final_mask_ = get_parameter("show_black_final_mask").as_bool();
    show_debug_image_ = get_parameter("show_debug_image").as_bool();
    display_max_width_ =
      std::max(1, static_cast<int>(get_parameter("display_max_width").as_int()));
    display_max_height_ =
      std::max(1, static_cast<int>(get_parameter("display_max_height").as_int()));

    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (enable_image_view_ && !display_available) {
      if (!headless_warned_) {
        RCLCPP_WARN(
          get_logger(),
          "enable_image_view=true but no DISPLAY/WAYLAND_DISPLAY is available; "
          "disabling OpenCV image view for this process.");
        headless_warned_ = true;
      }
      enable_image_view_ = false;
    }

    syncWindow(kInputWindowName, enable_image_view_ && show_input_image_, input_window_created_);
    syncWindow(
      kInputBlackMaskWindowName,
      enable_image_view_ && show_input_black_mask_,
      input_black_mask_window_created_);
    syncWindow(
      kBinaryNormalizedMaskWindowName,
      enable_image_view_ && show_binary_normalized_mask_,
      binary_normalized_mask_window_created_);
    syncWindow(
      kMorphOpenMaskWindowName,
      enable_image_view_ && show_morph_open_mask_,
      morph_open_mask_window_created_);
    syncWindow(
      kMorphCloseMaskWindowName,
      enable_image_view_ && show_morph_close_mask_,
      morph_close_mask_window_created_);
    syncWindow(
      kBorderFilteredMaskWindowName,
      enable_image_view_ && show_border_filtered_mask_,
      border_filtered_mask_window_created_);
    syncWindow(
      kCleanMaskWindowName,
      enable_image_view_ && show_clean_mask_,
      clean_mask_window_created_);
    syncWindow(kDotMaskWindowName, enable_image_view_ && show_dot_mask_, dot_mask_window_created_);
    syncWindow(
      kCircleMaskWindowName,
      enable_image_view_ && show_circle_mask_,
      circle_mask_window_created_);
    syncWindow(
      kCircleCandidateMaskWindowName,
      enable_image_view_ && show_circle_candidate_mask_,
      circle_candidate_mask_window_created_);
    syncWindow(
      kCircleRejectedComponentMaskWindowName,
      enable_image_view_ && show_circle_rejected_component_mask_,
      circle_rejected_component_mask_window_created_);
    syncWindow(
      kCircleRejectedArcMaskWindowName,
      enable_image_view_ && show_circle_rejected_arc_mask_,
      circle_rejected_arc_mask_window_created_);
    syncWindow(
      kCircleRadiusRejectedMaskWindowName,
      enable_image_view_ && show_circle_radius_rejected_mask_,
      circle_radius_rejected_mask_window_created_);
    syncWindow(
      kCircleTooFewPixelsRejectedMaskWindowName,
      enable_image_view_ && show_circle_too_few_pixels_rejected_mask_,
      circle_too_few_pixels_rejected_mask_window_created_);
    syncWindow(
      kCircleRadialFitRejectedMaskWindowName,
      enable_image_view_ && show_circle_radial_fit_rejected_mask_,
      circle_radial_fit_rejected_mask_window_created_);
    syncWindow(
      kCircleNoSegmentsRejectedMaskWindowName,
      enable_image_view_ && show_circle_no_segments_rejected_mask_,
      circle_no_segments_rejected_mask_window_created_);
    syncWindow(
      kFinalMaskWindowName,
      enable_image_view_ && show_black_final_mask_,
      final_mask_window_created_);
    syncWindow(kDebugWindowName, enable_image_view_ && show_debug_image_, debug_window_created_);
  }

  bool isDotCandidate(const std::vector<cv::Point> & contour) const
  {
    const double area = cv::contourArea(contour);
    const int min_area = static_cast<int>(get_parameter("dot_min_area_px").as_int());
    const int max_area = static_cast<int>(get_parameter("dot_max_area_px").as_int());
    if (area < static_cast<double>(min_area) || area > static_cast<double>(max_area)) {
      return false;
    }

    const double circularity = contourCircularity(contour, area);
    const double solidity = contourSolidity(contour, area);
    const double aspect_ratio = contourAspectRatio(contour);

    return circularity >= get_parameter("dot_min_circularity").as_double() &&
           solidity >= get_parameter("dot_min_solidity").as_double() &&
           aspect_ratio <= 1.8;
  }

  CircleEvaluationResult evaluateCircleContour(
    const std::vector<cv::Point> & contour,
    const cv::Mat & component_mask,
    const cv::Mat & clean_mask) const
  {
    CircleEvaluationResult result;
    cv::minEnclosingCircle(contour, result.center, result.radius);

    const double min_radius = get_parameter("circle_min_radius_px").as_double();
    const double max_radius = get_parameter("circle_max_radius_px").as_double();
    if (result.radius < min_radius || result.radius > max_radius) {
      result.reject_reason = CircleRejectReason::kRadiusOutOfRange;
      return result;
    }

    std::vector<cv::Point> component_points;
    cv::findNonZero(component_mask, component_points);
    if (component_points.size() < 10U) {
      result.reject_reason = CircleRejectReason::kTooFewPixels;
      return result;
    }

    std::vector<float> radial_residuals;
    radial_residuals.reserve(component_points.size());
    const double ring_half_width =
      0.5 * std::max(1.0, get_parameter("circle_ring_width_px").as_double());
    const double residual_limit =
      std::max(0.5, get_parameter("circle_max_radial_residual_px").as_double());
    const double angle_bin_deg =
      std::clamp(get_parameter("circle_angle_bin_deg").as_double(), 1.0, 45.0);
    const int total_bins =
      std::max(8, static_cast<int>(std::ceil(360.0 / angle_bin_deg)));
    const int max_gap_bins =
      std::max(0, static_cast<int>(get_parameter("circle_max_gap_bins").as_int()));
    std::vector<int> bin_counts(static_cast<std::size_t>(total_bins), 0);

    for (const auto & point : component_points) {
      const double dx = static_cast<double>(point.x) - static_cast<double>(result.center.x);
      const double dy = static_cast<double>(point.y) - static_cast<double>(result.center.y);
      const double distance = std::hypot(dx, dy);
      const float residual =
        static_cast<float>(std::abs(distance - static_cast<double>(result.radius)));
      radial_residuals.push_back(residual);

      if (residual <= ring_half_width + 1.0) {
        const double angle_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / kPi);
        int bin_idx = static_cast<int>(std::floor(angle_deg / angle_bin_deg));
        bin_idx = std::clamp(bin_idx, 0, total_bins - 1);
        ++bin_counts[static_cast<std::size_t>(bin_idx)];
      }
    }

    if (computeMedian(radial_residuals) > residual_limit) {
      result.reject_reason = CircleRejectReason::kBadRadialFit;
      return result;
    }

    const double min_arc_span_deg =
      std::clamp(get_parameter("circle_min_arc_span_deg").as_double(), angle_bin_deg, 360.0);
    const double inner_guard =
      std::max(0.0, get_parameter("circle_inner_guard_px").as_double());
    const double outer_guard =
      std::max(0.0, get_parameter("circle_outer_guard_px").as_double());
    const double min_ring_ratio =
      std::clamp(get_parameter("circle_min_ring_black_ratio").as_double(), 0.0, 1.0);
    const double max_inner_ratio =
      std::clamp(get_parameter("circle_max_inner_black_ratio").as_double(), 0.0, 1.0);
    const double max_outer_ratio =
      std::clamp(get_parameter("circle_max_outer_black_ratio").as_double(), 0.0, 1.0);

    const auto segments = collectOccupiedSegments(bin_counts, max_gap_bins);
    if (segments.empty()) {
      result.reject_reason = CircleRejectReason::kNoSegments;
      return result;
    }

    const double roi_radius =
      static_cast<double>(result.radius) + ring_half_width + std::max(inner_guard, outer_guard) +
      3.0;
    const int x_min = std::max(
      0,
      static_cast<int>(std::floor(static_cast<double>(result.center.x) - roi_radius)));
    const int x_max = std::min(
      component_mask.cols - 1,
      static_cast<int>(std::ceil(static_cast<double>(result.center.x) + roi_radius)));
    const int y_min = std::max(
      0,
      static_cast<int>(std::floor(static_cast<double>(result.center.y) - roi_radius)));
    const int y_max = std::min(
      component_mask.rows - 1,
      static_cast<int>(std::ceil(static_cast<double>(result.center.y) + roi_radius)));

    double ring_ratio_sum = 0.0;
    for (const auto & segment : segments) {
      const double segment_start_deg = static_cast<double>(segment.start) * angle_bin_deg;
      const double segment_end_deg = static_cast<double>(segment.end + 1) * angle_bin_deg;
      const double segment_span_deg =
        static_cast<double>(segment.end - segment.start + 1) * angle_bin_deg;

      int ring_total = 0;
      int ring_black = 0;
      int inner_total = 0;
      int inner_black = 0;
      int outer_total = 0;
      int outer_black = 0;
      cv::Mat arc_mask = cv::Mat::zeros(component_mask.size(), CV_8UC1);

      for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
          const double dx = static_cast<double>(x) - static_cast<double>(result.center.x);
          const double dy = static_cast<double>(y) - static_cast<double>(result.center.y);
          const double distance = std::hypot(dx, dy);
          const double angle_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / kPi);
          if (!angleInRange(angle_deg, segment_start_deg, segment_end_deg)) {
            continue;
          }

          const double radial_delta =
            std::abs(distance - static_cast<double>(result.radius));
          if (radial_delta <= ring_half_width) {
            ++ring_total;
            if (component_mask.at<uchar>(y, x) != 0) {
              ++ring_black;
            }
          } else if (
            distance >= static_cast<double>(result.radius) - ring_half_width - inner_guard &&
            distance < static_cast<double>(result.radius) - ring_half_width)
          {
            ++inner_total;
            if (clean_mask.at<uchar>(y, x) != 0) {
              ++inner_black;
            }
          } else if (
            distance > static_cast<double>(result.radius) + ring_half_width &&
            distance <= static_cast<double>(result.radius) + ring_half_width + outer_guard)
          {
            ++outer_total;
            if (clean_mask.at<uchar>(y, x) != 0) {
              ++outer_black;
            }
          }

          if (
            component_mask.at<uchar>(y, x) != 0 &&
            radial_delta <= ring_half_width + residual_limit)
          {
            arc_mask.at<uchar>(y, x) = 255;
          }
        }
      }

      if (ring_total == 0 || cv::countNonZero(arc_mask) == 0) {
        continue;
      }

      if (segment_span_deg < min_arc_span_deg) {
        result.rejected_arcs.push_back(
          {arc_mask, segment_start_deg, segment_end_deg, true, false, false, false});
        continue;
      }

      const double ring_ratio =
        static_cast<double>(ring_black) / static_cast<double>(ring_total);
      const double inner_ratio =
        inner_total > 0
        ? static_cast<double>(inner_black) / static_cast<double>(inner_total)
        : 0.0;
      const double outer_ratio =
        outer_total > 0
        ? static_cast<double>(outer_black) / static_cast<double>(outer_total)
        : 0.0;

      const bool rejected_for_ring_ratio = ring_ratio < min_ring_ratio;
      const bool rejected_for_inner_ratio = inner_ratio > max_inner_ratio;
      const bool rejected_for_outer_ratio = outer_ratio > max_outer_ratio;
      if (rejected_for_ring_ratio || rejected_for_inner_ratio || rejected_for_outer_ratio) {
        result.rejected_arcs.push_back(
          {
            arc_mask,
            segment_start_deg,
            segment_end_deg,
            false,
            rejected_for_ring_ratio,
            rejected_for_inner_ratio,
            rejected_for_outer_ratio,
          });
        continue;
      }

      result.arcs.push_back({arc_mask, segment_start_deg, segment_end_deg, ring_ratio});
      result.coverage_deg += segment_span_deg;
      ring_ratio_sum += ring_ratio;
    }

    result.accepted = !result.arcs.empty();
    if (result.accepted) {
      result.coverage_deg = std::min(360.0, result.coverage_deg);
      result.mean_ring_ratio = ring_ratio_sum / static_cast<double>(result.arcs.size());
    }
    return result;
  }

  rcj_localization::msg::BlackFeatureDetection makeDotDetection(
    const std::vector<cv::Point> & contour) const
  {
    rcj_localization::msg::BlackFeatureDetection detection;
    detection.type = rcj_localization::msg::BlackFeatureDetection::TYPE_DOT;

    const double area = cv::contourArea(contour);
    const double circularity = contourCircularity(contour, area);
    const double solidity = contourSolidity(contour, area);
    const double aspect_ratio = contourAspectRatio(contour);
    const double aspect_score = clamp01(1.8 / std::max(1.0, aspect_ratio));
    const double score =
      clamp01((clamp01(circularity) + clamp01(solidity) + aspect_score) / 3.0);

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(contour, center, radius);

    detection.score = static_cast<float>(score);
    detection.center_px.x = static_cast<double>(center.x);
    detection.center_px.y = static_cast<double>(center.y);
    detection.center_px.z = 0.0;
    detection.radius_px = radius;
    detection.area_px = static_cast<float>(area);
    detection.coverage_deg = 360.0f;
    detection.arc_count = 0U;
    return detection;
  }

  rcj_localization::msg::BlackFeatureDetection makeCircleDetection(
    const CircleEvaluationResult & circle_eval,
    double area) const
  {
    rcj_localization::msg::BlackFeatureDetection detection;
    detection.type = rcj_localization::msg::BlackFeatureDetection::TYPE_CIRCLE;

    const double coverage_score = clamp01(circle_eval.coverage_deg / 360.0);
    const double score =
      clamp01((coverage_score + clamp01(circle_eval.mean_ring_ratio)) * 0.5);

    detection.score = static_cast<float>(score);
    detection.center_px.x = static_cast<double>(circle_eval.center.x);
    detection.center_px.y = static_cast<double>(circle_eval.center.y);
    detection.center_px.z = 0.0;
    detection.radius_px = circle_eval.radius;
    detection.area_px = static_cast<float>(area);
    detection.coverage_deg = static_cast<float>(circle_eval.coverage_deg);
    detection.arc_count = static_cast<std::uint32_t>(circle_eval.arcs.size());
    return detection;
  }

  cv::Mat createFinalOverlayDebugImage(
    const cv::Mat & frame,
    const cv::Mat & black_final_mask,
    const std::vector<RejectedCircleArcCandidate> & rejected_arcs) const
  {
    cv::Mat debug_image =
      createMaskOverlay(frame, black_final_mask, cv::Scalar(0, 0, 255), 0.45);

    for (const auto & rejected_arc : rejected_arcs) {
      debug_image =
        createMaskOverlay(debug_image, rejected_arc.mask, cv::Scalar(0, 165, 255), 0.45);

      cv::Point centroid;
      if (!computeMaskCentroid(rejected_arc.mask, centroid)) {
        continue;
      }

      drawDebugLabel(
        debug_image,
        describeRejectedCircleArc(rejected_arc),
        centroid + cv::Point(4, -4),
        cv::Scalar(0, 255, 255));
    }

    return debug_image;
  }

  void showDebugWindows(
    const cv::Mat & frame,
    const cv::Mat & input_black_mask,
    const cv::Mat & binary_normalized_mask,
    const cv::Mat & morph_open_mask,
    const cv::Mat & morph_close_mask,
    const cv::Mat & border_filtered_mask,
    const cv::Mat & clean_mask,
    const cv::Mat & dot_mask,
    const cv::Mat & circle_mask,
    const cv::Mat & circle_candidate_mask,
    const cv::Mat & circle_rejected_component_mask,
    const cv::Mat & circle_rejected_arc_mask,
    const cv::Mat & circle_radius_rejected_mask,
    const cv::Mat & circle_too_few_pixels_rejected_mask,
    const cv::Mat & circle_radial_fit_rejected_mask,
    const cv::Mat & circle_no_segments_rejected_mask,
    const cv::Mat & black_final_mask,
    const cv::Mat & debug_image) const
  {
    bool displayed_any_window = false;

    if (input_window_created_) {
      cv::imshow(kInputWindowName, frame);
      resizeWindowToFitImage(kInputWindowName, frame, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (input_black_mask_window_created_) {
      cv::imshow(kInputBlackMaskWindowName, input_black_mask);
      resizeWindowToFitImage(
        kInputBlackMaskWindowName,
        input_black_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (binary_normalized_mask_window_created_) {
      cv::imshow(kBinaryNormalizedMaskWindowName, binary_normalized_mask);
      resizeWindowToFitImage(
        kBinaryNormalizedMaskWindowName,
        binary_normalized_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (morph_open_mask_window_created_) {
      cv::imshow(kMorphOpenMaskWindowName, morph_open_mask);
      resizeWindowToFitImage(
        kMorphOpenMaskWindowName,
        morph_open_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (morph_close_mask_window_created_) {
      cv::imshow(kMorphCloseMaskWindowName, morph_close_mask);
      resizeWindowToFitImage(
        kMorphCloseMaskWindowName,
        morph_close_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (border_filtered_mask_window_created_) {
      cv::imshow(kBorderFilteredMaskWindowName, border_filtered_mask);
      resizeWindowToFitImage(
        kBorderFilteredMaskWindowName,
        border_filtered_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (clean_mask_window_created_) {
      cv::imshow(kCleanMaskWindowName, clean_mask);
      resizeWindowToFitImage(
        kCleanMaskWindowName,
        clean_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (dot_mask_window_created_) {
      cv::imshow(kDotMaskWindowName, dot_mask);
      resizeWindowToFitImage(kDotMaskWindowName, dot_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (circle_mask_window_created_) {
      cv::imshow(kCircleMaskWindowName, circle_mask);
      resizeWindowToFitImage(
        kCircleMaskWindowName,
        circle_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_candidate_mask_window_created_) {
      cv::imshow(kCircleCandidateMaskWindowName, circle_candidate_mask);
      resizeWindowToFitImage(
        kCircleCandidateMaskWindowName,
        circle_candidate_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_rejected_component_mask_window_created_) {
      cv::imshow(kCircleRejectedComponentMaskWindowName, circle_rejected_component_mask);
      resizeWindowToFitImage(
        kCircleRejectedComponentMaskWindowName,
        circle_rejected_component_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_rejected_arc_mask_window_created_) {
      cv::imshow(kCircleRejectedArcMaskWindowName, circle_rejected_arc_mask);
      resizeWindowToFitImage(
        kCircleRejectedArcMaskWindowName,
        circle_rejected_arc_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_radius_rejected_mask_window_created_) {
      cv::imshow(kCircleRadiusRejectedMaskWindowName, circle_radius_rejected_mask);
      resizeWindowToFitImage(
        kCircleRadiusRejectedMaskWindowName,
        circle_radius_rejected_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_too_few_pixels_rejected_mask_window_created_) {
      cv::imshow(
        kCircleTooFewPixelsRejectedMaskWindowName,
        circle_too_few_pixels_rejected_mask);
      resizeWindowToFitImage(
        kCircleTooFewPixelsRejectedMaskWindowName,
        circle_too_few_pixels_rejected_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_radial_fit_rejected_mask_window_created_) {
      cv::imshow(kCircleRadialFitRejectedMaskWindowName, circle_radial_fit_rejected_mask);
      resizeWindowToFitImage(
        kCircleRadialFitRejectedMaskWindowName,
        circle_radial_fit_rejected_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (circle_no_segments_rejected_mask_window_created_) {
      cv::imshow(kCircleNoSegmentsRejectedMaskWindowName, circle_no_segments_rejected_mask);
      resizeWindowToFitImage(
        kCircleNoSegmentsRejectedMaskWindowName,
        circle_no_segments_rejected_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (final_mask_window_created_) {
      cv::imshow(kFinalMaskWindowName, black_final_mask);
      resizeWindowToFitImage(
        kFinalMaskWindowName,
        black_final_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (debug_window_created_ && !debug_image.empty()) {
      cv::imshow(kDebugWindowName, debug_image);
      resizeWindowToFitImage(kDebugWindowName, debug_image, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }

    if (displayed_any_window) {
      cv::waitKey(1);
    }
  }

  void synchronizedCallback(
    const Image::ConstSharedPtr & image_msg,
    const Image::ConstSharedPtr & black_mask_msg)
  {
    syncImageViewState();

    cv::Mat frame;
    cv::Mat raw_black_mask;
    try {
      frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      raw_black_mask = cv_bridge::toCvCopy(black_mask_msg, "mono8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge failed while reading synchronized black feature inputs: %s",
        e.what());
      return;
    }

    if (frame.size() != raw_black_mask.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Black feature detector received mismatched image sizes.");
      return;
    }

    const cv::Mat input_black_mask = normalizeBinaryMask(raw_black_mask);
    const int open_kernel =
      makeOddKernel(static_cast<int>(get_parameter("open_kernel").as_int()), 1);
    const int close_kernel =
      makeOddKernel(static_cast<int>(get_parameter("close_kernel").as_int()), 1);

    const cv::Mat binary_normalized_mask = input_black_mask.clone();
    cv::Mat morph_open_mask = binary_normalized_mask.clone();
    if (open_kernel > 1) {
      const cv::Mat open_element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(open_kernel, open_kernel));
      cv::morphologyEx(morph_open_mask, morph_open_mask, cv::MORPH_OPEN, open_element);
    }

    cv::Mat morph_close_mask = morph_open_mask.clone();
    if (close_kernel > 1) {
      const cv::Mat close_element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(close_kernel, close_kernel));
      cv::morphologyEx(morph_close_mask, morph_close_mask, cv::MORPH_CLOSE, close_element);
    }

    cv::Mat border_filtered_mask = removeBorderTouchingComponents(morph_close_mask);
    const cv::Mat clean_mask = border_filtered_mask.clone();

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(clean_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat dot_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_candidate_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_rejected_component_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_rejected_arc_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_radius_rejected_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_too_few_pixels_rejected_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_radial_fit_rejected_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat circle_no_segments_rejected_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<RejectedCircleArcCandidate> rejected_arc_debug_entries;
    rcj_localization::msg::BlackFeatureDetectionArray detections_msg;
    detections_msg.header = image_msg->header;

    for (const auto & contour : contours) {
      if (contour.empty()) {
        continue;
      }

      const double contour_area = cv::contourArea(contour);
      if (contour_area <= 0.0) {
        continue;
      }

      if (isDotCandidate(contour)) {
        cv::drawContours(
          dot_mask,
          std::vector<std::vector<cv::Point>>{contour},
          -1,
          cv::Scalar(255),
          cv::FILLED);
        detections_msg.detections.push_back(makeDotDetection(contour));
        continue;
      }

      cv::Mat component_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
      cv::drawContours(
        component_mask,
        std::vector<std::vector<cv::Point>>{contour},
        -1,
        cv::Scalar(255),
        cv::FILLED);
      cv::bitwise_or(circle_candidate_mask, component_mask, circle_candidate_mask);

      const CircleEvaluationResult circle_eval =
        evaluateCircleContour(contour, component_mask, clean_mask);
      for (const auto & rejected_arc : circle_eval.rejected_arcs) {
        rejected_arc_debug_entries.push_back(rejected_arc);
        cv::bitwise_or(circle_rejected_arc_mask, rejected_arc.mask, circle_rejected_arc_mask);
      }
      if (!circle_eval.accepted) {
        cv::bitwise_or(
          circle_rejected_component_mask,
          component_mask,
          circle_rejected_component_mask);
        switch (circle_eval.reject_reason) {
          case CircleRejectReason::kRadiusOutOfRange:
            cv::bitwise_or(
              circle_radius_rejected_mask,
              component_mask,
              circle_radius_rejected_mask);
            break;
          case CircleRejectReason::kTooFewPixels:
            cv::bitwise_or(
              circle_too_few_pixels_rejected_mask,
              component_mask,
              circle_too_few_pixels_rejected_mask);
            break;
          case CircleRejectReason::kBadRadialFit:
            cv::bitwise_or(
              circle_radial_fit_rejected_mask,
              component_mask,
              circle_radial_fit_rejected_mask);
            break;
          case CircleRejectReason::kNoSegments:
            cv::bitwise_or(
              circle_no_segments_rejected_mask,
              component_mask,
              circle_no_segments_rejected_mask);
            break;
          case CircleRejectReason::kNone:
            break;
        }
        continue;
      }

      for (const auto & arc : circle_eval.arcs) {
        cv::bitwise_or(circle_mask, arc.mask, circle_mask);
      }
      detections_msg.detections.push_back(makeCircleDetection(circle_eval, contour_area));
    }

    cv::Mat black_final_mask;
    cv::bitwise_or(dot_mask, circle_mask, black_final_mask);

    black_final_mask_pub_->publish(
      *cv_bridge::CvImage(image_msg->header, "mono8", black_final_mask).toImageMsg());
    detections_pub_->publish(detections_msg);

    cv::Mat debug_image;
    const bool debug_image_needed =
      publish_debug_image_ || (enable_image_view_ && show_debug_image_);
    if (debug_image_needed) {
      debug_image =
        createFinalOverlayDebugImage(frame, black_final_mask, rejected_arc_debug_entries);
      if (publish_debug_image_) {
        debug_image_pub_->publish(
          *cv_bridge::CvImage(image_msg->header, "bgr8", debug_image).toImageMsg());
      }
    }

    if (enable_image_view_) {
      showDebugWindows(
        frame,
        input_black_mask,
        binary_normalized_mask,
        morph_open_mask,
        morph_close_mask,
        border_filtered_mask,
        clean_mask,
        dot_mask,
        circle_mask,
        circle_candidate_mask,
        circle_rejected_component_mask,
        circle_rejected_arc_mask,
        circle_radius_rejected_mask,
        circle_too_few_pixels_rejected_mask,
        circle_radial_fit_rejected_mask,
        circle_no_segments_rejected_mask,
        black_final_mask,
        debug_image);
    }
  }

  message_filters::Subscriber<Image> input_sub_;
  message_filters::Subscriber<Image> black_mask_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_;

  rclcpp::Publisher<Image>::SharedPtr black_final_mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<rcj_localization::msg::BlackFeatureDetectionArray>::SharedPtr detections_pub_;

  bool enable_image_view_ = false;
  bool publish_debug_image_ = true;
  bool show_input_image_ = false;
  bool show_input_black_mask_ = true;
  bool show_binary_normalized_mask_ = false;
  bool show_morph_open_mask_ = false;
  bool show_morph_close_mask_ = false;
  bool show_border_filtered_mask_ = false;
  bool show_clean_mask_ = true;
  bool show_dot_mask_ = true;
  bool show_circle_mask_ = true;
  bool show_circle_candidate_mask_ = false;
  bool show_circle_rejected_component_mask_ = false;
  bool show_circle_rejected_arc_mask_ = false;
  bool show_circle_radius_rejected_mask_ = false;
  bool show_circle_too_few_pixels_rejected_mask_ = false;
  bool show_circle_radial_fit_rejected_mask_ = false;
  bool show_circle_no_segments_rejected_mask_ = false;
  bool show_black_final_mask_ = true;
  bool show_debug_image_ = true;

  bool input_window_created_ = false;
  bool input_black_mask_window_created_ = false;
  bool binary_normalized_mask_window_created_ = false;
  bool morph_open_mask_window_created_ = false;
  bool morph_close_mask_window_created_ = false;
  bool border_filtered_mask_window_created_ = false;
  bool clean_mask_window_created_ = false;
  bool dot_mask_window_created_ = false;
  bool circle_mask_window_created_ = false;
  bool circle_candidate_mask_window_created_ = false;
  bool circle_rejected_component_mask_window_created_ = false;
  bool circle_rejected_arc_mask_window_created_ = false;
  bool circle_radius_rejected_mask_window_created_ = false;
  bool circle_too_few_pixels_rejected_mask_window_created_ = false;
  bool circle_radial_fit_rejected_mask_window_created_ = false;
  bool circle_no_segments_rejected_mask_window_created_ = false;
  bool final_mask_window_created_ = false;
  bool debug_window_created_ = false;
  bool headless_warned_ = false;

  int display_max_width_ = 960;
  int display_max_height_ = 720;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlackFeatureDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
