#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kHueMax = 179;
constexpr int kByteMax = 255;
constexpr char kControlsWindowName[] = "HSV White Controls";
constexpr char kInputWindowName[] = "HSV White Input";
constexpr char kMaskWindowName[] = "HSV White Mask";
constexpr char kGreenMaskWindowName[] = "HSV Green Mask";
constexpr char kBlackMaskWindowName[] = "HSV Black Mask";
constexpr char kNoiseMaskWindowName[] = "HSV Noise Mask";
constexpr char kOverlayWindowName[] = "HSV White Overlay";

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

long long elapsedUs(const TimePoint & start, const TimePoint & end)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

enum class HsvTimingStage : std::size_t
{
  RuntimeSync = 0,
  CvBridgeConvert,
  HsvConvert,
  WhiteThreshold,
  BlackThreshold,
  GreenThreshold,
  ContextResolve,
  PublishOutputs,
  GuiDisplay,
  UnaccountedOverhead,
  CallbackTotal,
  Count
};

constexpr std::array<const char *, static_cast<std::size_t>(HsvTimingStage::Count)>
  kHsvTimingLabels = {
    "runtime_sync",
    "cv_bridge",
    "hsv_convert",
    "white_threshold",
    "black_threshold",
    "green_threshold",
    "context_resolve",
    "publish_outputs",
    "gui_display",
    "unaccounted_overhead",
    "callback_total",
  };

using HsvTimingArray =
  std::array<long long, static_cast<std::size_t>(HsvTimingStage::Count)>;

struct HsvFrameTiming
{
  HsvTimingArray stage_us{};
};

void recordStageDuration(
  HsvTimingArray & target,
  HsvTimingStage stage,
  const TimePoint & start,
  const TimePoint & end)
{
  target[static_cast<std::size_t>(stage)] = elapsedUs(start, end);
}

long long sumTimingStagesExcludingCallback(const HsvTimingArray & timing)
{
  long long total = 0;
  for (std::size_t i = 0; i < timing.size(); ++i) {
    if (i == static_cast<std::size_t>(HsvTimingStage::CallbackTotal)) {
      continue;
    }
    if (i == static_cast<std::size_t>(HsvTimingStage::UnaccountedOverhead)) {
      continue;
    }
    total += timing[i];
  }
  return total;
}

void appendTimingTable(
  std::ostringstream & oss,
  const HsvFrameTiming & timing,
  const HsvTimingArray & interval_totals,
  std::size_t interval_frame_count)
{
  const double callback_average_ms =
    interval_frame_count > 0
      ? static_cast<double>(
      interval_totals[static_cast<std::size_t>(HsvTimingStage::CallbackTotal)]) /
      static_cast<double>(interval_frame_count) / 1000.0
      : 0.0;

  oss << std::left << std::setw(20) << "stage"
      << std::right << std::setw(12) << "current ms"
      << std::setw(14) << "avg ms"
      << std::setw(12) << "ratio %" << '\n';

  for (std::size_t i = 0; i < kHsvTimingLabels.size(); ++i) {
    const double current_ms = static_cast<double>(timing.stage_us[i]) / 1000.0;
    const double interval_average_ms =
      interval_frame_count > 0
        ? static_cast<double>(interval_totals[i]) / static_cast<double>(interval_frame_count) /
        1000.0
        : 0.0;
    const double ratio =
      callback_average_ms > 0.0 ? (interval_average_ms / callback_average_ms) * 100.0 : 0.0;

    oss << std::left << std::setw(20) << kHsvTimingLabels[i]
        << std::right << std::setw(12) << std::fixed << std::setprecision(3) << current_ms
        << std::setw(14) << std::fixed << std::setprecision(3) << interval_average_ms
        << std::setw(12) << std::fixed << std::setprecision(1) << ratio << '\n';
  }
}

cv::Mat takeFromRemaining(const cv::Mat & candidate, cv::Mat & remaining)
{
  cv::Mat assigned;
  cv::bitwise_and(candidate, remaining, assigned);

  cv::Mat assigned_inv;
  cv::bitwise_not(assigned, assigned_inv);
  cv::bitwise_and(remaining, assigned_inv, remaining);
  return assigned;
}

int clampHue(int value)
{
  return std::clamp(value, 0, kHueMax);
}

int clampByte(int value)
{
  return std::clamp(value, 0, kByteMax);
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
    std::max(1, static_cast<int>(image_size.width * scale)),
    std::max(1, static_cast<int>(image_size.height * scale)));
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

}  // namespace

class WhiteLineHsvWhiteNode : public rclcpp::Node
{
public:
  WhiteLineHsvWhiteNode()
  : Node("white_line_hsv_white_node")
  {
    declare_parameter<std::string>(
      "input_topic", "/white_line_hsv_input_remap_node/image_remapped");
    declare_parameter<std::string>("robot_mask_topic", "");
    declare_parameter("white_h_min", 0);
    declare_parameter("white_h_max", kHueMax);
    declare_parameter("white_s_max", 60);
    declare_parameter("white_v_min", 170);
    declare_parameter("black_v_max", 70);
    declare_parameter("green_h_min", 35);
    declare_parameter("green_h_max", 95);
    declare_parameter("green_s_min", 40);
    declare_parameter("green_v_min", 40);
    declare_parameter("enable_timing_log", true);
    declare_parameter("timing_log_interval", 30);
    declare_parameter("enable_image_view", false);
    declare_parameter("enable_controls_window", false);
    declare_parameter("show_input_image", true);
    declare_parameter("show_white_mask", true);
    declare_parameter("show_green_mask", false);
    declare_parameter("show_black_mask", false);
    declare_parameter("show_noise_mask", false);
    declare_parameter("show_overlay_image", true);
    declare_parameter("publish_debug_images", false);
    declare_parameter("publish_input_image", true);
    declare_parameter("publish_white_mask", true);
    declare_parameter("publish_green_mask", true);
    declare_parameter("publish_black_mask", true);
    declare_parameter("publish_noise_mask", true);
    declare_parameter("publish_overlay_image", true);
    declare_parameter("display_max_width", 960);
    declare_parameter("display_max_height", 720);

    loadThresholdParameters();
    loadRuntimeParameters();
    syncImageViewState();

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto robot_mask_topic = get_parameter("robot_mask_topic").as_string();
    robot_mask_enabled_ = !robot_mask_topic.empty();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&WhiteLineHsvWhiteNode::imageCallback, this, std::placeholders::_1));
    if (!robot_mask_topic.empty()) {
      robot_mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
        robot_mask_topic,
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&WhiteLineHsvWhiteNode::robotMaskCallback, this, std::placeholders::_1));
    }

    white_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/white_mask", 10);
    green_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/green_mask", 10);
    black_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/black_mask", 10);
    noise_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/noise_mask", 10);
    debug_input_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/input_image", 10);
    debug_white_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/white_mask", 10);
    debug_green_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/green_mask", 10);
    debug_black_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/black_mask", 10);
    debug_noise_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/noise_mask", 10);
    debug_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/overlay_image", 10);

    RCLCPP_INFO(
      get_logger(),
      "white_line_hsv_white_node started. input_topic=%s, robot_mask_topic=%s, "
      "white_h_min=%d, white_h_max=%d, "
      "white_s_max=%d, white_v_min=%d, black_v_max=%d, green_h_min=%d, green_h_max=%d, "
      "green_s_min=%d, green_v_min=%d, enable_timing_log=%s, timing_log_interval=%d, "
      "enable_image_view=%s, enable_controls_window=%s",
      input_topic.c_str(),
      robot_mask_topic.empty() ? "<disabled>" : robot_mask_topic.c_str(),
      white_h_min_,
      white_h_max_,
      white_s_max_,
      white_v_min_,
      black_v_max_,
      green_h_min_,
      green_h_max_,
      green_s_min_,
      green_v_min_,
      enable_timing_log_ ? "true" : "false",
      timing_log_interval_,
      enable_image_view_ ? "true" : "false",
      enable_controls_window_ ? "true" : "false");
  }

  ~WhiteLineHsvWhiteNode() override
  {
    destroyDebugWindows();
  }

private:
  void robotMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      robot_allowed_mask_ = cv_bridge::toCvCopy(msg, "mono8")->image;
      robot_mask_received_ = true;
      robot_mask_validated_ = false;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge failed while reading robot mask: %s",
        e.what());
    }
  }

  bool validateRobotMaskForFrame(const cv::Mat & frame)
  {
    if (!robot_mask_enabled_) {
      return true;
    }

    if (!robot_mask_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "robot_mask_topic is configured but no robot mask has been received yet; dropping frame.");
      return false;
    }

    if (robot_mask_validated_) {
      return true;
    }

    if (robot_allowed_mask_.size() != frame.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "Robot mask size mismatch: mask=%dx%d image=%dx%d. Stopping node.",
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

  void loadThresholdParameters()
  {
    white_h_min_ = clampHue(static_cast<int>(get_parameter("white_h_min").as_int()));
    white_h_max_ = clampHue(static_cast<int>(get_parameter("white_h_max").as_int()));
    white_s_max_ = clampByte(static_cast<int>(get_parameter("white_s_max").as_int()));
    white_v_min_ = clampByte(static_cast<int>(get_parameter("white_v_min").as_int()));
    black_v_max_ = clampByte(static_cast<int>(get_parameter("black_v_max").as_int()));
    green_h_min_ = clampHue(static_cast<int>(get_parameter("green_h_min").as_int()));
    green_h_max_ = clampHue(static_cast<int>(get_parameter("green_h_max").as_int()));
    green_s_min_ = clampByte(static_cast<int>(get_parameter("green_s_min").as_int()));
    green_v_min_ = clampByte(static_cast<int>(get_parameter("green_v_min").as_int()));
  }

  void loadRuntimeParameters()
  {
    enable_timing_log_ = get_parameter("enable_timing_log").as_bool();
    timing_log_interval_ =
      std::max(1, static_cast<int>(get_parameter("timing_log_interval").as_int()));
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
    enable_controls_window_ = get_parameter("enable_controls_window").as_bool();
    show_input_image_ = get_parameter("show_input_image").as_bool();
    show_white_mask_ = get_parameter("show_white_mask").as_bool();
    show_green_mask_ = get_parameter("show_green_mask").as_bool();
    show_black_mask_ = get_parameter("show_black_mask").as_bool();
    show_noise_mask_ = get_parameter("show_noise_mask").as_bool();
    show_overlay_image_ = get_parameter("show_overlay_image").as_bool();
    publish_debug_images_ = get_parameter("publish_debug_images").as_bool();
    publish_input_image_ = get_parameter("publish_input_image").as_bool();
    publish_white_mask_ = get_parameter("publish_white_mask").as_bool();
    publish_green_mask_ = get_parameter("publish_green_mask").as_bool();
    publish_black_mask_ = get_parameter("publish_black_mask").as_bool();
    publish_noise_mask_ = get_parameter("publish_noise_mask").as_bool();
    publish_overlay_image_ = get_parameter("publish_overlay_image").as_bool();
    display_max_width_ =
      std::max(1, static_cast<int>(get_parameter("display_max_width").as_int()));
    display_max_height_ =
      std::max(1, static_cast<int>(get_parameter("display_max_height").as_int()));
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

  void syncControlsWindow(bool should_show)
  {
    if (should_show && !controls_window_created_) {
      cv::namedWindow(kControlsWindowName, cv::WINDOW_AUTOSIZE);
      cv::createTrackbar("white_h_min", kControlsWindowName, nullptr, kHueMax);
      cv::createTrackbar("white_h_max", kControlsWindowName, nullptr, kHueMax);
      cv::createTrackbar("white_s_max", kControlsWindowName, nullptr, kByteMax);
      cv::createTrackbar("white_v_min", kControlsWindowName, nullptr, kByteMax);
      cv::createTrackbar("black_v_max", kControlsWindowName, nullptr, kByteMax);
      cv::createTrackbar("green_h_min", kControlsWindowName, nullptr, kHueMax);
      cv::createTrackbar("green_h_max", kControlsWindowName, nullptr, kHueMax);
      cv::createTrackbar("green_s_min", kControlsWindowName, nullptr, kByteMax);
      cv::createTrackbar("green_v_min", kControlsWindowName, nullptr, kByteMax);

      controls_window_created_ = true;
      controls_window_initialized_ = false;
    } else if (!should_show && controls_window_created_) {
      cv::destroyWindow(kControlsWindowName);
      controls_window_created_ = false;
      controls_window_initialized_ = false;
    }

    if (!controls_window_created_) {
      return;
    }

    if (!controls_window_initialized_) {
      cv::setTrackbarPos("white_h_min", kControlsWindowName, white_h_min_);
      cv::setTrackbarPos("white_h_max", kControlsWindowName, white_h_max_);
      cv::setTrackbarPos("white_s_max", kControlsWindowName, white_s_max_);
      cv::setTrackbarPos("white_v_min", kControlsWindowName, white_v_min_);
      cv::setTrackbarPos("black_v_max", kControlsWindowName, black_v_max_);
      cv::setTrackbarPos("green_h_min", kControlsWindowName, green_h_min_);
      cv::setTrackbarPos("green_h_max", kControlsWindowName, green_h_max_);
      cv::setTrackbarPos("green_s_min", kControlsWindowName, green_s_min_);
      cv::setTrackbarPos("green_v_min", kControlsWindowName, green_v_min_);
      controls_window_initialized_ = true;
    }
  }

  void updateThresholdsFromControls()
  {
    if (!controls_window_created_) {
      return;
    }

    white_h_min_ = clampHue(cv::getTrackbarPos("white_h_min", kControlsWindowName));
    white_h_max_ = clampHue(cv::getTrackbarPos("white_h_max", kControlsWindowName));
    white_s_max_ = clampByte(cv::getTrackbarPos("white_s_max", kControlsWindowName));
    white_v_min_ = clampByte(cv::getTrackbarPos("white_v_min", kControlsWindowName));
    black_v_max_ = clampByte(cv::getTrackbarPos("black_v_max", kControlsWindowName));
    green_h_min_ = clampHue(cv::getTrackbarPos("green_h_min", kControlsWindowName));
    green_h_max_ = clampHue(cv::getTrackbarPos("green_h_max", kControlsWindowName));
    green_s_min_ = clampByte(cv::getTrackbarPos("green_s_min", kControlsWindowName));
    green_v_min_ = clampByte(cv::getTrackbarPos("green_v_min", kControlsWindowName));
  }

  void printCurrentParameters() const
  {
    RCLCPP_INFO(
      get_logger(),
      "white_h_min=%d, white_h_max=%d, white_s_max=%d, white_v_min=%d, "
      "black_v_max=%d, green_h_min=%d, green_h_max=%d, green_s_min=%d, green_v_min=%d",
      white_h_min_,
      white_h_max_,
      white_s_max_,
      white_v_min_,
      black_v_max_,
      green_h_min_,
      green_h_max_,
      green_s_min_,
      green_v_min_);
  }

  void syncImageViewState()
  {
    const bool gui_requested = enable_image_view_ || enable_controls_window_;

    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (gui_requested && !display_available) {
      if (!headless_warned_) {
        RCLCPP_WARN(
          get_logger(),
          "GUI display was requested but no DISPLAY/WAYLAND_DISPLAY is available; "
          "disabling OpenCV windows for this process.");
        headless_warned_ = true;
      }
      enable_image_view_ = false;
      enable_controls_window_ = false;
    }

    syncControlsWindow(enable_controls_window_);
    syncWindow(kInputWindowName, enable_image_view_ && show_input_image_, input_window_created_);
    syncWindow(kMaskWindowName, enable_image_view_ && show_white_mask_, mask_window_created_);
    syncWindow(
      kGreenMaskWindowName, enable_image_view_ && show_green_mask_, green_mask_window_created_);
    syncWindow(
      kBlackMaskWindowName, enable_image_view_ && show_black_mask_, black_mask_window_created_);
    syncWindow(
      kNoiseMaskWindowName, enable_image_view_ && show_noise_mask_, noise_mask_window_created_);
    syncWindow(
      kOverlayWindowName, enable_image_view_ && show_overlay_image_, overlay_window_created_);
  }

  void destroyDebugWindows()
  {
    syncControlsWindow(false);
    syncWindow(kInputWindowName, false, input_window_created_);
    syncWindow(kMaskWindowName, false, mask_window_created_);
    syncWindow(kGreenMaskWindowName, false, green_mask_window_created_);
    syncWindow(kBlackMaskWindowName, false, black_mask_window_created_);
    syncWindow(kNoiseMaskWindowName, false, noise_mask_window_created_);
    syncWindow(kOverlayWindowName, false, overlay_window_created_);
  }

  void showDebugImages(
    const cv::Mat & frame,
    const cv::Mat & white_mask,
    const cv::Mat & green_mask,
    const cv::Mat & black_mask,
    const cv::Mat & noise_mask)
  {
    if (input_window_created_) {
      cv::imshow(kInputWindowName, frame);
      resizeWindowToFitImage(
        kInputWindowName, frame, display_max_width_, display_max_height_);
    }

    if (mask_window_created_) {
      cv::imshow(kMaskWindowName, white_mask);
      resizeWindowToFitImage(
        kMaskWindowName, white_mask, display_max_width_, display_max_height_);
    }

    if (green_mask_window_created_) {
      cv::imshow(kGreenMaskWindowName, green_mask);
      resizeWindowToFitImage(
        kGreenMaskWindowName, green_mask, display_max_width_, display_max_height_);
    }

    if (black_mask_window_created_) {
      cv::imshow(kBlackMaskWindowName, black_mask);
      resizeWindowToFitImage(
        kBlackMaskWindowName, black_mask, display_max_width_, display_max_height_);
    }

    if (noise_mask_window_created_) {
      cv::imshow(kNoiseMaskWindowName, noise_mask);
      resizeWindowToFitImage(
        kNoiseMaskWindowName, noise_mask, display_max_width_, display_max_height_);
    }

    if (overlay_window_created_) {
      cv::Mat overlay = frame.clone();
      overlay.setTo(cv::Scalar(0, 255, 0), white_mask);
      cv::imshow(kOverlayWindowName, overlay);
      resizeWindowToFitImage(
        kOverlayWindowName, overlay, display_max_width_, display_max_height_);
    }
  }

  bool processGuiEvents()
  {
    if (
      !controls_window_created_ && !input_window_created_ && !mask_window_created_ &&
      !green_mask_window_created_ && !black_mask_window_created_ && !noise_mask_window_created_ &&
      !overlay_window_created_)
    {
      return false;
    }

    const int key = cv::waitKey(1);
    if (key == 'p' || key == 'P') {
      printCurrentParameters();
    }
    return true;
  }

  template<typename PublisherT>
  bool shouldPublishDebugImage(const std::shared_ptr<PublisherT> & publisher, bool image_enabled) const
  {
    return publish_debug_images_ && image_enabled && publisher != nullptr &&
           publisher->get_subscription_count() > 0U;
  }

  bool publishDebugImageIfNeeded(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    bool image_enabled,
    const std_msgs::msg::Header & header,
    const std::string & encoding,
    const cv::Mat & image)
  {
    if (!shouldPublishDebugImage(publisher, image_enabled)) {
      return false;
    }
    publisher->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
    return true;
  }

  bool publishDebugImages(
    const std_msgs::msg::Header & header,
    const cv::Mat & frame,
    const cv::Mat & white_mask,
    const cv::Mat & green_mask,
    const cv::Mat & black_mask,
    const cv::Mat & noise_mask)
  {
    bool published_any = false;
    published_any |= publishDebugImageIfNeeded(
      debug_input_pub_, publish_input_image_, header, "bgr8", frame);
    published_any |= publishDebugImageIfNeeded(
      debug_white_mask_pub_, publish_white_mask_, header, "mono8", white_mask);
    published_any |= publishDebugImageIfNeeded(
      debug_green_mask_pub_, publish_green_mask_, header, "mono8", green_mask);
    published_any |= publishDebugImageIfNeeded(
      debug_black_mask_pub_, publish_black_mask_, header, "mono8", black_mask);
    published_any |= publishDebugImageIfNeeded(
      debug_noise_mask_pub_, publish_noise_mask_, header, "mono8", noise_mask);

    if (shouldPublishDebugImage(debug_overlay_pub_, publish_overlay_image_)) {
      cv::Mat overlay = frame.clone();
      overlay.setTo(cv::Scalar(0, 255, 0), white_mask);
      debug_overlay_pub_->publish(*cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg());
      published_any = true;
    }
    return published_any;
  }

  void resetTimingSummary()
  {
    timing_frames_in_interval_ = 0;
    timing_interval_total_us_ = 0;
    timing_interval_max_us_ = 0;
    timing_stage_interval_totals_.fill(0);
  }

  void logTimingSummary(const HsvFrameTiming & timing)
  {
    if (!enable_timing_log_) {
      return;
    }

    const long long callback_duration_us =
      timing.stage_us[static_cast<std::size_t>(HsvTimingStage::CallbackTotal)];

    ++timing_frames_in_interval_;
    timing_interval_total_us_ += callback_duration_us;
    timing_interval_max_us_ =
      std::max(timing_interval_max_us_, static_cast<std::int64_t>(callback_duration_us));
    for (std::size_t i = 0; i < timing_stage_interval_totals_.size(); ++i) {
      timing_stage_interval_totals_[i] += timing.stage_us[i];
    }

    if (timing_frames_in_interval_ < static_cast<std::size_t>(timing_log_interval_)) {
      return;
    }

    const double avg_ms =
      static_cast<double>(timing_interval_total_us_) /
      static_cast<double>(timing_frames_in_interval_) / 1000.0;

    std::ostringstream oss;
    oss << "\n================ HSV White Timing Summary ================\n";
    oss << "frames: " << timing_frames_in_interval_
        << ", current total: " << static_cast<double>(callback_duration_us) / 1000.0
        << " ms, interval avg: " << avg_ms
        << " ms, interval max: " << static_cast<double>(timing_interval_max_us_) / 1000.0
        << " ms\n\n";
    appendTimingTable(oss, timing, timing_stage_interval_totals_, timing_frames_in_interval_);
    oss << "==========================================================";
    RCLCPP_INFO_STREAM(get_logger(), oss.str());

    resetTimingSummary();
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const bool previous_timing_log = enable_timing_log_;
    const int previous_timing_log_interval = timing_log_interval_;
    loadRuntimeParameters();
    syncImageViewState();
    if (controls_window_created_) {
      updateThresholdsFromControls();
    } else {
      loadThresholdParameters();
    }

    if (
      previous_timing_log != enable_timing_log_ ||
      previous_timing_log_interval != timing_log_interval_)
    {
      resetTimingSummary();
    }

    HsvFrameTiming timing;
    const bool timing_enabled = enable_timing_log_;
    const auto callback_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    auto stage_start = timing_enabled ? callback_start : TimePoint{};
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::RuntimeSync,
        stage_start,
        SteadyClock::now());
    }

    cv::Mat frame;
    try {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          HsvTimingStage::CvBridgeConvert,
          stage_start,
          SteadyClock::now());
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge failed: %s",
        e.what());
      return;
    }

    if (!validateRobotMaskForFrame(frame)) {
      return;
    }

    cv::Mat hsv_image;
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::HsvConvert,
        stage_start,
        SteadyClock::now());
    }

    cv::Mat white_candidate;
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    if (white_h_min_ <= white_h_max_) {
      cv::inRange(
        hsv_image,
        cv::Scalar(white_h_min_, 0, white_v_min_),
        cv::Scalar(white_h_max_, white_s_max_, kByteMax),
        white_candidate);
    } else {
      cv::Mat low_range_mask;
      cv::Mat high_range_mask;
      cv::inRange(
        hsv_image,
        cv::Scalar(0, 0, white_v_min_),
        cv::Scalar(white_h_max_, white_s_max_, kByteMax),
        low_range_mask);
      cv::inRange(
        hsv_image,
        cv::Scalar(white_h_min_, 0, white_v_min_),
        cv::Scalar(kHueMax, white_s_max_, kByteMax),
        high_range_mask);
      cv::bitwise_or(low_range_mask, high_range_mask, white_candidate);
    }
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::WhiteThreshold,
        stage_start,
        SteadyClock::now());
    }

    cv::Mat black_candidate;
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    cv::inRange(
      hsv_image,
      cv::Scalar(0, 0, 0),
      cv::Scalar(kHueMax, kByteMax, black_v_max_),
      black_candidate);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::BlackThreshold,
        stage_start,
        SteadyClock::now());
    }

    cv::Mat green_candidate = cv::Mat::zeros(frame.size(), CV_8UC1);
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    if (green_h_min_ <= green_h_max_) {
      cv::inRange(
        hsv_image,
        cv::Scalar(green_h_min_, green_s_min_, green_v_min_),
        cv::Scalar(green_h_max_, kByteMax, kByteMax),
        green_candidate);
    } else {
      cv::Mat low_range_mask;
      cv::Mat high_range_mask;
      cv::inRange(
        hsv_image,
        cv::Scalar(0, green_s_min_, green_v_min_),
        cv::Scalar(green_h_max_, kByteMax, kByteMax),
        low_range_mask);
      cv::inRange(
        hsv_image,
        cv::Scalar(green_h_min_, green_s_min_, green_v_min_),
        cv::Scalar(kHueMax, kByteMax, kByteMax),
        high_range_mask);
      cv::bitwise_or(low_range_mask, high_range_mask, green_candidate);
    }
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::GreenThreshold,
        stage_start,
        SteadyClock::now());
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    cv::Mat remaining =
      robot_mask_enabled_
      ? robot_allowed_mask_.clone()
      : cv::Mat(frame.size(), CV_8UC1, cv::Scalar(255));
    const cv::Mat green_mask = takeFromRemaining(green_candidate, remaining);
    const cv::Mat white_mask = takeFromRemaining(white_candidate, remaining);
    const cv::Mat black_mask = takeFromRemaining(black_candidate, remaining);
    const cv::Mat noise_mask = remaining.clone();
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::ContextResolve,
        stage_start,
        SteadyClock::now());
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    white_mask_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", white_mask).toImageMsg());
    green_mask_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", green_mask).toImageMsg());
    black_mask_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", black_mask).toImageMsg());
    noise_mask_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", noise_mask).toImageMsg());
    publishDebugImages(msg->header, frame, white_mask, green_mask, black_mask, noise_mask);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::PublishOutputs,
        stage_start,
        SteadyClock::now());
    }

    const bool gui_enabled =
      controls_window_created_ || input_window_created_ || mask_window_created_ ||
      green_mask_window_created_ || black_mask_window_created_ || noise_mask_window_created_ ||
      overlay_window_created_;
    if (gui_enabled) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      showDebugImages(frame, white_mask, green_mask, black_mask, noise_mask);
      processGuiEvents();
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          HsvTimingStage::GuiDisplay,
          stage_start,
          SteadyClock::now());
      }
    }

    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        HsvTimingStage::CallbackTotal,
        callback_start,
        SteadyClock::now());
      const long long accounted_stage_us = sumTimingStagesExcludingCallback(timing.stage_us);
      timing.stage_us[static_cast<std::size_t>(HsvTimingStage::UnaccountedOverhead)] =
        std::max(
        0LL,
        timing.stage_us[static_cast<std::size_t>(HsvTimingStage::CallbackTotal)] -
        accounted_stage_us);
      logTimingSummary(timing);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_mask_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr green_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr black_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr noise_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_white_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_green_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_black_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_noise_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_overlay_pub_;

  int white_h_min_{0};
  int white_h_max_{kHueMax};
  int white_s_max_{60};
  int white_v_min_{170};
  int black_v_max_{70};
  int green_h_min_{35};
  int green_h_max_{95};
  int green_s_min_{40};
  int green_v_min_{40};
  bool enable_timing_log_{true};
  int timing_log_interval_{30};
  bool enable_image_view_{false};
  bool enable_controls_window_{false};
  bool show_input_image_{true};
  bool show_white_mask_{true};
  bool show_green_mask_{false};
  bool show_black_mask_{false};
  bool show_noise_mask_{false};
  bool show_overlay_image_{true};
  bool publish_debug_images_{false};
  bool publish_input_image_{true};
  bool publish_white_mask_{true};
  bool publish_green_mask_{true};
  bool publish_black_mask_{true};
  bool publish_noise_mask_{true};
  bool publish_overlay_image_{true};
  bool headless_warned_{false};
  bool controls_window_created_{false};
  bool controls_window_initialized_{false};
  bool input_window_created_{false};
  bool mask_window_created_{false};
  bool green_mask_window_created_{false};
  bool black_mask_window_created_{false};
  bool noise_mask_window_created_{false};
  bool overlay_window_created_{false};
  int display_max_width_{960};
  int display_max_height_{720};
  bool robot_mask_enabled_{false};
  bool robot_mask_received_{false};
  bool robot_mask_validated_{false};
  cv::Mat robot_allowed_mask_;
  std::size_t timing_frames_in_interval_{0};
  std::int64_t timing_interval_total_us_{0};
  std::int64_t timing_interval_max_us_{0};
  HsvTimingArray timing_stage_interval_totals_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhiteLineHsvWhiteNode>());
  rclcpp::shutdown();
  return 0;
}
