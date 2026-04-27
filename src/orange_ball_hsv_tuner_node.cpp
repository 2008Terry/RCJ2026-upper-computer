#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kHueMax = 179;
constexpr int kByteMax = 255;
constexpr char kControlsWindowName[] = "Orange Ball HSV Controls";
constexpr char kInputWindowName[] = "Orange Ball HSV Input";
constexpr char kMaskWindowName[] = "Orange Ball HSV Mask";
constexpr char kOverlayWindowName[] = "Orange Ball HSV Overlay";

int clampHue(int value)
{
  return std::clamp(value, 0, kHueMax);
}

int clampByte(int value)
{
  return std::clamp(value, 0, kByteMax);
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

}  // namespace

class OrangeBallHsvTunerNode : public rclcpp::Node
{
public:
  OrangeBallHsvTunerNode()
  : Node("orange_ball_hsv_tuner_node")
  {
    declare_parameter<std::string>("input_topic", "/camera/image_raw");
    declare_parameter<std::string>(
      "robot_mask_path", packageConfigPath("mask.png"));
    declare_parameter("orange_h_min", 5);
    declare_parameter("orange_h_max", 30);
    declare_parameter("orange_s_min", 100);
    declare_parameter("orange_v_min", 60);
    declare_parameter("enable_morph_open", true);
    declare_parameter("morph_kernel_size", 3);
    declare_parameter("enable_image_view", true);
    declare_parameter("enable_controls_window", true);
    declare_parameter("show_input_image", true);
    declare_parameter("show_mask", true);
    declare_parameter("show_overlay_image", true);
    declare_parameter("publish_debug_images", false);
    declare_parameter("publish_input_image", true);
    declare_parameter("publish_mask", true);
    declare_parameter("publish_overlay_image", true);

    input_topic_ = get_parameter("input_topic").as_string();
    robot_mask_path_ = get_parameter("robot_mask_path").as_string();
    orange_h_min_ = clampHue(static_cast<int>(get_parameter("orange_h_min").as_int()));
    orange_h_max_ = clampHue(static_cast<int>(get_parameter("orange_h_max").as_int()));
    orange_s_min_ = clampByte(static_cast<int>(get_parameter("orange_s_min").as_int()));
    orange_v_min_ = clampByte(static_cast<int>(get_parameter("orange_v_min").as_int()));
    enable_morph_open_ = get_parameter("enable_morph_open").as_bool();
    morph_open_trackbar_ = enable_morph_open_ ? 1 : 0;
    morph_kernel_size_ =
      std::max(1, static_cast<int>(get_parameter("morph_kernel_size").as_int()));
    loadRuntimeParameters();
    loadRobotMask();

    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (!display_available && (enable_image_view_ || enable_controls_window_)) {
      RCLCPP_WARN(
        get_logger(),
        "GUI display was requested but no DISPLAY/WAYLAND_DISPLAY is available; "
        "disabling OpenCV windows for this process.");
      enable_image_view_ = false;
      enable_controls_window_ = false;
    }

    syncImageViewState();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OrangeBallHsvTunerNode::imageCallback, this, std::placeholders::_1));
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/mask", 10);
    input_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/input_image", 10);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/overlay_image", 10);

    RCLCPP_INFO(
      get_logger(),
      "orange_ball_hsv_tuner_node started. input_topic=%s, robot_mask_path=%s; press 'p' in any "
      "OpenCV window to print the current parameter values.",
      input_topic_.c_str(),
      robot_mask_enabled_ ? robot_mask_path_.c_str() : "<disabled>");
  }

  ~OrangeBallHsvTunerNode() override
  {
    destroyDebugWindows();
  }

private:
  void loadRuntimeParameters()
  {
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
    enable_controls_window_ = get_parameter("enable_controls_window").as_bool();
    show_input_image_ = get_parameter("show_input_image").as_bool();
    show_mask_ = get_parameter("show_mask").as_bool();
    show_overlay_image_ = get_parameter("show_overlay_image").as_bool();
    publish_debug_images_ = get_parameter("publish_debug_images").as_bool();
    publish_input_image_ = get_parameter("publish_input_image").as_bool();
    publish_mask_ = get_parameter("publish_mask").as_bool();
    publish_overlay_image_ = get_parameter("publish_overlay_image").as_bool();
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
      cv::createTrackbar("orange_h_min", kControlsWindowName, &orange_h_min_, kHueMax);
      cv::createTrackbar("orange_h_max", kControlsWindowName, &orange_h_max_, kHueMax);
      cv::createTrackbar("orange_s_min", kControlsWindowName, &orange_s_min_, kByteMax);
      cv::createTrackbar("orange_v_min", kControlsWindowName, &orange_v_min_, kByteMax);
      cv::createTrackbar("enable_morph_open", kControlsWindowName, &morph_open_trackbar_, 1);
      cv::createTrackbar("morph_kernel_size", kControlsWindowName, &morph_kernel_size_, 15);
      controls_window_created_ = true;
    } else if (!should_show && controls_window_created_) {
      cv::destroyWindow(kControlsWindowName);
      controls_window_created_ = false;
    }
  }

  void syncImageViewState()
  {
    loadRuntimeParameters();
    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if ((enable_image_view_ || enable_controls_window_) && !display_available) {
      enable_image_view_ = false;
      enable_controls_window_ = false;
    }
    syncControlsWindow(enable_controls_window_);
    syncWindow(kInputWindowName, enable_image_view_ && show_input_image_, input_window_created_);
    syncWindow(kMaskWindowName, enable_image_view_ && show_mask_, mask_window_created_);
    syncWindow(kOverlayWindowName, enable_image_view_ && show_overlay_image_, overlay_window_created_);
  }

  void destroyDebugWindows()
  {
    syncControlsWindow(false);
    syncWindow(kInputWindowName, false, input_window_created_);
    syncWindow(kMaskWindowName, false, mask_window_created_);
    syncWindow(kOverlayWindowName, false, overlay_window_created_);
  }

  void loadRobotMask()
  {
    robot_mask_enabled_ = false;
    robot_mask_validated_ = false;
    robot_allowed_mask_.release();

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

  void applyRobotMask(cv::Mat & mask) const
  {
    if (!robot_mask_enabled_ || mask.empty()) {
      return;
    }

    cv::bitwise_and(mask, robot_allowed_mask_, mask);
  }

  void printCurrentParameters() const
  {
    RCLCPP_INFO(
      get_logger(),
      "orange_h_min=%d, orange_h_max=%d, orange_s_min=%d, orange_v_min=%d, "
      "enable_morph_open=%s, morph_kernel_size=%d",
      orange_h_min_,
      orange_h_max_,
      orange_s_min_,
      orange_v_min_,
      enable_morph_open_ ? "true" : "false",
      morph_kernel_size_);
  }

  void buildMask(const cv::Mat & frame, cv::Mat & mask)
  {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

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

    applyRobotMask(mask);

    if (enable_morph_open_ && morph_kernel_size_ > 1) {
      const int kernel_size = std::max(1, morph_kernel_size_ | 1);
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernel_size, kernel_size));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
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

  void publishDebugImages(
    const std_msgs::msg::Header & header,
    const cv::Mat & frame,
    const cv::Mat & mask)
  {
    publishDebugImageIfNeeded(input_pub_, publish_input_image_, header, "bgr8", frame);
    publishDebugImageIfNeeded(mask_pub_, publish_mask_, header, "mono8", mask);

    if (shouldPublishDebugImage(overlay_pub_, publish_overlay_image_)) {
      cv::Mat overlay = frame.clone();
      overlay.setTo(cv::Scalar(0, 255, 255), mask);
      overlay_pub_->publish(*cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg());
    }
  }

  void showDebugImages(const cv::Mat & frame, const cv::Mat & mask)
  {
    if (input_window_created_) {
      cv::imshow(kInputWindowName, frame);
    }
    if (mask_window_created_) {
      cv::imshow(kMaskWindowName, mask);
    }
    if (overlay_window_created_) {
      cv::Mat overlay = frame.clone();
      overlay.setTo(cv::Scalar(0, 255, 255), mask);
      cv::imshow(kOverlayWindowName, overlay);
    }
    if (input_window_created_ || mask_window_created_ || overlay_window_created_) {
      const int key = cv::waitKey(1);
      if (key == 'p' || key == 'P') {
        printCurrentParameters();
      }
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    syncImageViewState();
    cv::Mat frame;
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

    if (!validateRobotMaskForFrame(frame)) {
      return;
    }

    if (controls_window_created_) {
      orange_h_min_ = clampHue(cv::getTrackbarPos("orange_h_min", kControlsWindowName));
      orange_h_max_ = clampHue(cv::getTrackbarPos("orange_h_max", kControlsWindowName));
      orange_s_min_ = clampByte(cv::getTrackbarPos("orange_s_min", kControlsWindowName));
      orange_v_min_ = clampByte(cv::getTrackbarPos("orange_v_min", kControlsWindowName));
      morph_open_trackbar_ = cv::getTrackbarPos("enable_morph_open", kControlsWindowName);
      enable_morph_open_ = morph_open_trackbar_ > 0;
      morph_kernel_size_ =
        std::max(1, cv::getTrackbarPos("morph_kernel_size", kControlsWindowName));
    } else {
      orange_h_min_ = clampHue(static_cast<int>(get_parameter("orange_h_min").as_int()));
      orange_h_max_ = clampHue(static_cast<int>(get_parameter("orange_h_max").as_int()));
      orange_s_min_ = clampByte(static_cast<int>(get_parameter("orange_s_min").as_int()));
      orange_v_min_ = clampByte(static_cast<int>(get_parameter("orange_v_min").as_int()));
      enable_morph_open_ = get_parameter("enable_morph_open").as_bool();
      morph_open_trackbar_ = enable_morph_open_ ? 1 : 0;
      morph_kernel_size_ =
        std::max(1, static_cast<int>(get_parameter("morph_kernel_size").as_int()));
    }

    cv::Mat mask;
    buildMask(frame, mask);
    publishDebugImages(msg->header, frame, mask);
    showDebugImages(frame, mask);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  std::string input_topic_;
  std::string robot_mask_path_;
  int orange_h_min_ = 5;
  int orange_h_max_ = 30;
  int orange_s_min_ = 100;
  int orange_v_min_ = 60;
  bool enable_morph_open_ = true;
  int morph_open_trackbar_ = 1;
  int morph_kernel_size_ = 3;
  bool enable_image_view_ = true;
  bool enable_controls_window_ = true;
  bool show_input_image_ = true;
  bool show_mask_ = true;
  bool show_overlay_image_ = true;
  bool publish_debug_images_ = false;
  bool publish_input_image_ = true;
  bool publish_mask_ = true;
  bool publish_overlay_image_ = true;
  bool controls_window_created_ = false;
  bool input_window_created_ = false;
  bool mask_window_created_ = false;
  bool overlay_window_created_ = false;
  bool robot_mask_enabled_ = false;
  bool robot_mask_validated_ = false;
  cv::Mat robot_allowed_mask_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrangeBallHsvTunerNode>());
  rclcpp::shutdown();
  return 0;
}
