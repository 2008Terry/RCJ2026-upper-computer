#include <algorithm>
#include <cstdlib>
#include <functional>
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

#include <opencv2/highgui.hpp>
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

}  // namespace

class OrangeBallHsvTunerNode : public rclcpp::Node
{
public:
  OrangeBallHsvTunerNode()
  : Node("orange_ball_hsv_tuner_node")
  {
    declare_parameter<std::string>("input_topic", "/camera/image_raw");
    declare_parameter("orange_h_min", 5);
    declare_parameter("orange_h_max", 30);
    declare_parameter("orange_s_min", 100);
    declare_parameter("orange_v_min", 60);
    declare_parameter("enable_morph_open", true);
    declare_parameter("morph_kernel_size", 3);

    input_topic_ = get_parameter("input_topic").as_string();
    orange_h_min_ = clampHue(static_cast<int>(get_parameter("orange_h_min").as_int()));
    orange_h_max_ = clampHue(static_cast<int>(get_parameter("orange_h_max").as_int()));
    orange_s_min_ = clampByte(static_cast<int>(get_parameter("orange_s_min").as_int()));
    orange_v_min_ = clampByte(static_cast<int>(get_parameter("orange_v_min").as_int()));
    enable_morph_open_ = get_parameter("enable_morph_open").as_bool();
    morph_open_trackbar_ = enable_morph_open_ ? 1 : 0;
    morph_kernel_size_ =
      std::max(1, static_cast<int>(get_parameter("morph_kernel_size").as_int()));

    const bool display_available =
      std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (!display_available) {
      throw std::runtime_error(
              "orange_ball_hsv_tuner_node requires a GUI environment with DISPLAY/WAYLAND_DISPLAY.");
    }

    cv::namedWindow(kControlsWindowName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(kInputWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(kMaskWindowName, cv::WINDOW_NORMAL);
    cv::namedWindow(kOverlayWindowName, cv::WINDOW_NORMAL);

    cv::createTrackbar("orange_h_min", kControlsWindowName, &orange_h_min_, kHueMax);
    cv::createTrackbar("orange_h_max", kControlsWindowName, &orange_h_max_, kHueMax);
    cv::createTrackbar("orange_s_min", kControlsWindowName, &orange_s_min_, kByteMax);
    cv::createTrackbar("orange_v_min", kControlsWindowName, &orange_v_min_, kByteMax);
    cv::createTrackbar("enable_morph_open", kControlsWindowName, &morph_open_trackbar_, 1);
    cv::createTrackbar("morph_kernel_size", kControlsWindowName, &morph_kernel_size_, 15);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OrangeBallHsvTunerNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "orange_ball_hsv_tuner_node started. input_topic=%s; press 'p' in any OpenCV window "
      "to print the current parameter values.",
      input_topic_.c_str());
  }

  ~OrangeBallHsvTunerNode() override
  {
    cv::destroyWindow(kControlsWindowName);
    cv::destroyWindow(kInputWindowName);
    cv::destroyWindow(kMaskWindowName);
    cv::destroyWindow(kOverlayWindowName);
  }

private:
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

    if (enable_morph_open_ && morph_kernel_size_ > 1) {
      const int kernel_size = std::max(1, morph_kernel_size_ | 1);
      const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernel_size, kernel_size));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
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

    orange_h_min_ = clampHue(cv::getTrackbarPos("orange_h_min", kControlsWindowName));
    orange_h_max_ = clampHue(cv::getTrackbarPos("orange_h_max", kControlsWindowName));
    orange_s_min_ = clampByte(cv::getTrackbarPos("orange_s_min", kControlsWindowName));
    orange_v_min_ = clampByte(cv::getTrackbarPos("orange_v_min", kControlsWindowName));
    morph_open_trackbar_ = cv::getTrackbarPos("enable_morph_open", kControlsWindowName);
    enable_morph_open_ = morph_open_trackbar_ > 0;
    morph_kernel_size_ = std::max(1, cv::getTrackbarPos("morph_kernel_size", kControlsWindowName));

    cv::Mat mask;
    buildMask(frame, mask);

    cv::Mat overlay = frame.clone();
    overlay.setTo(cv::Scalar(0, 255, 255), mask);

    cv::imshow(kInputWindowName, frame);
    cv::imshow(kMaskWindowName, mask);
    cv::imshow(kOverlayWindowName, overlay);
    const int key = cv::waitKey(1);
    if (key == 'p' || key == 'P') {
      printCurrentParameters();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::string input_topic_;
  int orange_h_min_ = 5;
  int orange_h_max_ = 30;
  int orange_s_min_ = 100;
  int orange_v_min_ = 60;
  bool enable_morph_open_ = true;
  int morph_open_trackbar_ = 1;
  int morph_kernel_size_ = 3;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrangeBallHsvTunerNode>());
  rclcpp::shutdown();
  return 0;
}
