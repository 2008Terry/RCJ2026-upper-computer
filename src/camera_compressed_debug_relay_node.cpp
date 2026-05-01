#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rcj_localization/white_line_debug_utils.hpp"

class CameraCompressedDebugRelayNode : public rclcpp::Node
{
public:
  CameraCompressedDebugRelayNode()
  : Node("camera_compressed_debug_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/image_raw");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/camera/image_raw/compressed_debug");
    declare_parameter("debug_jpeg_quality", 80);
    declare_parameter("debug_image_max_fps", 5.0);

    loadRuntimeParameters();
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      output_topic,
      rclcpp::SensorDataQoS());
    subscriber_sync_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&CameraCompressedDebugRelayNode::syncSubscription, this));

    RCLCPP_INFO(
      get_logger(),
      "camera_compressed_debug_relay started. input_topic=%s, output_topic=%s",
      input_topic_.c_str(),
      output_topic.c_str());
  }

private:
  void loadRuntimeParameters()
  {
    debug_jpeg_quality_ =
      std::clamp(static_cast<int>(get_parameter("debug_jpeg_quality").as_int()), 1, 100);
    debug_image_max_fps_ = std::max(0.0, get_parameter("debug_image_max_fps").as_double());
  }

  void syncSubscription()
  {
    if (publisher_ == nullptr) {
      return;
    }

    loadRuntimeParameters();
    const bool has_subscribers = publisher_->get_subscription_count() > 0U;
    if (has_subscribers && image_sub_ == nullptr) {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&CameraCompressedDebugRelayNode::imageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed to %s for compressed camera debug.", input_topic_.c_str());
    } else if (!has_subscribers && image_sub_ != nullptr) {
      image_sub_.reset();
      last_publish_time_ = {};
      RCLCPP_INFO(get_logger(), "No compressed camera debug subscribers; unsubscribed from %s.", input_topic_.c_str());
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (publisher_ == nullptr || publisher_->get_subscription_count() == 0U) {
      return;
    }

    loadRuntimeParameters();
    const auto now = std::chrono::steady_clock::now();
    if (!rcj_loc::vision::debug::consumeFpsGate(
        debug_image_max_fps_, now, last_publish_time_))
    {
      return;
    }

    try {
      const auto cv_image = cv_bridge::toCvShare(msg, msg->encoding);
      auto compressed_msg = rcj_loc::vision::debug::encodeJpegCompressedImage(
        msg->header,
        msg->encoding,
        cv_image->image,
        debug_jpeg_quality_);
      if (!compressed_msg.has_value()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Failed to JPEG-compress camera debug image.");
        return;
      }
      publisher_->publish(*compressed_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge failed in camera compressed debug relay: %s",
        e.what());
    }
  }

  std::string input_topic_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr subscriber_sync_timer_;
  int debug_jpeg_quality_ = 80;
  double debug_image_max_fps_ = 5.0;
  std::chrono::steady_clock::time_point last_publish_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCompressedDebugRelayNode>());
  rclcpp::shutdown();
  return 0;
}
