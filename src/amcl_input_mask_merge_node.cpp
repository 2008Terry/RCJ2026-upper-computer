#include <algorithm>
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
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>

#include "rcj_localization/white_line_debug_utils.hpp"

namespace
{

cv::Mat normalizeBinaryMask(const cv::Mat &input_mask)
{
  cv::Mat binary_mask;
  cv::compare(input_mask, 0, binary_mask, cv::CMP_GT);
  return binary_mask;
}

cv::Mat createOverlayImage(
    const cv::Mat &white_mask,
    const cv::Mat &black_mask)
{
  cv::Mat overlay(white_mask.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  overlay = rcj_loc::vision::debug::createMaskOverlay(
      overlay,
      white_mask,
      cv::Scalar(255, 255, 255),
      1.0);
  overlay = rcj_loc::vision::debug::createMaskOverlay(
      overlay,
      black_mask,
      cv::Scalar(0, 0, 255),
      1.0);
  return overlay;
}

} // namespace

class AmclInputMaskMergeNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using ExactSyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;

  AmclInputMaskMergeNode()
      : Node("amcl_input_mask_merge_node")
  {
    declare_parameter<std::string>(
        "white_mask_topic",
        "/white_line_dt_ridge_filter_node/white_final_mask");
    declare_parameter<std::string>(
        "black_mask_topic",
        "/yolo_roi_black_mask/black_mask");
    declare_parameter("sync_queue_size", 60);
    declare_parameter("publish_debug_image", false);
    declare_parameter<std::string>("debug_image_topic", "~/debug/overlay_image");

    const auto white_mask_topic = get_parameter("white_mask_topic").as_string();
    const auto black_mask_topic = get_parameter("black_mask_topic").as_string();
    const auto sync_queue_size = std::max(
        1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();

    white_mask_sub_.subscribe(this, white_mask_topic, rmw_qos_profile_sensor_data);
    black_mask_sub_.subscribe(this, black_mask_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
        ExactSyncPolicy(sync_queue_size),
        white_mask_sub_,
        black_mask_sub_);
    sync_->registerCallback(
        std::bind(
            &AmclInputMaskMergeNode::synchronizedCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    combined_mask_pub_ = create_publisher<Image>("~/combined_mask", 10);
    if (publish_debug_image_)
    {
      const auto debug_image_topic = get_parameter("debug_image_topic").as_string();
      debug_overlay_pub_ = create_publisher<Image>(debug_image_topic, 10);
    }

    RCLCPP_INFO(
        get_logger(),
        "amcl_input_mask_merge_node started. white_mask_topic='%s', "
        "black_mask_topic='%s', sync_queue_size=%d, publish_debug_image=%s",
        white_mask_topic.c_str(),
        black_mask_topic.c_str(),
        sync_queue_size,
        publish_debug_image_ ? "true" : "false");
  }

private:
  void synchronizedCallback(
      const Image::ConstSharedPtr &white_mask_msg,
      const Image::ConstSharedPtr &black_mask_msg)
  {
    cv::Mat white_mask;
    cv::Mat black_mask;
    try
    {
      white_mask = cv_bridge::toCvCopy(white_mask_msg, "mono8")->image;
      black_mask = cv_bridge::toCvCopy(black_mask_msg, "mono8")->image;
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "cv_bridge failed: %s",
          e.what());
      return;
    }

    if (white_mask.size() != black_mask.size())
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "White/black mask size mismatch: white=%dx%d black=%dx%d. "
          "Dropping synchronized pair.",
          white_mask.cols,
          white_mask.rows,
          black_mask.cols,
          black_mask.rows);
      return;
    }

    if (white_mask_msg->header.frame_id != black_mask_msg->header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "White/black mask frame_id mismatch: white='%s' black='%s'. "
          "Dropping synchronized pair.",
          white_mask_msg->header.frame_id.c_str(),
          black_mask_msg->header.frame_id.c_str());
      return;
    }

    const cv::Mat white_binary = normalizeBinaryMask(white_mask);
    const cv::Mat black_binary = normalizeBinaryMask(black_mask);

    if (!logged_first_sync_)
    {
      RCLCPP_INFO(
          get_logger(),
          "Received first synchronized white/black mask pair at %dx%d.",
          white_binary.cols,
          white_binary.rows);
      logged_first_sync_ = true;
    }

    cv::Mat combined_mask;
    cv::bitwise_or(white_binary, black_binary, combined_mask);

    combined_mask_pub_->publish(
        *cv_bridge::CvImage(
             white_mask_msg->header, "mono8", combined_mask)
             .toImageMsg());

    if (debug_overlay_pub_)
    {
      const cv::Mat overlay_image =
          createOverlayImage(white_binary, black_binary);
      debug_overlay_pub_->publish(
          *cv_bridge::CvImage(
               white_mask_msg->header, "bgr8", overlay_image)
               .toImageMsg());
    }
  }

  message_filters::Subscriber<Image> white_mask_sub_;
  message_filters::Subscriber<Image> black_mask_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_;

  rclcpp::Publisher<Image>::SharedPtr combined_mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_overlay_pub_;

  bool publish_debug_image_ = false;
  bool logged_first_sync_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclInputMaskMergeNode>());
  rclcpp::shutdown();
  return 0;
}
