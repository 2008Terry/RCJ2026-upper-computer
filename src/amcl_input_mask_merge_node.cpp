#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
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

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

long long elapsedUs(const TimePoint &start, const TimePoint &end)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

std::string formatStamp(const builtin_interfaces::msg::Time &stamp)
{
  std::ostringstream oss;
  oss << stamp.sec << '.';
  oss.width(9);
  oss.fill('0');
  oss << stamp.nanosec;
  return oss.str();
}

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
    declare_parameter("enable_timing_log", true);
    declare_parameter("timing_log_interval", 1);

    const auto white_mask_topic = get_parameter("white_mask_topic").as_string();
    const auto black_mask_topic = get_parameter("black_mask_topic").as_string();
    const auto sync_queue_size = std::max(
        1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    enable_timing_log_ = get_parameter("enable_timing_log").as_bool();
    timing_log_interval_ = std::max(
        1, static_cast<int>(get_parameter("timing_log_interval").as_int()));
    frame_cycle_start_ = SteadyClock::now();
    white_arrival_time_ = frame_cycle_start_;
    black_arrival_time_ = frame_cycle_start_;

    white_mask_sub_.subscribe(this, white_mask_topic, rmw_qos_profile_sensor_data);
    black_mask_sub_.subscribe(this, black_mask_topic, rmw_qos_profile_sensor_data);
    white_mask_sub_.registerCallback(
        std::bind(
            &AmclInputMaskMergeNode::recordWhiteMaskArrival,
            this,
            std::placeholders::_1));
    black_mask_sub_.registerCallback(
        std::bind(
            &AmclInputMaskMergeNode::recordBlackMaskArrival,
            this,
            std::placeholders::_1));
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
        "black_mask_topic='%s', sync_queue_size=%d, publish_debug_image=%s, "
        "enable_timing_log=%s, timing_log_interval=%d",
        white_mask_topic.c_str(),
        black_mask_topic.c_str(),
        sync_queue_size,
        publish_debug_image_ ? "true" : "false",
        enable_timing_log_ ? "true" : "false",
        timing_log_interval_);
  }

private:
  void recordWhiteMaskArrival(const Image::ConstSharedPtr &)
  {
    recordMaskArrival(MaskSource::White);
  }

  void recordBlackMaskArrival(const Image::ConstSharedPtr &)
  {
    recordMaskArrival(MaskSource::Black);
  }

  void synchronizedCallback(
      const Image::ConstSharedPtr &white_mask_msg,
      const Image::ConstSharedPtr &black_mask_msg)
  {
    const TimePoint sync_callback_start = SteadyClock::now();
    cv::Mat white_mask;
    cv::Mat black_mask;
    try
    {
      white_mask = cv_bridge::toCvCopy(white_mask_msg, "mono8")->image;
      black_mask = cv_bridge::toCvCopy(black_mask_msg, "mono8")->image;
    }
    catch (const cv_bridge::Exception &e)
    {
      clearPendingArrivals();
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
      clearPendingArrivals();
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
      clearPendingArrivals();
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

    logFrameTiming(
        white_mask_msg->header.stamp,
        black_mask_msg->header.stamp,
        sync_callback_start,
        SteadyClock::now());
  }

  enum class MaskSource
  {
    White,
    Black
  };

  void recordMaskArrival(MaskSource source)
  {
    if (!enable_timing_log_)
    {
      return;
    }

    const TimePoint now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(timing_mutex_);
    if (source == MaskSource::White)
    {
      if (!white_arrival_recorded_)
      {
        white_arrival_time_ = now;
        white_arrival_recorded_ = true;
      }
      return;
    }

    if (!black_arrival_recorded_)
    {
      black_arrival_time_ = now;
      black_arrival_recorded_ = true;
    }
  }

  void clearPendingArrivals()
  {
    if (!enable_timing_log_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(timing_mutex_);
    white_arrival_recorded_ = false;
    black_arrival_recorded_ = false;
    white_arrival_time_ = frame_cycle_start_;
    black_arrival_time_ = frame_cycle_start_;
  }

  void logFrameTiming(
      const builtin_interfaces::msg::Time &white_stamp,
      const builtin_interfaces::msg::Time &black_stamp,
      const TimePoint &sync_callback_start,
      const TimePoint &publish_end)
  {
    if (!enable_timing_log_)
    {
      return;
    }

    long long white_wait_us = 0;
    long long black_wait_us = 0;
    long long total_cycle_us = 0;
    std::uint64_t frame_index = 0;
    const long long sync_to_publish_us = elapsedUs(sync_callback_start, publish_end);
    const long long white_stamp_ns =
        static_cast<long long>(white_stamp.sec) * 1000000000LL +
        static_cast<long long>(white_stamp.nanosec);
    const long long black_stamp_ns =
        static_cast<long long>(black_stamp.sec) * 1000000000LL +
        static_cast<long long>(black_stamp.nanosec);
    const long long stamp_delta_ns = std::llabs(white_stamp_ns - black_stamp_ns);

    {
      std::lock_guard<std::mutex> lock(timing_mutex_);
      const TimePoint white_ready_time =
          white_arrival_recorded_ ? white_arrival_time_ : sync_callback_start;
      const TimePoint black_ready_time =
          black_arrival_recorded_ ? black_arrival_time_ : sync_callback_start;

      white_wait_us = elapsedUs(frame_cycle_start_, white_ready_time);
      black_wait_us = elapsedUs(frame_cycle_start_, black_ready_time);
      total_cycle_us = elapsedUs(frame_cycle_start_, publish_end);

      frame_cycle_start_ = publish_end;
      white_arrival_recorded_ = false;
      black_arrival_recorded_ = false;
      white_arrival_time_ = publish_end;
      black_arrival_time_ = publish_end;
      frame_index = ++published_frame_count_;
    }

    if (frame_index % static_cast<std::uint64_t>(timing_log_interval_) != 0U)
    {
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Merged mask frame %llu timing: white_wait=%.3f ms, black_wait=%.3f ms, "
        "merge_and_publish=%.3f ms, sync_to_publish=%.3f ms, total_cycle=%.3f ms, "
        "white_stamp=%s, black_stamp=%s, stamp_delta=%.3f ms",
        static_cast<unsigned long long>(frame_index),
        static_cast<double>(white_wait_us) / 1000.0,
        static_cast<double>(black_wait_us) / 1000.0,
        static_cast<double>(sync_to_publish_us) / 1000.0,
        static_cast<double>(sync_to_publish_us) / 1000.0,
        static_cast<double>(total_cycle_us) / 1000.0,
        formatStamp(white_stamp).c_str(),
        formatStamp(black_stamp).c_str(),
        static_cast<double>(stamp_delta_ns) / 1000000.0);
  }

  message_filters::Subscriber<Image> white_mask_sub_;
  message_filters::Subscriber<Image> black_mask_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_;

  rclcpp::Publisher<Image>::SharedPtr combined_mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_overlay_pub_;

  bool publish_debug_image_ = false;
  bool logged_first_sync_ = false;
  bool enable_timing_log_ = true;
  int timing_log_interval_ = 1;
  std::mutex timing_mutex_;
  TimePoint frame_cycle_start_{};
  TimePoint white_arrival_time_{};
  TimePoint black_arrival_time_{};
  bool white_arrival_recorded_ = false;
  bool black_arrival_recorded_ = false;
  std::uint64_t published_frame_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclInputMaskMergeNode>());
  rclcpp::shutdown();
  return 0;
}
