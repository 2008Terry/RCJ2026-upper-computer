#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <deque>
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
  struct ArrivalSample
  {
    builtin_interfaces::msg::Time stamp{};
    TimePoint arrival_time{};
  };

  static constexpr std::size_t kMaxRecentArrivalHistory = 256;

  void recordWhiteMaskArrival(const Image::ConstSharedPtr &msg)
  {
    recordMaskArrival(MaskSource::White, msg);
  }

  void recordBlackMaskArrival(const Image::ConstSharedPtr &msg)
  {
    recordMaskArrival(MaskSource::Black, msg);
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
      if (enable_timing_log_)
      {
        std::lock_guard<std::mutex> lock(timing_mutex_);
        ++cv_bridge_error_count_;
      }
      discardFailedPairArrivals(
          white_mask_msg->header.stamp,
          black_mask_msg->header.stamp);
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
      if (enable_timing_log_)
      {
        std::lock_guard<std::mutex> lock(timing_mutex_);
        ++size_mismatch_drop_count_;
      }
      discardFailedPairArrivals(
          white_mask_msg->header.stamp,
          black_mask_msg->header.stamp);
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
      if (enable_timing_log_)
      {
        std::lock_guard<std::mutex> lock(timing_mutex_);
        ++frame_id_mismatch_drop_count_;
      }
      discardFailedPairArrivals(
          white_mask_msg->header.stamp,
          black_mask_msg->header.stamp);
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

  static bool stampsEqual(
      const builtin_interfaces::msg::Time &lhs,
      const builtin_interfaces::msg::Time &rhs)
  {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
  }

  static bool lookupArrivalTime(
      const std::deque<ArrivalSample> &recent_arrivals,
      const builtin_interfaces::msg::Time &stamp,
      TimePoint &arrival_time)
  {
    for (auto it = recent_arrivals.rbegin(); it != recent_arrivals.rend(); ++it)
    {
      if (stampsEqual(it->stamp, stamp))
      {
        arrival_time = it->arrival_time;
        return true;
      }
    }
    return false;
  }

  static void discardArrivalSamplesUpTo(
      std::deque<ArrivalSample> &recent_arrivals,
      const builtin_interfaces::msg::Time &stamp)
  {
    while (!recent_arrivals.empty())
    {
      const bool is_match = stampsEqual(recent_arrivals.front().stamp, stamp);
      recent_arrivals.pop_front();
      if (is_match)
      {
        break;
      }
    }
  }

  static void discardArrivalSample(
      std::deque<ArrivalSample> &recent_arrivals,
      const builtin_interfaces::msg::Time &stamp)
  {
    const auto it = std::find_if(
        recent_arrivals.begin(),
        recent_arrivals.end(),
        [&stamp](const ArrivalSample &sample)
        { return stampsEqual(sample.stamp, stamp); });
    if (it != recent_arrivals.end())
    {
      recent_arrivals.erase(it);
    }
  }

  static bool discardArrivalSampleIfPresent(
      std::deque<ArrivalSample> &recent_arrivals,
      const builtin_interfaces::msg::Time &stamp)
  {
    const auto it = std::find_if(
        recent_arrivals.begin(),
        recent_arrivals.end(),
        [&stamp](const ArrivalSample &sample)
        { return stampsEqual(sample.stamp, stamp); });
    if (it == recent_arrivals.end())
    {
      return false;
    }

    recent_arrivals.erase(it);
    return true;
  }

  void discardFailedPairArrivals(
      const builtin_interfaces::msg::Time &white_stamp,
      const builtin_interfaces::msg::Time &black_stamp)
  {
    if (!enable_timing_log_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(timing_mutex_);
    if (discardArrivalSampleIfPresent(white_recent_arrivals_, white_stamp))
    {
      ++dropped_or_expired_white_arrivals_;
    }
    if (discardArrivalSampleIfPresent(black_recent_arrivals_, black_stamp))
    {
      ++dropped_or_expired_black_arrivals_;
    }
  }

  void recordMaskArrival(
      MaskSource source,
      const Image::ConstSharedPtr &msg)
  {
    if (!enable_timing_log_)
    {
      return;
    }

    const TimePoint now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(timing_mutex_);
    if (source == MaskSource::White)
    {
      ++white_arrival_count_;
      white_recent_arrivals_.push_back({msg->header.stamp, now});
      while (white_recent_arrivals_.size() > kMaxRecentArrivalHistory)
      {
        white_recent_arrivals_.pop_front();
        ++dropped_or_expired_white_arrivals_;
      }
      return;
    }

    ++black_arrival_count_;
    black_recent_arrivals_.push_back({msg->header.stamp, now});
    while (black_recent_arrivals_.size() > kMaxRecentArrivalHistory)
    {
      black_recent_arrivals_.pop_front();
      ++dropped_or_expired_black_arrivals_;
    }
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

    long long white_arrived_before_black_us = 0;
    long long black_arrived_before_white_us = 0;
    long long pair_ready_to_callback_delay_us = 0;
    const long long callback_processing_us = elapsedUs(sync_callback_start, publish_end);
    long long ready_to_publish_us = 0;
    long long output_period_us = 0;
    std::uint64_t frame_index = 0;
    std::uint64_t white_arrival_count = 0;
    std::uint64_t black_arrival_count = 0;
    std::uint64_t published_pair_count = 0;
    std::uint64_t unmatched_white_arrivals = 0;
    std::uint64_t unmatched_black_arrivals = 0;
    std::uint64_t dropped_or_expired_white_arrivals = 0;
    std::uint64_t dropped_or_expired_black_arrivals = 0;
    std::uint64_t cv_bridge_error_count = 0;
    std::uint64_t size_mismatch_drop_count = 0;
    std::uint64_t frame_id_mismatch_drop_count = 0;
    std::uint64_t missing_arrival_lookup_count = 0;
    const long long white_stamp_ns =
        static_cast<long long>(white_stamp.sec) * 1000000000LL +
        static_cast<long long>(white_stamp.nanosec);
    const long long black_stamp_ns =
        static_cast<long long>(black_stamp.sec) * 1000000000LL +
        static_cast<long long>(black_stamp.nanosec);
    const long long stamp_delta_ns = std::llabs(white_stamp_ns - black_stamp_ns);

    {
      std::lock_guard<std::mutex> lock(timing_mutex_);
      TimePoint white_ready_time = sync_callback_start;
      TimePoint black_ready_time = sync_callback_start;
      const bool have_white_arrival = lookupArrivalTime(
          white_recent_arrivals_, white_stamp, white_ready_time);
      const bool have_black_arrival = lookupArrivalTime(
          black_recent_arrivals_, black_stamp, black_ready_time);

      if (!have_white_arrival || !have_black_arrival)
      {
        ++missing_arrival_lookup_count_;
      }

      // The pair becomes "ready" once both accepted messages have been seen,
      // which is the later of the two per-topic arrival callbacks.
      const TimePoint sync_ready_time =
          (have_white_arrival && have_black_arrival)
              ? (white_ready_time < black_ready_time ? black_ready_time : white_ready_time)
              : sync_callback_start;

      if (have_white_arrival && have_black_arrival)
      {
        if (white_ready_time < black_ready_time)
        {
          white_arrived_before_black_us = elapsedUs(white_ready_time, black_ready_time);
        }
        else if (black_ready_time < white_ready_time)
        {
          black_arrived_before_white_us = elapsedUs(black_ready_time, white_ready_time);
        }
      }

      // Time spent after both inputs were ready but before the synchronized
      // callback started running.
      pair_ready_to_callback_delay_us = elapsedUs(sync_ready_time, sync_callback_start);

      // Time from "both inputs ready" until publication finishes. This includes
      // both callback-start delay and actual callback processing.
      ready_to_publish_us = elapsedUs(sync_ready_time, publish_end);

      // Output period measures time between successful merged publications.
      // For the first successful pair there is no previous output, so log 0.
      if (has_previous_publish_end_)
      {
        output_period_us = elapsedUs(previous_publish_end_, publish_end);
      }

      discardArrivalSamplesUpTo(white_recent_arrivals_, white_stamp);
      discardArrivalSamplesUpTo(black_recent_arrivals_, black_stamp);
      frame_index = ++published_pair_count_;
      previous_publish_end_ = publish_end;
      has_previous_publish_end_ = true;
      white_arrival_count = white_arrival_count_;
      black_arrival_count = black_arrival_count_;
      published_pair_count = published_pair_count_;
      unmatched_white_arrivals = static_cast<std::uint64_t>(white_recent_arrivals_.size());
      unmatched_black_arrivals = static_cast<std::uint64_t>(black_recent_arrivals_.size());
      dropped_or_expired_white_arrivals = dropped_or_expired_white_arrivals_;
      dropped_or_expired_black_arrivals = dropped_or_expired_black_arrivals_;
      cv_bridge_error_count = cv_bridge_error_count_;
      size_mismatch_drop_count = size_mismatch_drop_count_;
      frame_id_mismatch_drop_count = frame_id_mismatch_drop_count_;
      missing_arrival_lookup_count = missing_arrival_lookup_count_;
    }

    if (frame_index % static_cast<std::uint64_t>(timing_log_interval_) != 0U)
    {
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Merged mask frame %llu timing: white_arrived_before_black_ms=%.3f, "
        "black_arrived_before_white_ms=%.3f, "
        "pair_ready_to_callback_delay_ms=%.3f, callback_processing_ms=%.3f, "
        "ready_to_publish_ms=%.3f, output_period_ms=%.3f, "
        "white_stamp=%s, black_stamp=%s, stamp_delta_ms=%.3f, "
        "counts={white_arrivals:%llu,black_arrivals:%llu,published_pairs:%llu,"
        "unmatched_white_arrivals:%llu,unmatched_black_arrivals:%llu,"
        "dropped_or_expired_white_arrivals:%llu,"
        "dropped_or_expired_black_arrivals:%llu,"
        "cv_bridge_errors:%llu,size_mismatch_drops:%llu,"
        "frame_id_mismatch_drops:%llu,missing_arrival_lookups:%llu}",
        static_cast<unsigned long long>(frame_index),
        static_cast<double>(white_arrived_before_black_us) / 1000.0,
        static_cast<double>(black_arrived_before_white_us) / 1000.0,
        static_cast<double>(pair_ready_to_callback_delay_us) / 1000.0,
        static_cast<double>(callback_processing_us) / 1000.0,
        static_cast<double>(ready_to_publish_us) / 1000.0,
        static_cast<double>(output_period_us) / 1000.0,
        formatStamp(white_stamp).c_str(),
        formatStamp(black_stamp).c_str(),
        static_cast<double>(stamp_delta_ns) / 1000000.0,
        static_cast<unsigned long long>(white_arrival_count),
        static_cast<unsigned long long>(black_arrival_count),
        static_cast<unsigned long long>(published_pair_count),
        static_cast<unsigned long long>(unmatched_white_arrivals),
        static_cast<unsigned long long>(unmatched_black_arrivals),
        static_cast<unsigned long long>(dropped_or_expired_white_arrivals),
        static_cast<unsigned long long>(dropped_or_expired_black_arrivals),
        static_cast<unsigned long long>(cv_bridge_error_count),
        static_cast<unsigned long long>(size_mismatch_drop_count),
        static_cast<unsigned long long>(frame_id_mismatch_drop_count),
        static_cast<unsigned long long>(missing_arrival_lookup_count));
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
  TimePoint previous_publish_end_{};
  bool has_previous_publish_end_ = false;
  std::deque<ArrivalSample> white_recent_arrivals_{};
  std::deque<ArrivalSample> black_recent_arrivals_{};
  std::uint64_t white_arrival_count_ = 0;
  std::uint64_t black_arrival_count_ = 0;
  std::uint64_t published_pair_count_ = 0;
  std::uint64_t dropped_or_expired_white_arrivals_ = 0;
  std::uint64_t dropped_or_expired_black_arrivals_ = 0;
  std::uint64_t cv_bridge_error_count_ = 0;
  std::uint64_t size_mismatch_drop_count_ = 0;
  std::uint64_t frame_id_mismatch_drop_count_ = 0;
  std::uint64_t missing_arrival_lookup_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclInputMaskMergeNode>());
  rclcpp::shutdown();
  return 0;
}
