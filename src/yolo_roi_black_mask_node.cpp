#include <algorithm>
#include <memory>
#include <stdexcept>
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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rcj_localization/white_line_debug_utils.hpp"

namespace
{

  constexpr int kHueMax = 179;
  constexpr int kByteMax = 255;
  constexpr char kOverlayWindowName[] = "YOLO ROI Black Mask Overlay";

  bool isHueInRange(int value)
  {
    return value >= 0 && value <= kHueMax;
  }

  bool isByteInRange(int value)
  {
    return value >= 0 && value <= kByteMax;
  }

  cv::Mat thresholdBlackMask(
      const cv::Mat &hsv_image,
      int black_h_min,
      int black_h_max,
      int black_s_min,
      int black_s_max,
      int black_v_min,
      int black_v_max)
  {
    cv::Mat black_candidate;
    if (black_h_min <= black_h_max)
    {
      cv::inRange(
          hsv_image,
          cv::Scalar(black_h_min, black_s_min, black_v_min),
          cv::Scalar(black_h_max, black_s_max, black_v_max),
          black_candidate);
    }
    else
    {
      cv::Mat low_range_mask;
      cv::Mat high_range_mask;
      cv::inRange(
          hsv_image,
          cv::Scalar(0, black_s_min, black_v_min),
          cv::Scalar(black_h_max, black_s_max, black_v_max),
          low_range_mask);
      cv::inRange(
          hsv_image,
          cv::Scalar(black_h_min, black_s_min, black_v_min),
          cv::Scalar(kHueMax, black_s_max, black_v_max),
          high_range_mask);
      cv::bitwise_or(low_range_mask, high_range_mask, black_candidate);
    }
    return black_candidate;
  }

  cv::Mat normalizeBinaryMask(const cv::Mat &input_mask)
  {
    cv::Mat binary_mask;
    cv::compare(input_mask, 0, binary_mask, cv::CMP_GT);
    return binary_mask;
  }

  cv::Mat createOverlayImage(
      const cv::Mat &frame,
      const cv::Mat &roi_mask,
      const cv::Mat &roi_black_mask)
  {
    cv::Mat overlay =
        rcj_loc::vision::debug::createMaskOverlay(frame, roi_mask, cv::Scalar(255, 0, 0), 0.20);
    overlay = rcj_loc::vision::debug::createMaskOverlay(
        overlay,
        roi_black_mask,
        cv::Scalar(0, 0, 255),
        0.55);

    std::vector<cv::Point> roi_points;
    cv::findNonZero(roi_mask, roi_points);
    if (!roi_points.empty())
    {
      const cv::Rect roi_bounds = cv::boundingRect(roi_points);
      cv::rectangle(overlay, roi_bounds, cv::Scalar(0, 255, 255), 2);
    }

    return overlay;
  }

} // namespace

class YoloRoiBlackMaskNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using ExactSyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;

  YoloRoiBlackMaskNode()
      : Node("yolo_roi_black_mask")
  {
    declare_parameter<std::string>(
        "input_topic",
        "/black_feature_input_remap_node/image_remapped");
    declare_parameter<std::string>(
        "roi_mask_topic",
        "/yolo_black_circle_debug/roi_mask");
    declare_parameter("black_h_min", 0);
    declare_parameter("black_h_max", kHueMax);
    declare_parameter("black_s_min", 0);
    declare_parameter("black_s_max", kByteMax);
    declare_parameter("black_v_min", 0);
    declare_parameter("black_v_max", 70);
    declare_parameter("sync_queue_size", 60);
    declare_parameter("publish_debug_image", true);
    declare_parameter<std::string>("debug_image_topic", "~/debug/overlay_image");
    declare_parameter("enable_image_view", false);

    loadParameters();

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto roi_mask_topic = get_parameter("roi_mask_topic").as_string();

    image_sub_.subscribe(this, input_topic, rmw_qos_profile_sensor_data);
    roi_mask_sub_.subscribe(this, roi_mask_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
      ExactSyncPolicy(sync_queue_size_),
      image_sub_,
      roi_mask_sub_);
    sync_->registerCallback(
        std::bind(
            &YoloRoiBlackMaskNode::synchronizedCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    black_mask_pub_ = create_publisher<Image>("~/black_mask", 10);
    if (publish_debug_image_)
    {
      const auto debug_image_topic = get_parameter("debug_image_topic").as_string();
      debug_overlay_pub_ = create_publisher<Image>(debug_image_topic, 10);
    }

    if (enable_image_view_)
    {
      cv::namedWindow(kOverlayWindowName, cv::WINDOW_NORMAL);
      overlay_window_created_ = true;
    }

    parameter_callback_handle_ = add_on_set_parameters_callback(
        std::bind(
            &YoloRoiBlackMaskNode::handleParameterUpdates,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
      "yolo_roi_black_mask started. input_topic='%s', roi_mask_topic='%s', "
      "black_h_min=%d, black_h_max=%d, black_s_min=%d, black_s_max=%d, "
      "black_v_min=%d, black_v_max=%d, sync_queue_size=%d, "
      "publish_debug_image=%s, enable_image_view=%s",
      input_topic.c_str(),
      roi_mask_topic.c_str(),
      black_h_min_,
        black_h_max_,
      black_s_min_,
      black_s_max_,
      black_v_min_,
      black_v_max_,
      sync_queue_size_,
      publish_debug_image_ ? "true" : "false",
      enable_image_view_ ? "true" : "false");
  }

  ~YoloRoiBlackMaskNode() override
  {
    if (overlay_window_created_)
    {
      cv::destroyWindow(kOverlayWindowName);
    }
  }

private:
  void loadParameters()
  {
    black_h_min_ = requireHueParameter("black_h_min");
    black_h_max_ = requireHueParameter("black_h_max");
    black_s_min_ = requireByteParameter("black_s_min");
    black_s_max_ = requireByteParameter("black_s_max");
    black_v_min_ = requireByteParameter("black_v_min");
    black_v_max_ = requireByteParameter("black_v_max");
    sync_queue_size_ =
      std::max(1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
  }

  int requireHueParameter(const char *name) const
  {
    const int value = static_cast<int>(get_parameter(name).as_int());
    if (!isHueInRange(value))
    {
      throw std::runtime_error(
          std::string("Parameter '") + name + "' must be between 0 and " +
          std::to_string(kHueMax) + ".");
    }
    return value;
  }

  int requireByteParameter(const char *name) const
  {
    const int value = static_cast<int>(get_parameter(name).as_int());
    if (!isByteInRange(value))
    {
      throw std::runtime_error(
          std::string("Parameter '") + name + "' must be between 0 and " +
          std::to_string(kByteMax) + ".");
    }
    return value;
  }

  rcl_interfaces::msg::SetParametersResult handleParameterUpdates(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    int candidate_black_h_min = black_h_min_;
    int candidate_black_h_max = black_h_max_;
    int candidate_black_s_min = black_s_min_;
    int candidate_black_s_max = black_s_max_;
    int candidate_black_v_min = black_v_min_;
    int candidate_black_v_max = black_v_max_;

    for (const auto &parameter : parameters)
    {
      const auto &name = parameter.get_name();
      if (name != "black_h_min" && name != "black_h_max" &&
          name != "black_s_min" && name != "black_s_max" &&
          name != "black_v_min" && name != "black_v_max")
      {
        continue;
      }

      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        result.successful = false;
        result.reason = "Black HSV parameters must be integers.";
        return result;
      }

      const int value = static_cast<int>(parameter.as_int());
      if ((name == "black_h_min" || name == "black_h_max") && !isHueInRange(value))
      {
        result.successful = false;
        result.reason =
            std::string("Parameter '") + name + "' must be between 0 and " +
            std::to_string(kHueMax) + ".";
        return result;
      }
      if ((name == "black_s_min" || name == "black_s_max" ||
           name == "black_v_min" || name == "black_v_max") &&
          !isByteInRange(value))
      {
        result.successful = false;
        result.reason =
            std::string("Parameter '") + name + "' must be between 0 and " +
            std::to_string(kByteMax) + ".";
        return result;
      }

      if (name == "black_h_min")
      {
        candidate_black_h_min = value;
      }
      else if (name == "black_h_max")
      {
        candidate_black_h_max = value;
      }
      else if (name == "black_s_min")
      {
        candidate_black_s_min = value;
      }
      else if (name == "black_s_max")
      {
        candidate_black_s_max = value;
      }
      else if (name == "black_v_min")
      {
        candidate_black_v_min = value;
      }
      else if (name == "black_v_max")
      {
        candidate_black_v_max = value;
      }
    }

    black_h_min_ = candidate_black_h_min;
    black_h_max_ = candidate_black_h_max;
    black_s_min_ = candidate_black_s_min;
    black_s_max_ = candidate_black_s_max;
    black_v_min_ = candidate_black_v_min;
    black_v_max_ = candidate_black_v_max;
    return result;
  }

  void synchronizedCallback(
      const Image::ConstSharedPtr &image_msg,
      const Image::ConstSharedPtr &roi_mask_msg)
  {
    cv::Mat frame;
    cv::Mat roi_mask;
    try
    {
      frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      roi_mask = cv_bridge::toCvCopy(roi_mask_msg, "mono8")->image;
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

    if (frame.size() != roi_mask.size())
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Image/ROI size mismatch: image=%dx%d roi=%dx%d. Dropping synchronized pair.",
          frame.cols,
          frame.rows,
          roi_mask.cols,
          roi_mask.rows);
      return;
    }

    const cv::Mat roi_binary = normalizeBinaryMask(roi_mask);
    if (!logged_first_sync_) {
      RCLCPP_INFO(
        get_logger(),
        "Received first synchronized remapped image + ROI pair at %dx%d.",
        frame.cols,
        frame.rows);
      logged_first_sync_ = true;
    }

    cv::Mat hsv_image;
    cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);
    const cv::Mat black_candidate = thresholdBlackMask(
        hsv_image,
        black_h_min_,
        black_h_max_,
        black_s_min_,
        black_s_max_,
        black_v_min_,
        black_v_max_);

    cv::Mat roi_black_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    if (cv::countNonZero(roi_binary) > 0)
    {
      cv::bitwise_and(black_candidate, roi_binary, roi_black_mask);
    }

    black_mask_pub_->publish(
        *cv_bridge::CvImage(image_msg->header, "mono8", roi_black_mask).toImageMsg());

    if (publish_debug_image_ || enable_image_view_)
    {
      const cv::Mat overlay_image = createOverlayImage(frame, roi_binary, roi_black_mask);
      if (debug_overlay_pub_)
      {
        debug_overlay_pub_->publish(
            *cv_bridge::CvImage(image_msg->header, "bgr8", overlay_image).toImageMsg());
      }
      if (overlay_window_created_)
      {
        cv::imshow(kOverlayWindowName, overlay_image);
        cv::waitKey(1);
      }
    }
  }

  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Image> roi_mask_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_;

  rclcpp::Publisher<Image>::SharedPtr black_mask_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_overlay_pub_;

  int black_h_min_ = 0;
  int black_h_max_ = kHueMax;
  int black_s_min_ = 0;
  int black_s_max_ = kByteMax;
  int black_v_min_ = 0;
  int black_v_max_ = 70;
  int sync_queue_size_ = 60;
  bool publish_debug_image_ = true;
  bool enable_image_view_ = false;
  bool overlay_window_created_ = false;
  bool logged_first_sync_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloRoiBlackMaskNode>());
  rclcpp::shutdown();
  return 0;
}
