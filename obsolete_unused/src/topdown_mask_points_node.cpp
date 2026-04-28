#include <algorithm>
#include <cmath>
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
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>

namespace {

struct AxisMapping {
    char axis = '\0';
    double sign = 0.0;
};

AxisMapping parseAxisMapping(const std::string &value) {
    if (value == "u+") {
        return {'u', 1.0};
    }
    if (value == "u-") {
        return {'u', -1.0};
    }
    if (value == "v+") {
        return {'v', 1.0};
    }
    if (value == "v-") {
        return {'v', -1.0};
    }

    throw std::runtime_error(
        "Invalid axis mapping '" + value +
        "'. Supported values: u+, u-, v+, v-.");
}

double mapAxisValue(const AxisMapping &mapping, double du, double dv) {
    const double base_value = mapping.axis == 'u' ? du : dv;
    return mapping.sign * base_value;
}

std::size_t selectSampleIndex(
    std::size_t sample_index,
    std::size_t sample_count,
    std::size_t total_count) {
    if (sample_count == 0 || total_count == 0) {
        return 0;
    }
    if (sample_count >= total_count) {
        return sample_index;
    }
    if (sample_count == 1) {
        return 0;
    }

    const double ratio =
        static_cast<double>(sample_index) / static_cast<double>(sample_count - 1);
    return static_cast<std::size_t>(std::llround(ratio * static_cast<double>(total_count - 1)));
}

}  // namespace

class TopdownMaskPointsNode : public rclcpp::Node {
public:
    TopdownMaskPointsNode() : Node("topdown_mask_points_node") {
        this->declare_parameter<std::string>(
            "mask_topic",
            "/white_line_skeleton_filter_node/white_final_mask");
        this->declare_parameter<std::string>(
            "output_topic",
            "/field_line_observations");
        this->declare_parameter("meters_per_pixel", 0.0025);
        this->declare_parameter<std::string>("forward_axis", "__unset__");
        this->declare_parameter<std::string>("left_axis", "__unset__");
        this->declare_parameter("max_points", 5000);

        loadAndValidateParameters();

        points_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
        mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            mask_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TopdownMaskPointsNode::maskCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "topdown_mask_points_node started. mask_topic='%s', output_topic='%s', "
            "meters_per_pixel=%.6f, origin=image_center, forward_axis='%s', left_axis='%s', "
            "max_points=%d",
            mask_topic_.c_str(),
            output_topic_.c_str(),
            meters_per_pixel_,
            forward_axis_name_.c_str(),
            left_axis_name_.c_str(),
            max_points_);
    }

private:
    void loadAndValidateParameters() {
        mask_topic_ = this->get_parameter("mask_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        meters_per_pixel_ = this->get_parameter("meters_per_pixel").as_double();
        forward_axis_name_ = this->get_parameter("forward_axis").as_string();
        left_axis_name_ = this->get_parameter("left_axis").as_string();
        max_points_ = std::max(1, static_cast<int>(this->get_parameter("max_points").as_int()));

        if (!std::isfinite(meters_per_pixel_) || meters_per_pixel_ <= 0.0) {
            throw std::runtime_error(
                "Parameter 'meters_per_pixel' must be a positive finite number.");
        }

        forward_axis_ = parseAxisMapping(forward_axis_name_);
        left_axis_ = parseAxisMapping(left_axis_name_);
        if (forward_axis_.axis == left_axis_.axis) {
            throw std::runtime_error(
                "Parameters 'forward_axis' and 'left_axis' must be orthogonal and cannot "
                "reuse the same pixel axis.");
        }
    }

    void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv::Mat mask;
        try {
            mask = cv_bridge::toCvCopy(msg, "mono8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cv_bridge failed while reading topdown mask: %s",
                e.what());
            return;
        }

        geometry_msgs::msg::PoseArray observations;
        observations.header = msg->header;
        observations.header.frame_id = "base_link";

        std::vector<cv::Point> active_pixels;
        cv::findNonZero(mask, active_pixels);
        if (active_pixels.empty()) {
            points_pub_->publish(observations);
            return;
        }

        const std::size_t sample_count = std::min<std::size_t>(active_pixels.size(), max_points_);
        observations.poses.reserve(sample_count);
        const double robot_origin_u_px =
            (static_cast<double>(mask.cols) - 1.0) * 0.5;
        const double robot_origin_v_px =
            (static_cast<double>(mask.rows) - 1.0) * 0.5;

        for (std::size_t i = 0; i < sample_count; ++i) {
            const cv::Point &pixel =
                active_pixels[selectSampleIndex(i, sample_count, active_pixels.size())];
            const double u = static_cast<double>(pixel.x);
            const double v = static_cast<double>(pixel.y);
            const double du = (u - robot_origin_u_px) * meters_per_pixel_;
            const double dv = (v - robot_origin_v_px) * meters_per_pixel_;

            geometry_msgs::msg::Pose pose;
            pose.position.x = mapAxisValue(forward_axis_, du, dv);
            pose.position.y = mapAxisValue(left_axis_, du, dv);
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            observations.poses.push_back(pose);
        }

        points_pub_->publish(observations);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr points_pub_;

    std::string mask_topic_;
    std::string output_topic_;
    double meters_per_pixel_ = -1.0;
    std::string forward_axis_name_;
    std::string left_axis_name_;
    AxisMapping forward_axis_;
    AxisMapping left_axis_;
    int max_points_ = 5000;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopdownMaskPointsNode>());
    rclcpp::shutdown();
    return 0;
}
