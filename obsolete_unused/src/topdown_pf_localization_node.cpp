#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core.hpp>

#include "rcj_localization/particle_filter.hpp"

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
    return static_cast<std::size_t>(
        std::llround(ratio * static_cast<double>(total_count - 1)));
}

}  // namespace

class TopdownPfLocalizationNode : public rclcpp::Node {
public:
    TopdownPfLocalizationNode() : Node("topdown_pf_localization_node") {
        this->declare_parameter<std::string>(
            "mask_topic",
            "/white_line_skeleton_filter_node/white_final_mask");
        this->declare_parameter("meters_per_pixel", 0.0025);
        this->declare_parameter<std::string>("forward_axis", "v+");
        this->declare_parameter<std::string>("left_axis", "u-");
        this->declare_parameter("max_points", 5000);
        this->declare_parameter("enable_localization", true);
        this->declare_parameter("publish_debug_pointcloud", false);
        this->declare_parameter<std::string>(
            "debug_pointcloud_topic",
            "/field_line_observations_debug");
        this->declare_parameter("num_particles", 1000);
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("yaw_topic", "/robot/yaw");

        loadAndValidateParameters();

        mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            mask_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TopdownPfLocalizationNode::maskCallback, this, std::placeholders::_1));

        if (publish_debug_pointcloud_) {
            debug_pointcloud_pub_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    debug_pointcloud_topic_,
                    10);
        }

        if (enable_localization_) {
            pf_ = std::make_unique<rcj_loc::ParticleFilter>(num_particles_);
            pf_->initRandom(2.0, 3.0);

            yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                yaw_topic_,
                10,
                std::bind(&TopdownPfLocalizationNode::yawCallback, this, std::placeholders::_1));
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
                std::bind(&TopdownPfLocalizationNode::mapCallback, this, std::placeholders::_1));

            pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/amcl_pose",
                    10);
            particle_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&TopdownPfLocalizationNode::filterLoop, this));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "topdown_pf_localization_node started. mask_topic='%s', "
            "meters_per_pixel=%.6f, forward_axis='%s', left_axis='%s', max_points=%d, "
            "enable_localization=%s, publish_debug_pointcloud=%s, debug_pointcloud_topic='%s'",
            mask_topic_.c_str(),
            meters_per_pixel_,
            forward_axis_name_.c_str(),
            left_axis_name_.c_str(),
            max_points_,
            enable_localization_ ? "true" : "false",
            publish_debug_pointcloud_ ? "true" : "false",
            debug_pointcloud_topic_.c_str());
    }

private:
    void loadAndValidateParameters() {
        mask_topic_ = this->get_parameter("mask_topic").as_string();
        meters_per_pixel_ = this->get_parameter("meters_per_pixel").as_double();
        forward_axis_name_ = this->get_parameter("forward_axis").as_string();
        left_axis_name_ = this->get_parameter("left_axis").as_string();
        max_points_ = std::max(1, static_cast<int>(this->get_parameter("max_points").as_int()));
        enable_localization_ = this->get_parameter("enable_localization").as_bool();
        publish_debug_pointcloud_ = this->get_parameter("publish_debug_pointcloud").as_bool();
        debug_pointcloud_topic_ = this->get_parameter("debug_pointcloud_topic").as_string();
        num_particles_ = static_cast<int>(this->get_parameter("num_particles").as_int());
        map_topic_ = this->get_parameter("map_topic").as_string();
        yaw_topic_ = this->get_parameter("yaw_topic").as_string();

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

    void yawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_rad_ = static_cast<double>(msg->data) * (M_PI / 180.0);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Map received. Building distance transform field...");
        pf_->setMap(msg);
        map_received_ = true;
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

        std::vector<cv::Point> active_pixels;
        cv::findNonZero(mask, active_pixels);

        std::vector<rcj_loc::Point2D> observations;
        if (!active_pixels.empty()) {
            const std::size_t sample_count =
                std::min<std::size_t>(active_pixels.size(), max_points_);
            observations.reserve(sample_count);

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

                observations.push_back(
                    {mapAxisValue(forward_axis_, du, dv), mapAxisValue(left_axis_, du, dv)});
            }
        }

        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            latest_observations_ = observations;
        }

        if (publish_debug_pointcloud_) {
            publishDebugPointCloud(msg->header, observations);
        }
    }

    void publishDebugPointCloud(
        const std_msgs::msg::Header &header,
        const std::vector<rcj_loc::Point2D> &observations) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        cloud.header.frame_id = "base_link";

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(observations.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto &observation : observations) {
            *iter_x = static_cast<float>(observation.x);
            *iter_y = static_cast<float>(observation.y);
            *iter_z = 0.0f;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        debug_pointcloud_pub_->publish(cloud);
    }

    void filterLoop() {
        if (!enable_localization_ || !map_received_) {
            return;
        }

        std::vector<rcj_loc::Point2D> current_observations;
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            current_observations = latest_observations_;
        }

        pf_->predict(current_yaw_rad_);
        pf_->updateWeights(current_observations);
        pf_->resample();
        publishVisualizationsAndTF();
    }

    void publishVisualizationsAndTF() {
        const rclcpp::Time now = this->now();

        geometry_msgs::msg::PoseArray cloud_msg;
        cloud_msg.header.stamp = now;
        cloud_msg.header.frame_id = "map";

        const auto &particles = pf_->getParticles();
        cloud_msg.poses.reserve(particles.size());
        for (const auto &particle : particles) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = particle.x;
            pose.position.y = particle.y;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, particle.theta);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            cloud_msg.poses.push_back(pose);
        }
        particle_pub_->publish(cloud_msg);

        const rcj_loc::Particle best = pf_->getBestPose();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = best.x;
        pose_msg.pose.pose.position.y = best.y;

        tf2::Quaternion q_best;
        q_best.setRPY(0.0, 0.0, best.theta);
        pose_msg.pose.pose.orientation.x = q_best.x();
        pose_msg.pose.pose.orientation.y = q_best.y();
        pose_msg.pose.pose.orientation.z = q_best.z();
        pose_msg.pose.pose.orientation.w = q_best.w();
        pose_pub_->publish(pose_msg);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = best.x;
        transform.transform.translation.y = best.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = pose_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex obs_mutex_;
    std::vector<rcj_loc::Point2D> latest_observations_;
    std::unique_ptr<rcj_loc::ParticleFilter> pf_;

    std::string mask_topic_;
    double meters_per_pixel_ = -1.0;
    std::string forward_axis_name_;
    std::string left_axis_name_;
    AxisMapping forward_axis_;
    AxisMapping left_axis_;
    int max_points_ = 5000;
    bool enable_localization_ = true;
    bool publish_debug_pointcloud_ = false;
    std::string debug_pointcloud_topic_;
    int num_particles_ = 1000;
    std::string map_topic_;
    std::string yaw_topic_;
    double current_yaw_rad_ = 0.0;
    bool map_received_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopdownPfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
