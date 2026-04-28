#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rcj_localization/particle_filter.hpp"

class PfLocalizationNode : public rclcpp::Node {
public:
    PfLocalizationNode() : Node("pf_localization_node") {
        this->declare_parameter("num_particles", 1000);
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("yaw_topic", "/robot/yaw");
        this->declare_parameter<std::string>(
            "observations_topic",
            "/field_line_observations");

        num_particles_ = static_cast<int>(this->get_parameter("num_particles").as_int());
        map_topic_ = this->get_parameter("map_topic").as_string();
        yaw_topic_ = this->get_parameter("yaw_topic").as_string();
        observations_topic_ = this->get_parameter("observations_topic").as_string();

        pf_ = std::make_unique<rcj_loc::ParticleFilter>(num_particles_);
        pf_->initRandom(2.0, 3.0);

        yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            yaw_topic_,
            10,
            std::bind(&PfLocalizationNode::yawCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&PfLocalizationNode::mapCallback, this, std::placeholders::_1));
        observations_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            observations_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&PfLocalizationNode::observationsCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            10);
        particle_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PfLocalizationNode::filterLoop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "pf_localization_node started. map_topic='%s', yaw_topic='%s', "
            "observations_topic='%s', num_particles=%d",
            map_topic_.c_str(),
            yaw_topic_.c_str(),
            observations_topic_.c_str(),
            num_particles_);
    }

private:
    void yawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_rad_ = static_cast<double>(msg->data) * (M_PI / 180.0);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Map received. Building distance transform field...");
        pf_->setMap(msg);
        map_received_ = true;
    }

    void observationsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        latest_observations_.clear();
        latest_observations_.reserve(msg->poses.size());
        for (const auto &pose : msg->poses) {
            latest_observations_.push_back(
                {pose.position.x, pose.position.y});
        }
    }

    void filterLoop() {
        if (!map_received_) {
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

    int num_particles_ = 1000;
    std::string map_topic_;
    std::string yaw_topic_;
    std::string observations_topic_;
    double current_yaw_rad_ = 0.0;
    bool map_received_ = false;

    std::unique_ptr<rcj_loc::ParticleFilter> pf_;
    std::mutex obs_mutex_;
    std::vector<rcj_loc::Point2D> latest_observations_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr observations_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
