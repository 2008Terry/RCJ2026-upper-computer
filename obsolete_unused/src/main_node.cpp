#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <opencv2/opencv.hpp>
#include <mutex>

#include "rcj_localization/particle_filter.hpp"
#include "rcj_localization/vision_processor.hpp"



class RcjLocalizationNode : public rclcpp::Node
{
public:
    RcjLocalizationNode() : Node("rcj_localization_node")
    {
        this->declare_parameter("num_particles", 1000);
        num_particles_ = this->get_parameter("num_particles").as_int();

        // Initialize the Particle Filter
        pf_ = std::make_unique<rcj_loc::ParticleFilter>(num_particles_);
        // RCJ field is ~1.58m x ~2.19m. We initialize particles in a 2x3 meter box around the origin
        pf_->initRandom(2.0, 3.0); 

        // QoS Profile for Camera
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            std::bind(&RcjLocalizationNode::imageCallback, this, std::placeholders::_1));

        yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/robot/yaw", 10,
            std::bind(&RcjLocalizationNode::yawCallback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&RcjLocalizationNode::mapCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
        particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision_debug/image_raw", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/projected_lines", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Run the PF loop at 10 Hz (Every 100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RcjLocalizationNode::filterLoop, this));

        RCLCPP_INFO(this->get_logger(), "RCJ Localization Node initialized with %d particles.", num_particles_);
    }

private:
    int num_particles_;
    float current_yaw_ = 0.0;
    bool map_received_ = false;

    rcj_loc::vision::VisionProcessor vision_processor_;
    std::unique_ptr<rcj_loc::ParticleFilter> pf_;
    
    // Thread safety for sharing observations between camera callback and timer loop
    std::mutex obs_mutex_;
    std::vector<rcj_loc::Point2D> latest_observations_;

    // ROS 2 Objects
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void yawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_ = msg->data * (M_PI / 180.0);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Map received. Building distance transform field...");
        pf_->setMap(msg);
        map_received_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            return;
        }

        // 1. Run Vision Processor
        auto points = vision_processor_.extractFieldLines(frame);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Extracted Points: " << points.size());

        // 2. Safely store the points for the Particle Filter to use
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            // Convert from the VisionProcessor's Point2D to the ParticleFilter's Point2D
            latest_observations_.clear();
            for(const auto& p : points) {
                latest_observations_.push_back({p.x, p.y});
            }
        }

        // 3. Publish debug image
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_link";
        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        debug_image_pub_->publish(*debug_msg);

        // 4. Publish RViz 3D Marker
        visualization_msgs::msg::Marker p_marker;
        p_marker.header.frame_id = "base_link";
        p_marker.header.stamp = this->now();
        p_marker.ns = "ipm_projection";
        p_marker.id = 0;
        p_marker.type = visualization_msgs::msg::Marker::POINTS;
        p_marker.action = visualization_msgs::msg::Marker::ADD;
        p_marker.scale.x = 0.02; p_marker.scale.y = 0.02;
        p_marker.color.r = 0.0f; p_marker.color.g = 1.0f; p_marker.color.b = 1.0f; p_marker.color.a = 1.0f;
        
        for (const auto& p : points) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x; gp.y = p.y; gp.z = 0.0;
            p_marker.points.push_back(gp);
        }
        marker_pub_->publish(p_marker);
    }

    void filterLoop() {
        if (!map_received_) return;

        // 1. Get a safe copy of the latest camera observations
        std::vector<rcj_loc::Point2D> current_obs;
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            current_obs = latest_observations_;
        }

        // --- THE CORE AMCL MATH ---
        pf_->predict(current_yaw_);
        pf_->updateWeights(current_obs);
        pf_->resample();

        // Publish to RViz2
        publishVisualizationsAndTF();
    }

    void publishVisualizationsAndTF() {
        rclcpp::Time now = this->now();

        // --- A. Publish Real Particle Cloud ---
        geometry_msgs::msg::PoseArray cloud_msg;
        cloud_msg.header.stamp = now;
        cloud_msg.header.frame_id = "map";

        const auto& particles = pf_->getParticles();
        for(const auto& p : particles){
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y; 
            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            pose.orientation.x = q.x(); pose.orientation.y = q.y(); 
            pose.orientation.z = q.z(); pose.orientation.w = q.w();
            cloud_msg.poses.push_back(pose);
        }
        particle_pub_->publish(cloud_msg);

        // --- B. Publish Best Estimated Pose ---
        rcj_loc::Particle best = pf_->getBestPose();
        
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = best.x;
        pose_msg.pose.pose.position.y = best.y;
        tf2::Quaternion q_best;
        q_best.setRPY(0, 0, best.theta);
        pose_msg.pose.pose.orientation.x = q_best.x(); pose_msg.pose.pose.orientation.y = q_best.y(); 
        pose_msg.pose.pose.orientation.z = q_best.z(); pose_msg.pose.pose.orientation.w = q_best.w();
        pose_pub_->publish(pose_msg);

        // --- C. Broadcast TF (map -> base_link) ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = best.x;
        t.transform.translation.y = best.y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = pose_msg.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RcjLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
