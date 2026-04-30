#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/core.hpp>

#include "rcj_localization/particle_filter_amcl_fusion.hpp"
#include "rcj_localization/srv/stm32_command.hpp"

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

  throw std::runtime_error("Invalid axis mapping '" + value +
                           "'. Supported values: u+, u-, v+, v-.");
}

double mapAxisValue(const AxisMapping &mapping, double du, double dv) {
  const double base_value = mapping.axis == 'u' ? du : dv;
  return mapping.sign * base_value;
}

std::size_t selectSampleIndex(std::size_t sample_index,
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

bool isValidDistanceTransformMaskSize(int value) {
  return value == 3 || value == 5;
}

double normalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double degreesToRadians(double angle_deg) { return angle_deg * (M_PI / 180.0); }

double radiansToDegrees(double angle_rad) { return angle_rad * (180.0 / M_PI); }

double fieldYawDegreesToRosMapRadians(double yaw_degrees,
                                      double zero_map_degrees) {
  // Robot yaw is field-relative: 0=top/forward, 90=left, 180=back,
  // 270=right. ROS map yaw is 0=+X/right, 90=+Y/top.
  return normalizeAngle(degreesToRadians(90.0 + zero_map_degrees + yaw_degrees));
}

double stm32XAxisDegreesToRosMapRadians(double zero_map_degrees) {
  // STM32 dx is field-fixed map-left, independent of the robot's current yaw.
  // With zero_map_degrees=0 this is ROS map 180 deg.
  return normalizeAngle(degreesToRadians(180.0 + zero_map_degrees));
}

void validateAxisMotionNoiseConfig(const rcj_loc::AxisMotionNoiseConfig &config,
                                   const std::string &prefix) {
  const auto validate = [&](double value, const std::string &suffix) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::runtime_error("Parameter '" + prefix + suffix +
                               "' must be a non-negative finite number.");
    }
  };

  validate(config.from_x, "_from_x");
  validate(config.from_y, "_from_y");
  validate(config.from_theta, "_from_theta");
  validate(config.bias, "_bias");
}

} // namespace

class AmclFusionNode : public rclcpp::Node {
public:
  using Stm32Command = rcj_localization::srv::Stm32Command;
  using Stm32CommandFuture = rclcpp::Client<Stm32Command>::SharedFuture;

  AmclFusionNode() : Node("amcl_fusion") {
    declareParameters();
    loadAndValidateParameters();

    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AmclFusionNode::maskCallback, this, std::placeholders::_1));

    if (publish_debug_pointcloud_) {
      debug_pointcloud_pub_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>(
              debug_pointcloud_topic_, 10);
    }

    if (publish_processing_time_) {
      processing_time_pub_ = this->create_publisher<std_msgs::msg::Float32>(
          processing_time_topic_, 10);
    }

    if (enable_localization_) {
      pf_ = std::make_unique<rcj_loc::ParticleFilterAmclFusion>(filter_config_);

      if (use_fake_yaw_) {
        current_yaw_rad_ =
            fieldYawDegreesToRosMapRadians(fake_yaw_degrees_,
                                           yaw_zero_map_degrees_);
        yaw_initialized_ = true;
        fake_yaw_pub_ =
            this->create_publisher<std_msgs::msg::Float32>(yaw_topic_, 10);
        fake_yaw_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AmclFusionNode::publishFakeYaw, this));
        publishFakeYaw();
        RCLCPP_INFO(this->get_logger(),
                    "Using fixed fake yaw: %.3f degrees on '%s'.",
                    fake_yaw_degrees_, yaw_topic_.c_str());
      } else {
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            yaw_topic_, 10,
            std::bind(&AmclFusionNode::yawCallback, this,
                      std::placeholders::_1));
      }
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
          std::bind(&AmclFusionNode::mapCallback, this, std::placeholders::_1));
      if (use_stm32_gateway_odometry_) {
        stm32_command_client_ =
            this->create_client<Stm32Command>(stm32_command_service_);
      }

      pose_pub_ =
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "/amcl_pose", 10);
      particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/particlecloud", 10);
      if (publish_particle_weight_markers_) {
        particle_weight_marker_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                particle_weight_marker_topic_, 10);
      }
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      recreateTimer();
    }

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &AmclFusionNode::handleParameterUpdates, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "amcl_fusion started. mask_topic='%s', yaw_topic='%s', "
        "use_fake_yaw=%s, fake_yaw_degrees=%.3f, "
        "yaw_zero_map_degrees=%.3f, "
        "use_stm32_gateway_odometry=%s, stm32_command_service='%s', "
        "stm32_request_timeout_ms=%d, stm32_enable_odometry_log=%s, "
        "use_stm32_request_theta=%s, "
        "meters_per_pixel=%.6f, "
        "forward_axis='%s', "
        "left_axis='%s', max_points=%d, num_particles=%d, sigma_hit=%.3f, "
        "use_weighted_mean_pose=%s, noise_xy=%.3f, noise_theta=%.3f, "
        "publish_particle_weight_markers=%s, "
        "particle_weight_marker_topic='%s', particle_weight_marker_scale=%.3f, "
        "filter_period_ms=%d, "
        "publish_processing_time=%s, processing_time_topic='%s', "
        "enable_timing_log=%s, timing_log_interval=%d",
        mask_topic_.c_str(), yaw_topic_.c_str(),
        use_fake_yaw_ ? "true" : "false", fake_yaw_degrees_,
        yaw_zero_map_degrees_,
        use_stm32_gateway_odometry_ ? "true" : "false",
        stm32_command_service_.c_str(), stm32_request_timeout_ms_,
        stm32_enable_odometry_log_ ? "true" : "false",
        use_stm32_request_theta_ ? "true" : "false",
        meters_per_pixel_, forward_axis_name_.c_str(), left_axis_name_.c_str(),
        max_points_, filter_config_.num_particles, filter_config_.sigma_hit,
        use_weighted_mean_pose_ ? "true" : "false",
        filter_config_.noise_xy, filter_config_.noise_theta,
        publish_particle_weight_markers_ ? "true" : "false",
        particle_weight_marker_topic_.c_str(), particle_weight_marker_scale_,
        filter_period_ms_,
        publish_processing_time_ ? "true" : "false",
        processing_time_topic_.c_str(), enable_timing_log_ ? "true" : "false",
        timing_log_interval_);
  }

private:
  struct GatewayOdomResult {
    std::uint64_t request_id = 0;
    bool success = false;
    std::string status;
    std::string message;
    double dx_cm = 0.0;
    double dy_cm = 0.0;
    double dtheta_deg = 0.0;
    double theta_deg = 0.0;
    double theta_rad = 0.0;
    bool has_theta = false;
    double request_yaw_rad = 0.0;
    std::uint64_t total_request_count = 0;
    std::uint64_t successful_request_count = 0;
    double success_rate_percent = 0.0;
    std::optional<rcj_loc::Particle> request_pose;
  };

  void declareParameters() {
    this->declare_parameter<std::string>(
        "mask_topic", "/white_line_dt_ridge_filter_node/white_final_mask");
    this->declare_parameter("meters_per_pixel", 0.0025);
    this->declare_parameter<std::string>("forward_axis", "v-");
    this->declare_parameter<std::string>("left_axis", "u-");
    this->declare_parameter("max_points", 5000);
    this->declare_parameter("enable_localization", true);
    this->declare_parameter("publish_debug_pointcloud", false);
    this->declare_parameter<std::string>("debug_pointcloud_topic",
                                         "/field_line_observations_debug");
    this->declare_parameter("use_weighted_mean_pose", false);
    this->declare_parameter("publish_particle_weight_markers", false);
    this->declare_parameter<std::string>("particle_weight_marker_topic",
                                         "/particle_weights");
    this->declare_parameter("particle_weight_marker_scale", 0.035);
    this->declare_parameter("num_particles", 1000);
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("yaw_topic", "/robot/yaw");
    this->declare_parameter("use_fake_yaw", false);
    this->declare_parameter("fake_yaw_degrees", 0.0);
    this->declare_parameter("yaw_zero_map_degrees", 0.0);
    this->declare_parameter("use_stm32_gateway_odometry", false);
    this->declare_parameter<std::string>("stm32_command_service",
                                         "/stm32/send_command");
    this->declare_parameter("stm32_request_timeout_ms", 200);
    this->declare_parameter("stm32_enable_odometry_log", false);
    this->declare_parameter("use_stm32_request_theta", false);
    this->declare_parameter("enable_global_search", true);
    this->declare_parameter("global_search_random_ratio", 0.50);
    this->declare_parameter("global_search_noise_xy", 0.12);
    this->declare_parameter("global_search_noise_theta", 0.20);
    this->declare_parameter("localized_xy_std_threshold", 0.20);
    this->declare_parameter("localized_theta_std_threshold", 0.35);
    this->declare_parameter("localized_min_updates", 5);
    this->declare_parameter("lost_alpha_ratio_threshold", 0.45);
    this->declare_parameter("lost_min_updates", 3);

    this->declare_parameter("sigma_hit", 0.10);
    this->declare_parameter("noise_xy", 0.05);
    this->declare_parameter("noise_theta", 0.10);
    this->declare_parameter("alpha_fast_rate", 0.1);
    this->declare_parameter("alpha_slow_rate", 0.001);
    this->declare_parameter("random_injection_max_ratio", 0.25);
    this->declare_parameter("off_map_penalty", 1.0);
    this->declare_parameter("occupancy_threshold", 50);
    this->declare_parameter("distance_transform_mask_size", 5);
    this->declare_parameter("init_field_width", 2.0);
    this->declare_parameter("init_field_height", 3.0);
    this->declare_parameter("odom_noise_x_from_x", 0.08);
    this->declare_parameter("odom_noise_x_from_y", 0.02);
    this->declare_parameter("odom_noise_x_from_theta", 0.0025);
    this->declare_parameter("odom_noise_x_bias", 0.000064);
    this->declare_parameter("odom_noise_y_from_x", 0.02);
    this->declare_parameter("odom_noise_y_from_y", 0.16);
    this->declare_parameter("odom_noise_y_from_theta", 0.0049);
    this->declare_parameter("odom_noise_y_bias", 0.000144);
    this->declare_parameter("odom_noise_theta_from_x", 0.30);
    this->declare_parameter("odom_noise_theta_from_y", 0.60);
    this->declare_parameter("odom_noise_theta_from_theta", 0.09);
    this->declare_parameter("odom_noise_theta_bias", 0.000304617);
    this->declare_parameter("filter_period_ms", 100);
    this->declare_parameter("publish_processing_time", true);
    this->declare_parameter<std::string>("processing_time_topic",
                                         "~/processing_time_ms");
    this->declare_parameter("enable_timing_log", true);
    this->declare_parameter("timing_log_interval", 30);
  }

  void loadAndValidateParameters() {
    mask_topic_ = this->get_parameter("mask_topic").as_string();
    meters_per_pixel_ = this->get_parameter("meters_per_pixel").as_double();
    forward_axis_name_ = this->get_parameter("forward_axis").as_string();
    left_axis_name_ = this->get_parameter("left_axis").as_string();
    max_points_ = std::max(
        1, static_cast<int>(this->get_parameter("max_points").as_int()));
    enable_localization_ = this->get_parameter("enable_localization").as_bool();
    publish_debug_pointcloud_ =
        this->get_parameter("publish_debug_pointcloud").as_bool();
    debug_pointcloud_topic_ =
        this->get_parameter("debug_pointcloud_topic").as_string();
    use_weighted_mean_pose_ =
        this->get_parameter("use_weighted_mean_pose").as_bool();
    publish_particle_weight_markers_ =
        this->get_parameter("publish_particle_weight_markers").as_bool();
    particle_weight_marker_topic_ =
        this->get_parameter("particle_weight_marker_topic").as_string();
    particle_weight_marker_scale_ =
        this->get_parameter("particle_weight_marker_scale").as_double();
    map_topic_ = this->get_parameter("map_topic").as_string();
    yaw_topic_ = this->get_parameter("yaw_topic").as_string();
    use_fake_yaw_ = this->get_parameter("use_fake_yaw").as_bool();
    fake_yaw_degrees_ = this->get_parameter("fake_yaw_degrees").as_double();
    yaw_zero_map_degrees_ =
        this->get_parameter("yaw_zero_map_degrees").as_double();
    if (!std::isfinite(fake_yaw_degrees_)) {
      throw std::runtime_error(
          "Parameter 'fake_yaw_degrees' must be a finite number.");
    }
    if (!std::isfinite(yaw_zero_map_degrees_)) {
      throw std::runtime_error(
          "Parameter 'yaw_zero_map_degrees' must be a finite number.");
    }
    if (publish_particle_weight_markers_ &&
        particle_weight_marker_topic_.empty()) {
      throw std::runtime_error(
          "Parameter 'particle_weight_marker_topic' must not be empty when "
          "'publish_particle_weight_markers' is true.");
    }
    if (!std::isfinite(particle_weight_marker_scale_) ||
        particle_weight_marker_scale_ <= 0.0) {
      throw std::runtime_error(
          "Parameter 'particle_weight_marker_scale' must be a positive finite "
          "number.");
    }
    use_stm32_gateway_odometry_ =
        this->get_parameter("use_stm32_gateway_odometry").as_bool();
    stm32_command_service_ =
        this->get_parameter("stm32_command_service").as_string();
    stm32_request_timeout_ms_ = static_cast<int>(
        this->get_parameter("stm32_request_timeout_ms").as_int());
    stm32_enable_odometry_log_ =
        this->get_parameter("stm32_enable_odometry_log").as_bool();
    use_stm32_request_theta_ =
        this->get_parameter("use_stm32_request_theta").as_bool();
    if (use_stm32_gateway_odometry_ && stm32_command_service_.empty()) {
      throw std::runtime_error(
          "Parameter 'stm32_command_service' must not be empty when "
          "'use_stm32_gateway_odometry' is true.");
    }
    enable_global_search_ =
        this->get_parameter("enable_global_search").as_bool();
    global_search_random_ratio_ =
        this->get_parameter("global_search_random_ratio").as_double();
    global_search_noise_xy_ =
        this->get_parameter("global_search_noise_xy").as_double();
    global_search_noise_theta_ =
        this->get_parameter("global_search_noise_theta").as_double();
    localized_xy_std_threshold_ =
        this->get_parameter("localized_xy_std_threshold").as_double();
    localized_theta_std_threshold_ =
        this->get_parameter("localized_theta_std_threshold").as_double();
    localized_min_updates_ = static_cast<int>(
        this->get_parameter("localized_min_updates").as_int());
    lost_alpha_ratio_threshold_ =
        this->get_parameter("lost_alpha_ratio_threshold").as_double();
    lost_min_updates_ =
        static_cast<int>(this->get_parameter("lost_min_updates").as_int());
    global_search_active_ = enable_global_search_;

    filter_config_.num_particles =
        static_cast<int>(this->get_parameter("num_particles").as_int());
    filter_config_.sigma_hit = this->get_parameter("sigma_hit").as_double();
    filter_config_.noise_xy = this->get_parameter("noise_xy").as_double();
    filter_config_.noise_theta = this->get_parameter("noise_theta").as_double();
    filter_config_.alpha_fast_rate =
        this->get_parameter("alpha_fast_rate").as_double();
    filter_config_.alpha_slow_rate =
        this->get_parameter("alpha_slow_rate").as_double();
    filter_config_.random_injection_max_ratio =
        this->get_parameter("random_injection_max_ratio").as_double();
    filter_config_.off_map_penalty =
        this->get_parameter("off_map_penalty").as_double();
    filter_config_.occupancy_threshold =
        static_cast<int>(this->get_parameter("occupancy_threshold").as_int());
    filter_config_.distance_transform_mask_size = static_cast<int>(
        this->get_parameter("distance_transform_mask_size").as_int());
    filter_config_.init_field_width =
        this->get_parameter("init_field_width").as_double();
    filter_config_.init_field_height =
        this->get_parameter("init_field_height").as_double();
    filter_config_.odom_noise_x.from_x =
        this->get_parameter("odom_noise_x_from_x").as_double();
    filter_config_.odom_noise_x.from_y =
        this->get_parameter("odom_noise_x_from_y").as_double();
    filter_config_.odom_noise_x.from_theta =
        this->get_parameter("odom_noise_x_from_theta").as_double();
    filter_config_.odom_noise_x.bias =
        this->get_parameter("odom_noise_x_bias").as_double();
    filter_config_.odom_noise_y.from_x =
        this->get_parameter("odom_noise_y_from_x").as_double();
    filter_config_.odom_noise_y.from_y =
        this->get_parameter("odom_noise_y_from_y").as_double();
    filter_config_.odom_noise_y.from_theta =
        this->get_parameter("odom_noise_y_from_theta").as_double();
    filter_config_.odom_noise_y.bias =
        this->get_parameter("odom_noise_y_bias").as_double();
    filter_config_.odom_noise_theta.from_x =
        this->get_parameter("odom_noise_theta_from_x").as_double();
    filter_config_.odom_noise_theta.from_y =
        this->get_parameter("odom_noise_theta_from_y").as_double();
    filter_config_.odom_noise_theta.from_theta =
        this->get_parameter("odom_noise_theta_from_theta").as_double();
    filter_config_.odom_noise_theta.bias =
        this->get_parameter("odom_noise_theta_bias").as_double();
    filter_period_ms_ =
        static_cast<int>(this->get_parameter("filter_period_ms").as_int());
    publish_processing_time_ =
        this->get_parameter("publish_processing_time").as_bool();
    processing_time_topic_ =
        this->get_parameter("processing_time_topic").as_string();
    enable_timing_log_ = this->get_parameter("enable_timing_log").as_bool();
    timing_log_interval_ =
        static_cast<int>(this->get_parameter("timing_log_interval").as_int());

    validateConfig(meters_per_pixel_, forward_axis_name_, left_axis_name_,
                   max_points_, filter_config_, stm32_request_timeout_ms_,
                   filter_period_ms_, timing_log_interval_,
                   processing_time_topic_);
    validateGlobalSearchConfig(
        global_search_random_ratio_, global_search_noise_xy_,
        global_search_noise_theta_, localized_xy_std_threshold_,
        localized_theta_std_threshold_, localized_min_updates_,
        lost_alpha_ratio_threshold_, lost_min_updates_);

    forward_axis_ = parseAxisMapping(forward_axis_name_);
    left_axis_ = parseAxisMapping(left_axis_name_);
  }

  void validateConfig(double meters_per_pixel,
                      const std::string &forward_axis_name,
                      const std::string &left_axis_name, int max_points,
                      const rcj_loc::ParticleFilterAmclFusionConfig &config,
                      int stm32_request_timeout_ms, int filter_period_ms,
                      int timing_log_interval,
                      const std::string &processing_time_topic) const {
    if (!std::isfinite(meters_per_pixel) || meters_per_pixel <= 0.0) {
      throw std::runtime_error(
          "Parameter 'meters_per_pixel' must be a positive finite number.");
    }
    if (max_points < 1) {
      throw std::runtime_error("Parameter 'max_points' must be at least 1.");
    }
    if (config.num_particles < 1) {
      throw std::runtime_error("Parameter 'num_particles' must be at least 1.");
    }
    if (!std::isfinite(config.sigma_hit) || config.sigma_hit <= 0.0) {
      throw std::runtime_error(
          "Parameter 'sigma_hit' must be a positive finite number.");
    }
    if (!std::isfinite(config.noise_xy) || config.noise_xy < 0.0) {
      throw std::runtime_error(
          "Parameter 'noise_xy' must be a non-negative finite number.");
    }
    if (!std::isfinite(config.noise_theta) || config.noise_theta < 0.0) {
      throw std::runtime_error(
          "Parameter 'noise_theta' must be a non-negative finite number.");
    }
    if (!std::isfinite(config.alpha_fast_rate) ||
        config.alpha_fast_rate < 0.0 || config.alpha_fast_rate > 1.0) {
      throw std::runtime_error(
          "Parameter 'alpha_fast_rate' must be between 0 and 1.");
    }
    if (!std::isfinite(config.alpha_slow_rate) ||
        config.alpha_slow_rate < 0.0 || config.alpha_slow_rate > 1.0) {
      throw std::runtime_error(
          "Parameter 'alpha_slow_rate' must be between 0 and 1.");
    }
    if (!std::isfinite(config.random_injection_max_ratio) ||
        config.random_injection_max_ratio < 0.0 ||
        config.random_injection_max_ratio > 1.0) {
      throw std::runtime_error(
          "Parameter 'random_injection_max_ratio' must be between 0 and 1.");
    }
    if (!std::isfinite(config.off_map_penalty) ||
        config.off_map_penalty < 0.0) {
      throw std::runtime_error(
          "Parameter 'off_map_penalty' must be a non-negative finite number.");
    }
    if (config.occupancy_threshold < 0 || config.occupancy_threshold > 100) {
      throw std::runtime_error(
          "Parameter 'occupancy_threshold' must be in [0, 100].");
    }
    if (!isValidDistanceTransformMaskSize(
            config.distance_transform_mask_size)) {
      throw std::runtime_error(
          "Parameter 'distance_transform_mask_size' must be 3 or 5.");
    }
    if (!std::isfinite(config.init_field_width) ||
        config.init_field_width <= 0.0) {
      throw std::runtime_error(
          "Parameter 'init_field_width' must be a positive finite number.");
    }
    if (!std::isfinite(config.init_field_height) ||
        config.init_field_height <= 0.0) {
      throw std::runtime_error(
          "Parameter 'init_field_height' must be a positive finite number.");
    }
    validateAxisMotionNoiseConfig(config.odom_noise_x, "odom_noise_x");
    validateAxisMotionNoiseConfig(config.odom_noise_y, "odom_noise_y");
    validateAxisMotionNoiseConfig(config.odom_noise_theta, "odom_noise_theta");
    if (stm32_request_timeout_ms < 1) {
      throw std::runtime_error(
          "Parameter 'stm32_request_timeout_ms' must be at least 1.");
    }
    if (filter_period_ms < 1) {
      throw std::runtime_error(
          "Parameter 'filter_period_ms' must be at least 1.");
    }
    if (timing_log_interval < 1) {
      throw std::runtime_error(
          "Parameter 'timing_log_interval' must be at least 1.");
    }
    if (processing_time_topic.empty()) {
      throw std::runtime_error(
          "Parameter 'processing_time_topic' must not be empty.");
    }

    const AxisMapping forward_axis = parseAxisMapping(forward_axis_name);
    const AxisMapping left_axis = parseAxisMapping(left_axis_name);
    if (forward_axis.axis == left_axis.axis) {
      throw std::runtime_error(
          "Parameters 'forward_axis' and 'left_axis' must be orthogonal.");
    }
  }

  void validateGlobalSearchConfig(double global_search_random_ratio,
                                  double global_search_noise_xy,
                                  double global_search_noise_theta,
                                  double localized_xy_std_threshold,
                                  double localized_theta_std_threshold,
                                  int localized_min_updates,
                                  double lost_alpha_ratio_threshold,
                                  int lost_min_updates) const {
    if (!std::isfinite(global_search_random_ratio) ||
        global_search_random_ratio < 0.0 || global_search_random_ratio > 1.0) {
      throw std::runtime_error(
          "Parameter 'global_search_random_ratio' must be between 0 and 1.");
    }
    if (!std::isfinite(global_search_noise_xy) ||
        global_search_noise_xy < 0.0) {
      throw std::runtime_error(
          "Parameter 'global_search_noise_xy' must be a non-negative finite "
          "number.");
    }
    if (!std::isfinite(global_search_noise_theta) ||
        global_search_noise_theta < 0.0) {
      throw std::runtime_error(
          "Parameter 'global_search_noise_theta' must be a non-negative "
          "finite number.");
    }
    if (!std::isfinite(localized_xy_std_threshold) ||
        localized_xy_std_threshold <= 0.0) {
      throw std::runtime_error(
          "Parameter 'localized_xy_std_threshold' must be a positive finite "
          "number.");
    }
    if (!std::isfinite(localized_theta_std_threshold) ||
        localized_theta_std_threshold <= 0.0) {
      throw std::runtime_error(
          "Parameter 'localized_theta_std_threshold' must be a positive "
          "finite number.");
    }
    if (localized_min_updates < 1) {
      throw std::runtime_error(
          "Parameter 'localized_min_updates' must be at least 1.");
    }
    if (!std::isfinite(lost_alpha_ratio_threshold) ||
        lost_alpha_ratio_threshold < 0.0 ||
        lost_alpha_ratio_threshold > 1.0) {
      throw std::runtime_error(
          "Parameter 'lost_alpha_ratio_threshold' must be between 0 and 1.");
    }
    if (lost_min_updates < 1) {
      throw std::runtime_error(
          "Parameter 'lost_min_updates' must be at least 1.");
    }
  }

  rcl_interfaces::msg::SetParametersResult
  handleParameterUpdates(const std::vector<rclcpp::Parameter> &parameters) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    double candidate_meters_per_pixel = meters_per_pixel_;
    std::string candidate_forward_axis_name = forward_axis_name_;
    std::string candidate_left_axis_name = left_axis_name_;
    int candidate_max_points = max_points_;
    rcj_loc::ParticleFilterAmclFusionConfig candidate_filter_config =
        filter_config_;
    int candidate_stm32_request_timeout_ms = stm32_request_timeout_ms_;
    bool candidate_stm32_enable_odometry_log = stm32_enable_odometry_log_;
    int candidate_filter_period_ms = filter_period_ms_;
    bool candidate_enable_timing_log = enable_timing_log_;
    int candidate_timing_log_interval = timing_log_interval_;
    bool candidate_use_weighted_mean_pose = use_weighted_mean_pose_;
    bool candidate_enable_global_search = enable_global_search_;
    double candidate_global_search_random_ratio = global_search_random_ratio_;
    double candidate_global_search_noise_xy = global_search_noise_xy_;
    double candidate_global_search_noise_theta = global_search_noise_theta_;
    double candidate_localized_xy_std_threshold =
        localized_xy_std_threshold_;
    double candidate_localized_theta_std_threshold =
        localized_theta_std_threshold_;
    int candidate_localized_min_updates = localized_min_updates_;
    double candidate_lost_alpha_ratio_threshold =
        lost_alpha_ratio_threshold_;
    int candidate_lost_min_updates = lost_min_updates_;

    bool reinitialize_particles = false;
    bool recreate_timer = false;
    bool map_rebuild_deferred = false;

    for (const auto &parameter : parameters) {
      const auto &name = parameter.get_name();

      if (name == "meters_per_pixel") {
        candidate_meters_per_pixel = parameter.as_double();
      } else if (name == "forward_axis") {
        candidate_forward_axis_name = parameter.as_string();
      } else if (name == "left_axis") {
        candidate_left_axis_name = parameter.as_string();
      } else if (name == "max_points") {
        candidate_max_points = static_cast<int>(parameter.as_int());
      } else if (name == "use_weighted_mean_pose") {
        candidate_use_weighted_mean_pose = parameter.as_bool();
      } else if (name == "num_particles") {
        candidate_filter_config.num_particles =
            static_cast<int>(parameter.as_int());
        reinitialize_particles = true;
      } else if (name == "sigma_hit") {
        candidate_filter_config.sigma_hit = parameter.as_double();
      } else if (name == "noise_xy") {
        candidate_filter_config.noise_xy = parameter.as_double();
      } else if (name == "noise_theta") {
        candidate_filter_config.noise_theta = parameter.as_double();
      } else if (name == "alpha_fast_rate") {
        candidate_filter_config.alpha_fast_rate = parameter.as_double();
      } else if (name == "alpha_slow_rate") {
        candidate_filter_config.alpha_slow_rate = parameter.as_double();
      } else if (name == "random_injection_max_ratio") {
        candidate_filter_config.random_injection_max_ratio =
            parameter.as_double();
      } else if (name == "off_map_penalty") {
        candidate_filter_config.off_map_penalty = parameter.as_double();
      } else if (name == "occupancy_threshold") {
        candidate_filter_config.occupancy_threshold =
            static_cast<int>(parameter.as_int());
        map_rebuild_deferred = true;
      } else if (name == "distance_transform_mask_size") {
        candidate_filter_config.distance_transform_mask_size =
            static_cast<int>(parameter.as_int());
        map_rebuild_deferred = true;
      } else if (name == "init_field_width") {
        candidate_filter_config.init_field_width = parameter.as_double();
        reinitialize_particles = true;
      } else if (name == "init_field_height") {
        candidate_filter_config.init_field_height = parameter.as_double();
        reinitialize_particles = true;
      } else if (name == "odom_noise_x_from_x") {
        candidate_filter_config.odom_noise_x.from_x = parameter.as_double();
      } else if (name == "odom_noise_x_from_y") {
        candidate_filter_config.odom_noise_x.from_y = parameter.as_double();
      } else if (name == "odom_noise_x_from_theta") {
        candidate_filter_config.odom_noise_x.from_theta = parameter.as_double();
      } else if (name == "odom_noise_x_bias") {
        candidate_filter_config.odom_noise_x.bias = parameter.as_double();
      } else if (name == "odom_noise_y_from_x") {
        candidate_filter_config.odom_noise_y.from_x = parameter.as_double();
      } else if (name == "odom_noise_y_from_y") {
        candidate_filter_config.odom_noise_y.from_y = parameter.as_double();
      } else if (name == "odom_noise_y_from_theta") {
        candidate_filter_config.odom_noise_y.from_theta = parameter.as_double();
      } else if (name == "odom_noise_y_bias") {
        candidate_filter_config.odom_noise_y.bias = parameter.as_double();
      } else if (name == "odom_noise_theta_from_x") {
        candidate_filter_config.odom_noise_theta.from_x = parameter.as_double();
      } else if (name == "odom_noise_theta_from_y") {
        candidate_filter_config.odom_noise_theta.from_y = parameter.as_double();
      } else if (name == "odom_noise_theta_from_theta") {
        candidate_filter_config.odom_noise_theta.from_theta =
            parameter.as_double();
      } else if (name == "odom_noise_theta_bias") {
        candidate_filter_config.odom_noise_theta.bias = parameter.as_double();
      } else if (name == "stm32_request_timeout_ms") {
        candidate_stm32_request_timeout_ms =
            static_cast<int>(parameter.as_int());
      } else if (name == "stm32_enable_odometry_log") {
        candidate_stm32_enable_odometry_log = parameter.as_bool();
      } else if (name == "enable_global_search") {
        candidate_enable_global_search = parameter.as_bool();
      } else if (name == "global_search_random_ratio") {
        candidate_global_search_random_ratio = parameter.as_double();
      } else if (name == "global_search_noise_xy") {
        candidate_global_search_noise_xy = parameter.as_double();
      } else if (name == "global_search_noise_theta") {
        candidate_global_search_noise_theta = parameter.as_double();
      } else if (name == "localized_xy_std_threshold") {
        candidate_localized_xy_std_threshold = parameter.as_double();
      } else if (name == "localized_theta_std_threshold") {
        candidate_localized_theta_std_threshold = parameter.as_double();
      } else if (name == "localized_min_updates") {
        candidate_localized_min_updates =
            static_cast<int>(parameter.as_int());
      } else if (name == "lost_alpha_ratio_threshold") {
        candidate_lost_alpha_ratio_threshold = parameter.as_double();
      } else if (name == "lost_min_updates") {
        candidate_lost_min_updates = static_cast<int>(parameter.as_int());
      } else if (name == "filter_period_ms") {
        candidate_filter_period_ms = static_cast<int>(parameter.as_int());
        recreate_timer = true;
      } else if (name == "enable_timing_log") {
        candidate_enable_timing_log = parameter.as_bool();
      } else if (name == "timing_log_interval") {
        candidate_timing_log_interval = static_cast<int>(parameter.as_int());
      } else if (name == "mask_topic" || name == "enable_localization" ||
                 name == "publish_debug_pointcloud" ||
                 name == "debug_pointcloud_topic" || name == "map_topic" ||
                 name == "yaw_topic" || name == "use_fake_yaw" ||
                 name == "fake_yaw_degrees" ||
                 name == "yaw_zero_map_degrees" ||
                 name == "use_stm32_gateway_odometry" ||
                 name == "stm32_command_service" ||
                 name == "use_stm32_request_theta" ||
                 name == "publish_particle_weight_markers" ||
                 name == "particle_weight_marker_topic" ||
                 name == "particle_weight_marker_scale" ||
                 name == "publish_processing_time" ||
                 name == "processing_time_topic") {
        result.successful = false;
        result.reason =
            "Parameter '" + name + "' requires restarting the node.";
        return result;
      }
    }

    try {
      validateConfig(candidate_meters_per_pixel, candidate_forward_axis_name,
                     candidate_left_axis_name, candidate_max_points,
                     candidate_filter_config,
                     candidate_stm32_request_timeout_ms,
                     candidate_filter_period_ms, candidate_timing_log_interval,
                     processing_time_topic_);
      validateGlobalSearchConfig(
          candidate_global_search_random_ratio,
          candidate_global_search_noise_xy,
          candidate_global_search_noise_theta,
          candidate_localized_xy_std_threshold,
          candidate_localized_theta_std_threshold,
          candidate_localized_min_updates,
          candidate_lost_alpha_ratio_threshold, candidate_lost_min_updates);
    } catch (const std::exception &ex) {
      result.successful = false;
      result.reason = ex.what();
      return result;
    }

    meters_per_pixel_ = candidate_meters_per_pixel;
    forward_axis_name_ = candidate_forward_axis_name;
    left_axis_name_ = candidate_left_axis_name;
    max_points_ = candidate_max_points;
    filter_config_ = candidate_filter_config;
    stm32_request_timeout_ms_ = candidate_stm32_request_timeout_ms;
    stm32_enable_odometry_log_ = candidate_stm32_enable_odometry_log;
    filter_period_ms_ = candidate_filter_period_ms;
    enable_timing_log_ = candidate_enable_timing_log;
    timing_log_interval_ = candidate_timing_log_interval;
    use_weighted_mean_pose_ = candidate_use_weighted_mean_pose;
    const bool global_search_was_enabled = enable_global_search_;
    enable_global_search_ = candidate_enable_global_search;
    global_search_random_ratio_ = candidate_global_search_random_ratio;
    global_search_noise_xy_ = candidate_global_search_noise_xy;
    global_search_noise_theta_ = candidate_global_search_noise_theta;
    localized_xy_std_threshold_ = candidate_localized_xy_std_threshold;
    localized_theta_std_threshold_ = candidate_localized_theta_std_threshold;
    localized_min_updates_ = candidate_localized_min_updates;
    lost_alpha_ratio_threshold_ = candidate_lost_alpha_ratio_threshold;
    lost_min_updates_ = candidate_lost_min_updates;
    forward_axis_ = parseAxisMapping(forward_axis_name_);
    left_axis_ = parseAxisMapping(left_axis_name_);

    if (pf_) {
      pf_->setConfig(filter_config_);
      if (reinitialize_particles) {
        pf_->initRandom();
        RCLCPP_INFO(this->get_logger(),
                    "Particle set reinitialized after parameter update.");
      }
    }

    if (recreate_timer && enable_localization_) {
      recreateTimer();
      RCLCPP_INFO(this->get_logger(), "Filter period updated to %d ms.",
                  filter_period_ms_);
    }

    if (map_rebuild_deferred) {
      RCLCPP_WARN(this->get_logger(),
                  "Updated map interpretation parameters will take effect on "
                  "the next received map.");
    }

    if (!enable_global_search_) {
      leaveGlobalSearch("global search disabled");
    } else if (!global_search_was_enabled) {
      enterGlobalSearch("global search enabled");
    }

    return result;
  }

  void clearGatewayOdomState() {
    if (!use_stm32_gateway_odometry_) {
      return;
    }

    std::lock_guard<std::mutex> lock(gateway_mutex_);
    gateway_request_in_flight_ = false;
    active_request_id_ = 0;
    active_request_yaw_rad_ = 0.0;
    active_request_sent_time_ = std::chrono::steady_clock::time_point{};
    active_request_pose_.reset();
    pending_gateway_result_.reset();
    last_success_pose_.reset();
    latest_failed_request_pose_.reset();
  }

  void enterGlobalSearch(const std::string &reason) {
    if (!enable_global_search_ || global_search_active_) {
      return;
    }

    global_search_active_ = true;
    localized_candidate_count_ = 0;
    lost_candidate_count_ = 0;
    clearGatewayOdomState();

    RCLCPP_WARN(
        this->get_logger(),
        "AMCL fusion entering global search: %s. Odom prediction is paused; "
        "particles will use random diffusion and random map injection.",
        reason.c_str());
  }

  void leaveGlobalSearch(const std::string &reason) {
    if (!global_search_active_) {
      return;
    }

    global_search_active_ = false;
    localized_candidate_count_ = 0;
    lost_candidate_count_ = 0;
    clearGatewayOdomState();

    RCLCPP_INFO(this->get_logger(),
                "AMCL fusion leaving global search: %s. Odom prediction is "
                "enabled again.",
                reason.c_str());
  }

  void updateGlobalSearchState(bool weights_updated) {
    if (!enable_global_search_ || !weights_updated || !pf_) {
      return;
    }

    const double xy_std = pf_->getPositionStdDev();
    const double theta_std = pf_->getHeadingStdDev();
    const bool localized_candidate =
        std::isfinite(xy_std) && std::isfinite(theta_std) &&
        xy_std <= localized_xy_std_threshold_ &&
        theta_std <= localized_theta_std_threshold_;

    if (global_search_active_) {
      localized_candidate_count_ =
          localized_candidate ? localized_candidate_count_ + 1 : 0;
      if (localized_candidate_count_ >= localized_min_updates_) {
        leaveGlobalSearch("particle cloud is concentrated");
      }
      return;
    }

    const double alpha_ratio = pf_->getAlphaRatio();
    const bool lost_candidate =
        std::isfinite(alpha_ratio) &&
        alpha_ratio < lost_alpha_ratio_threshold_;
    lost_candidate_count_ = lost_candidate ? lost_candidate_count_ + 1 : 0;
    if (lost_candidate_count_ >= lost_min_updates_) {
      enterGlobalSearch("adaptive weight ratio indicates localization loss");
    }
  }

  void recreateTimer() {
    timer_.reset();
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(filter_period_ms_),
                                std::bind(&AmclFusionNode::filterLoop, this));
  }

  void yawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_yaw_rad_ = fieldYawDegreesToRosMapRadians(
        static_cast<double>(msg->data), yaw_zero_map_degrees_);
    yaw_initialized_ = true;
  }

  void publishFakeYaw() {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(fake_yaw_degrees_);
    fake_yaw_pub_->publish(msg);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!pf_) {
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Map received. Building AMCL fusion distance transform field...");
    pf_->setMap(msg);
    map_received_ = true;
    if (enable_global_search_) {
      pf_->initRandomInMap();
      global_search_active_ = true;
      localized_candidate_count_ = 0;
      lost_candidate_count_ = 0;
      clearGatewayOdomState();
      RCLCPP_INFO(
          this->get_logger(),
          "AMCL fusion initialized global search particles across the map.");
    }
  }

  void expireTimedOutGatewayRequest() {
    if (!use_stm32_gateway_odometry_) {
      return;
    }

    GatewayOdomResult timeout_result;
    {
      std::lock_guard<std::mutex> lock(gateway_mutex_);
      if (!gateway_request_in_flight_) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - active_request_sent_time_);
      if (elapsed_ms < std::chrono::milliseconds(stm32_request_timeout_ms_)) {
        return;
      }

      timeout_result.request_id = active_request_id_;
      timeout_result.success = false;
      timeout_result.status = "timeout";
      timeout_result.message =
          "STM32 odometry request timed out locally after " +
          std::to_string(stm32_request_timeout_ms_) + " ms.";
      timeout_result.request_yaw_rad = active_request_yaw_rad_;
      timeout_result.request_pose = active_request_pose_;
      gateway_request_in_flight_ = false;
      active_request_id_ = 0;
      active_request_yaw_rad_ = 0.0;
      active_request_sent_time_ = std::chrono::steady_clock::time_point{};
      active_request_pose_.reset();
      recordGatewayOdomResultLocked(timeout_result);
      pending_gateway_result_ = timeout_result;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "STM32 odometry request %" PRIu64 " timed out locally after %d ms. "
        "Falling back to random diffusion until a later request succeeds.",
        timeout_result.request_id, stm32_request_timeout_ms_);
  }

  std::optional<GatewayOdomResult> consumePendingGatewayResult() {
    std::lock_guard<std::mutex> lock(gateway_mutex_);
    if (!pending_gateway_result_.has_value()) {
      return std::nullopt;
    }

    std::optional<GatewayOdomResult> result = pending_gateway_result_;
    pending_gateway_result_.reset();
    return result;
  }

  bool isSuccessfulGatewayResult(const GatewayOdomResult &result) const {
    return result.success && result.status == "ok";
  }

  void recordGatewayOdomResultLocked(GatewayOdomResult &result) {
    ++gateway_request_total_count_;
    if (isSuccessfulGatewayResult(result)) {
      ++gateway_request_successful_count_;
    }

    result.total_request_count = gateway_request_total_count_;
    result.successful_request_count = gateway_request_successful_count_;
    result.success_rate_percent =
        gateway_request_total_count_ == 0
            ? 0.0
            : (100.0 * static_cast<double>(gateway_request_successful_count_) /
               static_cast<double>(gateway_request_total_count_));
  }

  void rememberFailedGatewayRequest(const GatewayOdomResult &result) {
    if (!result.request_pose.has_value()) {
      RCLCPP_WARN(
          this->get_logger(),
          "STM32 odometry request %" PRIu64
          " failed without a request pose anchor; compensation state unchanged.",
          result.request_id);
      return;
    }

    latest_failed_request_pose_ = result.request_pose;
  }

  void rememberSuccessfulGatewayRequest(const rcj_loc::Particle &posterior_pose) {
    last_success_pose_ = posterior_pose;
    latest_failed_request_pose_.reset();
  }

  bool buildCompensatedGatewayOdom(const GatewayOdomResult &result,
                                   double &dx_cm, double &dy_cm,
                                   double &dtheta_deg,
                                   double &request_yaw_rad) {
    dx_cm = result.dx_cm;
    dy_cm = result.dy_cm;
    dtheta_deg = result.dtheta_deg;
    request_yaw_rad = result.request_yaw_rad;

    if (!latest_failed_request_pose_.has_value() ||
        !last_success_pose_.has_value()) {
      return true;
    }

    const auto &last_success = *last_success_pose_;
    const auto &latest_failure = *latest_failed_request_pose_;
    const double absorbed_global_x_m = latest_failure.x - last_success.x;
    const double absorbed_global_y_m = latest_failure.y - last_success.y;
    const double absorbed_dtheta_rad =
        normalizeAngle(latest_failure.theta - last_success.theta);
    const double stm32_x_axis_map_rad =
        stm32XAxisDegreesToRosMapRadians(yaw_zero_map_degrees_);
    const double cos_axis = std::cos(stm32_x_axis_map_rad);
    const double sin_axis = std::sin(stm32_x_axis_map_rad);
    const double absorbed_stm32_x_m =
        (cos_axis * absorbed_global_x_m) + (sin_axis * absorbed_global_y_m);
    const double absorbed_stm32_y_m =
        (-sin_axis * absorbed_global_x_m) + (cos_axis * absorbed_global_y_m);

    dx_cm -= absorbed_stm32_x_m * 100.0;
    dy_cm -= absorbed_stm32_y_m * 100.0;
    dtheta_deg = radiansToDegrees(normalizeAngle(
        degreesToRadians(dtheta_deg) - absorbed_dtheta_rad));
    request_yaw_rad = latest_failure.theta;

    if (stm32_enable_odometry_log_) {
      RCLCPP_INFO(
          this->get_logger(),
          "Compensated STM32 odometry request %" PRIu64
          ": raw=(%.6f, %.6f, %.6f), absorbed=(%.6f, %.6f, %.6f), "
          "compensated=(%.6f, %.6f, %.6f).",
          result.request_id, result.dx_cm, result.dy_cm, result.dtheta_deg,
          absorbed_stm32_x_m * 100.0, absorbed_stm32_y_m * 100.0,
          radiansToDegrees(absorbed_dtheta_rad), dx_cm, dy_cm, dtheta_deg);
    }

    return true;
  }

  bool applyGatewayMotionPrediction(const GatewayOdomResult &result) {
    double dx_cm = 0.0;
    double dy_cm = 0.0;
    double dtheta_deg = 0.0;
    double request_yaw_rad = 0.0;
    if (!buildCompensatedGatewayOdom(result, dx_cm, dy_cm, dtheta_deg,
                                     request_yaw_rad)) {
      return false;
    }

    const double delta_x_stm32_m = dx_cm * 0.01;
    const double delta_y_stm32_m = dy_cm * 0.01;
    const double delta_theta_rad = normalizeAngle(degreesToRadians(dtheta_deg));
    double absolute_yaw_rad = current_yaw_rad_;
    if (use_stm32_request_theta_) {
      if (!result.has_theta || !std::isfinite(result.theta_rad)) {
        RCLCPP_WARN(this->get_logger(),
                    "Ignoring STM32 odometry response %" PRIu64
                    " because request theta is not available.",
                    result.request_id);
        return false;
      }
      absolute_yaw_rad = result.theta_rad;
      request_yaw_rad = normalizeAngle(result.theta_rad - delta_theta_rad);
      current_yaw_rad_ = result.theta_rad;
      yaw_initialized_ = true;
    }
    const double stm32_x_axis_map_rad =
        stm32XAxisDegreesToRosMapRadians(yaw_zero_map_degrees_);
    const double cos_axis = std::cos(stm32_x_axis_map_rad);
    const double sin_axis = std::sin(stm32_x_axis_map_rad);
    const double delta_x_global_m =
        (cos_axis * delta_x_stm32_m) - (sin_axis * delta_y_stm32_m);
    const double delta_y_global_m =
        (sin_axis * delta_x_stm32_m) + (cos_axis * delta_y_stm32_m);

    if (!std::isfinite(delta_x_stm32_m) || !std::isfinite(delta_y_stm32_m) ||
        !std::isfinite(delta_x_global_m) || !std::isfinite(delta_y_global_m) ||
        !std::isfinite(delta_theta_rad) ||
        !std::isfinite(absolute_yaw_rad) ||
        !std::isfinite(request_yaw_rad)) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring STM32 odometry response %" PRIu64
                  " because it contains non-finite motion data.",
                  result.request_id);
      return false;
    }

    pf_->predict(absolute_yaw_rad, request_yaw_rad, delta_x_global_m,
                 delta_y_global_m, delta_theta_rad);
    return true;
  }

  void handleGatewayOdomResponse(std::uint64_t request_id,
                                 double request_yaw_rad,
                                 rcj_loc::Particle request_pose,
                                 Stm32CommandFuture future) {
    GatewayOdomResult result;
    result.request_id = request_id;
    result.request_yaw_rad = request_yaw_rad;
    result.request_pose = request_pose;

    try {
      const auto response = future.get();
      result.success = response->success;
      result.status = response->status;
      result.message = response->message;
      result.dx_cm = response->dx;
      result.dy_cm = response->dy;
      result.dtheta_deg = response->dtheta;
      result.theta_deg = response->theta;
      result.theta_rad = fieldYawDegreesToRosMapRadians(
          result.theta_deg, yaw_zero_map_degrees_);
      result.has_theta = true;
    } catch (const std::exception &ex) {
      result.success = false;
      result.status = "future_error";
      result.message = ex.what();
    }

    std::lock_guard<std::mutex> lock(gateway_mutex_);
    if (!gateway_request_in_flight_ || active_request_id_ != request_id) {
      return;
    }

    gateway_request_in_flight_ = false;
    active_request_id_ = 0;
    active_request_yaw_rad_ = 0.0;
    active_request_sent_time_ = std::chrono::steady_clock::time_point{};
    active_request_pose_.reset();
    recordGatewayOdomResultLocked(result);
    pending_gateway_result_ = result;
  }

  void issueGatewayOdomRequest(const rcj_loc::Particle &request_pose) {
    if (!use_stm32_gateway_odometry_ || !stm32_command_client_) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(gateway_mutex_);
      if (gateway_request_in_flight_ || pending_gateway_result_.has_value()) {
        return;
      }
    }

    if (!yaw_initialized_) {
      return;
    }

    if (!stm32_command_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "STM32 command service '%s' is not ready yet. "
          "AMCL fusion will keep using random diffusion fallback.",
          stm32_command_service_.c_str());
      return;
    }

    auto request = std::make_shared<Stm32Command::Request>();
    request->command = "cmd_request";

    std::uint64_t request_id = 0;
    double request_yaw_rad = current_yaw_rad_;
    {
      std::lock_guard<std::mutex> lock(gateway_mutex_);
      gateway_request_in_flight_ = true;
      active_request_id_ = next_gateway_request_id_++;
      active_request_sent_time_ = std::chrono::steady_clock::now();
      active_request_yaw_rad_ = current_yaw_rad_;
      active_request_pose_ = request_pose;
      request_id = active_request_id_;
      request_yaw_rad = active_request_yaw_rad_;
    }

    try {
      stm32_command_client_->async_send_request(
          request,
          [this, request_id, request_yaw_rad,
           request_pose](Stm32CommandFuture future) {
            handleGatewayOdomResponse(request_id, request_yaw_rad, request_pose,
                                      future);
          });
    } catch (const std::exception &ex) {
      std::lock_guard<std::mutex> lock(gateway_mutex_);
      if (active_request_id_ == request_id) {
        gateway_request_in_flight_ = false;
        active_request_id_ = 0;
        active_request_yaw_rad_ = 0.0;
        active_request_sent_time_ = std::chrono::steady_clock::time_point{};
        active_request_pose_.reset();
      }

      RCLCPP_WARN(this->get_logger(),
                  "Failed to dispatch async STM32 odometry request: %s",
                  ex.what());
    }
  }

  void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv::Mat mask;
    try {
      mask = cv_bridge::toCvCopy(msg, "mono8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cv_bridge failed while reading topdown mask: %s",
                           e.what());
      return;
    }

    std::vector<cv::Point> active_pixels;
    cv::findNonZero(mask, active_pixels);

    std::vector<rcj_loc::Point2D> observations;
    if (!active_pixels.empty()) {
      const std::size_t sample_count = std::min<std::size_t>(
          active_pixels.size(), static_cast<std::size_t>(max_points_));
      observations.reserve(sample_count);

      const double robot_origin_u_px =
          (static_cast<double>(mask.cols) - 1.0) * 0.5;
      const double robot_origin_v_px =
          (static_cast<double>(mask.rows) - 1.0) * 0.5;

      for (std::size_t i = 0; i < sample_count; ++i) {
        const cv::Point &pixel = active_pixels[selectSampleIndex(
            i, sample_count, active_pixels.size())];
        const double u = static_cast<double>(pixel.x);
        const double v = static_cast<double>(pixel.y);
        const double du = (u - robot_origin_u_px) * meters_per_pixel_;
        const double dv = (v - robot_origin_v_px) * meters_per_pixel_;

        observations.push_back({mapAxisValue(forward_axis_, du, dv),
                                mapAxisValue(left_axis_, du, dv)});
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

  void
  publishDebugPointCloud(const std_msgs::msg::Header &header,
                         const std::vector<rcj_loc::Point2D> &observations) {
    std::optional<rcj_loc::Particle> pose_estimate;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      pose_estimate = latest_pose_estimate_;
    }
    if (!pose_estimate.has_value()) {
      return;
    }

    const double cos_t = std::cos(pose_estimate->theta);
    const double sin_t = std::sin(pose_estimate->theta);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(observations.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto &observation : observations) {
      const double map_x = pose_estimate->x + (observation.x * cos_t) -
                           (observation.y * sin_t);
      const double map_y = pose_estimate->y + (observation.x * sin_t) +
                           (observation.y * cos_t);
      *iter_x = static_cast<float>(map_x);
      *iter_y = static_cast<float>(map_y);
      *iter_z = 0.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    debug_pointcloud_pub_->publish(cloud);
  }

  void filterLoop() {
    if (!enable_localization_ || !map_received_ || !pf_) {
      return;
    }

    const auto filter_start = std::chrono::steady_clock::now();

    std::vector<rcj_loc::Point2D> current_observations;
    {
      std::lock_guard<std::mutex> lock(obs_mutex_);
      current_observations = latest_observations_;
    }

    bool motion_prediction_applied = false;
    bool successful_gateway_result_seen = false;
    if (enable_global_search_ && global_search_active_) {
      pf_->predictWithNoise(current_yaw_rad_, global_search_noise_xy_,
                            global_search_noise_theta_);
      motion_prediction_applied = true;
    } else if (use_stm32_gateway_odometry_) {
      expireTimedOutGatewayRequest();

      const std::optional<GatewayOdomResult> gateway_result =
          consumePendingGatewayResult();
      if (gateway_result.has_value()) {
        if (stm32_enable_odometry_log_) {
          RCLCPP_INFO(
              this->get_logger(),
              "STM32 odometry response %" PRIu64
              ": success=%s, status='%s', dx=%.6f cm, dy=%.6f cm, "
              "dtheta=%.6f deg, theta=%.6f deg, total_requests=%" PRIu64
              ", successful_requests=%" PRIu64 ", success_rate=%.2f%%.",
              gateway_result->request_id,
              gateway_result->success ? "true" : "false",
              gateway_result->status.c_str(), gateway_result->dx_cm,
              gateway_result->dy_cm, gateway_result->dtheta_deg,
              gateway_result->theta_deg,
              gateway_result->total_request_count,
              gateway_result->successful_request_count,
              gateway_result->success_rate_percent);
        }
        if (gateway_result->success && gateway_result->status == "ok" &&
            yaw_initialized_) {
          successful_gateway_result_seen = true;
          motion_prediction_applied =
              applyGatewayMotionPrediction(*gateway_result);
        } else {
          rememberFailedGatewayRequest(*gateway_result);
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "STM32 odometry request failed with success=%s, status='%s'. "
              "Using random diffusion fallback.",
              gateway_result->success ? "true" : "false",
              gateway_result->status.c_str());
        }
      }
    }

    if (!motion_prediction_applied) {
      pf_->predict(current_yaw_rad_);
    }

    const bool weights_updated = pf_->updateWeights(current_observations);
    updateGlobalSearchState(weights_updated);

    const std::vector<rcj_loc::Particle> posterior_particles =
        pf_->getParticles();
    const rcj_loc::Particle posterior_pose =
        use_weighted_mean_pose_ ? pf_->getWeightedMeanPose()
                                : pf_->getBestPose();

    if (successful_gateway_result_seen) {
      rememberSuccessfulGatewayRequest(posterior_pose);
    }

    publishVisualizationsAndTF(posterior_particles, posterior_pose);
    const double forced_random_ratio =
        enable_global_search_ && global_search_active_
            ? global_search_random_ratio_
            : 0.0;
    pf_->resample(forced_random_ratio);

    const auto filter_end = std::chrono::steady_clock::now();
    const auto filter_duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(filter_end -
                                                              filter_start)
            .count();
    publishAndLogFilterTiming(filter_duration_us);

    if (use_stm32_gateway_odometry_ &&
        !(enable_global_search_ && global_search_active_)) {
      issueGatewayOdomRequest(posterior_pose);
    }
  }

  void publishAndLogFilterTiming(std::int64_t filter_duration_us) {
    ++filter_iteration_count_;
    total_filter_duration_us_ += filter_duration_us;

    if (publish_processing_time_ && processing_time_pub_) {
      std_msgs::msg::Float32 timing_msg;
      timing_msg.data = static_cast<float>(filter_duration_us) / 1000.0f;
      processing_time_pub_->publish(timing_msg);
    }

    if (!enable_timing_log_ ||
        filter_iteration_count_ %
                static_cast<std::size_t>(timing_log_interval_) !=
            0U) {
      return;
    }

    const double current_ms = static_cast<double>(filter_duration_us) / 1000.0;
    const double average_ms = static_cast<double>(total_filter_duration_us_) /
                              1000.0 /
                              static_cast<double>(filter_iteration_count_);

    RCLCPP_INFO(
        this->get_logger(),
        "AMCL fusion timing: current=%.3f ms, average=%.3f ms, iterations=%zu",
        current_ms, average_ms, filter_iteration_count_);
  }

  void
  publishVisualizationsAndTF(const std::vector<rcj_loc::Particle> &particles,
                             const rcj_loc::Particle &pose_estimate) {
    const rclcpp::Time now = this->now();
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      latest_pose_estimate_ = pose_estimate;
    }

    geometry_msgs::msg::PoseArray cloud_msg;
    cloud_msg.header.stamp = now;
    cloud_msg.header.frame_id = "map";

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
    publishParticleWeightMarkers(particles, now);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = pose_estimate.x;
    pose_msg.pose.pose.position.y = pose_estimate.y;

    tf2::Quaternion q_estimate;
    q_estimate.setRPY(0.0, 0.0, pose_estimate.theta);
    pose_msg.pose.pose.orientation.x = q_estimate.x();
    pose_msg.pose.pose.orientation.y = q_estimate.y();
    pose_msg.pose.pose.orientation.z = q_estimate.z();
    pose_msg.pose.pose.orientation.w = q_estimate.w();
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pose_estimate.x;
    transform.transform.translation.y = pose_estimate.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = pose_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std_msgs::msg::ColorRGBA particleWeightColor(double normalized_weight) const {
    const double t = std::clamp(normalized_weight, 0.0, 1.0);
    std_msgs::msg::ColorRGBA color;
    color.a = static_cast<float>(0.25 + (0.75 * t));

    if (t < 0.5) {
      const double u = t * 2.0;
      color.r = static_cast<float>(0.15 + (0.85 * u));
      color.g = static_cast<float>(0.35 + (0.55 * u));
      color.b = static_cast<float>(1.0 - (0.85 * u));
    } else {
      const double u = (t - 0.5) * 2.0;
      color.r = 1.0f;
      color.g = static_cast<float>(0.90 - (0.75 * u));
      color.b = 0.05f;
    }

    return color;
  }

  void publishParticleWeightMarkers(
      const std::vector<rcj_loc::Particle> &particles,
      const rclcpp::Time &stamp) {
    if (!publish_particle_weight_markers_ || !particle_weight_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "map";
    marker.ns = "amcl_particle_weights";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = particle_weight_marker_scale_;
    marker.scale.y = particle_weight_marker_scale_;
    marker.color.a = 1.0;
    marker.points.reserve(particles.size());
    marker.colors.reserve(particles.size());

    double min_weight = std::numeric_limits<double>::infinity();
    double max_weight = 0.0;
    for (const auto &particle : particles) {
      if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
        continue;
      }
      min_weight = std::min(min_weight, particle.weight);
      max_weight = std::max(max_weight, particle.weight);
    }

    const double weight_range = max_weight - min_weight;
    const bool has_weight_range =
        std::isfinite(min_weight) && weight_range > 1e-12;

    for (const auto &particle : particles) {
      geometry_msgs::msg::Point point;
      point.x = particle.x;
      point.y = particle.y;
      point.z = 0.03;
      marker.points.push_back(point);

      double normalized_weight = 0.0;
      if (has_weight_range && std::isfinite(particle.weight)) {
        normalized_weight = (particle.weight - min_weight) / weight_range;
      }
      marker.colors.push_back(particleWeightColor(normalized_weight));
    }

    particle_weight_marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Client<Stm32Command>::SharedPtr stm32_command_client_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fake_yaw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_pointcloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      particle_weight_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_yaw_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;

  std::mutex obs_mutex_;
  std::mutex gateway_mutex_;
  std::mutex pose_mutex_;
  std::vector<rcj_loc::Point2D> latest_observations_;
  std::unique_ptr<rcj_loc::ParticleFilterAmclFusion> pf_;
  std::optional<GatewayOdomResult> pending_gateway_result_;
  std::optional<rcj_loc::Particle> latest_pose_estimate_;

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
  bool use_weighted_mean_pose_ = false;
  bool publish_particle_weight_markers_ = false;
  std::string particle_weight_marker_topic_;
  double particle_weight_marker_scale_ = 0.035;
  std::string map_topic_;
  std::string yaw_topic_;
  bool use_fake_yaw_ = false;
  double fake_yaw_degrees_ = 0.0;
  double yaw_zero_map_degrees_ = 0.0;
  bool use_stm32_gateway_odometry_ = false;
  std::string stm32_command_service_;
  int stm32_request_timeout_ms_ = 200;
  bool stm32_enable_odometry_log_ = false;
  bool use_stm32_request_theta_ = false;
  bool enable_global_search_ = true;
  double global_search_random_ratio_ = 0.50;
  double global_search_noise_xy_ = 0.12;
  double global_search_noise_theta_ = 0.20;
  double localized_xy_std_threshold_ = 0.20;
  double localized_theta_std_threshold_ = 0.35;
  int localized_min_updates_ = 5;
  double lost_alpha_ratio_threshold_ = 0.45;
  int lost_min_updates_ = 3;
  bool global_search_active_ = true;
  int localized_candidate_count_ = 0;
  int lost_candidate_count_ = 0;
  double current_yaw_rad_ = 0.0;
  bool yaw_initialized_ = false;
  bool gateway_request_in_flight_ = false;
  std::uint64_t next_gateway_request_id_ = 1;
  std::uint64_t gateway_request_total_count_ = 0;
  std::uint64_t gateway_request_successful_count_ = 0;
  std::uint64_t active_request_id_ = 0;
  std::chrono::steady_clock::time_point active_request_sent_time_{};
  double active_request_yaw_rad_ = 0.0;
  std::optional<rcj_loc::Particle> active_request_pose_;
  std::optional<rcj_loc::Particle> last_success_pose_;
  std::optional<rcj_loc::Particle> latest_failed_request_pose_;
  bool map_received_ = false;
  int filter_period_ms_ = 100;
  bool publish_processing_time_ = true;
  std::string processing_time_topic_;
  bool enable_timing_log_ = true;
  int timing_log_interval_ = 30;
  std::size_t filter_iteration_count_ = 0;
  std::int64_t total_filter_duration_us_ = 0;
  rcj_loc::ParticleFilterAmclFusionConfig filter_config_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclFusionNode>());
  rclcpp::shutdown();
  return 0;
}
