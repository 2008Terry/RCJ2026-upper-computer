#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
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
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

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
        current_yaw_rad_ = normalizeAngle(degreesToRadians(fake_yaw_degrees_));
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
      } else {
        odom_sub_ = this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            odom_topic_, 10,
            std::bind(&AmclFusionNode::odomCallback, this,
                      std::placeholders::_1));
      }

      pose_pub_ =
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "/amcl_pose", 10);
      particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/particlecloud", 10);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      recreateTimer();
    }

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &AmclFusionNode::handleParameterUpdates, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "amcl_fusion started. mask_topic='%s', yaw_topic='%s', "
        "use_fake_yaw=%s, fake_yaw_degrees=%.3f, "
        "odom_topic='%s', "
        "use_stm32_gateway_odometry=%s, stm32_command_service='%s', "
        "stm32_request_timeout_ms=%d, meters_per_pixel=%.6f, "
        "forward_axis='%s', "
        "left_axis='%s', max_points=%d, num_particles=%d, sigma_hit=%.3f, "
        "noise_xy=%.3f, noise_theta=%.3f, filter_period_ms=%d, "
        "publish_processing_time=%s, processing_time_topic='%s', "
        "enable_timing_log=%s, timing_log_interval=%d",
        mask_topic_.c_str(), yaw_topic_.c_str(),
        use_fake_yaw_ ? "true" : "false", fake_yaw_degrees_,
        odom_topic_.c_str(),
        use_stm32_gateway_odometry_ ? "true" : "false",
        stm32_command_service_.c_str(), stm32_request_timeout_ms_,
        meters_per_pixel_, forward_axis_name_.c_str(), left_axis_name_.c_str(),
        max_points_, filter_config_.num_particles, filter_config_.sigma_hit,
        filter_config_.noise_xy, filter_config_.noise_theta, filter_period_ms_,
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
    double request_yaw_rad = 0.0;
  };

  void declareParameters() {
    this->declare_parameter<std::string>(
        "mask_topic", "/white_line_skeleton_filter_node/white_final_mask");
    this->declare_parameter("meters_per_pixel", 0.0025);
    this->declare_parameter<std::string>("forward_axis", "v+");
    this->declare_parameter<std::string>("left_axis", "u-");
    this->declare_parameter("max_points", 5000);
    this->declare_parameter("enable_localization", true);
    this->declare_parameter("publish_debug_pointcloud", false);
    this->declare_parameter<std::string>("debug_pointcloud_topic",
                                         "/field_line_observations_debug");
    this->declare_parameter("num_particles", 1000);
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("yaw_topic", "/robot/yaw");
    this->declare_parameter("use_fake_yaw", false);
    this->declare_parameter("fake_yaw_degrees", 0.0);
    this->declare_parameter<std::string>("odom_topic", "/wheel_odometry");
    this->declare_parameter("use_stm32_gateway_odometry", false);
    this->declare_parameter<std::string>("stm32_command_service",
                                         "/stm32/send_command");
    this->declare_parameter("stm32_request_timeout_ms", 200);

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
    map_topic_ = this->get_parameter("map_topic").as_string();
    yaw_topic_ = this->get_parameter("yaw_topic").as_string();
    use_fake_yaw_ = this->get_parameter("use_fake_yaw").as_bool();
    fake_yaw_degrees_ = this->get_parameter("fake_yaw_degrees").as_double();
    if (!std::isfinite(fake_yaw_degrees_)) {
      throw std::runtime_error(
          "Parameter 'fake_yaw_degrees' must be a finite number.");
    }
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    use_stm32_gateway_odometry_ =
        this->get_parameter("use_stm32_gateway_odometry").as_bool();
    stm32_command_service_ =
        this->get_parameter("stm32_command_service").as_string();
    stm32_request_timeout_ms_ = static_cast<int>(
        this->get_parameter("stm32_request_timeout_ms").as_int());
    if (use_stm32_gateway_odometry_ && stm32_command_service_.empty()) {
      throw std::runtime_error(
          "Parameter 'stm32_command_service' must not be empty when "
          "'use_stm32_gateway_odometry' is true.");
    }

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
    int candidate_filter_period_ms = filter_period_ms_;
    bool candidate_enable_timing_log = enable_timing_log_;
    int candidate_timing_log_interval = timing_log_interval_;

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
                 name == "fake_yaw_degrees" || name == "odom_topic" ||
                 name == "use_stm32_gateway_odometry" ||
                 name == "stm32_command_service" ||
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
    filter_period_ms_ = candidate_filter_period_ms;
    enable_timing_log_ = candidate_enable_timing_log;
    timing_log_interval_ = candidate_timing_log_interval;
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

    return result;
  }

  void recreateTimer() {
    timer_.reset();
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(filter_period_ms_),
                                std::bind(&AmclFusionNode::filterLoop, this));
  }

  void yawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_yaw_rad_ =
        normalizeAngle(degreesToRadians(static_cast<double>(msg->data)));
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
  }

  void odomCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 rotation_matrix(q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rotation_matrix.getRPY(roll, pitch, yaw);

    odom_x_ = x;
    odom_y_ = y;
    odom_theta_ = yaw;

    if (!odom_initialized_) {
      prev_odom_x_ = odom_x_;
      prev_odom_y_ = odom_y_;
      prev_odom_theta_ = odom_theta_;
      odom_initialized_ = true;
    }
  }

  void expireTimedOutGatewayRequest() {
    if (!use_stm32_gateway_odometry_) {
      return;
    }

    std::uint64_t expired_request_id = 0;
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

      expired_request_id = active_request_id_;
      gateway_request_in_flight_ = false;
      active_request_id_ = 0;
      active_request_yaw_rad_ = 0.0;
      active_request_sent_time_ = std::chrono::steady_clock::time_point{};
    }

    RCLCPP_WARN(
        this->get_logger(),
        "STM32 odometry request %" PRIu64 " timed out locally after %d ms. "
        "Falling back to random diffusion until a later request succeeds.",
        expired_request_id, stm32_request_timeout_ms_);
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

  bool applyGatewayMotionPrediction(const GatewayOdomResult &result) {
    const double delta_x_global_m = result.dx_cm * 0.01;
    const double delta_y_global_m = result.dy_cm * 0.01;
    const double delta_theta_rad =
        normalizeAngle(degreesToRadians(result.dtheta_deg));

    if (!std::isfinite(delta_x_global_m) || !std::isfinite(delta_y_global_m) ||
        !std::isfinite(delta_theta_rad) ||
        !std::isfinite(result.request_yaw_rad)) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring STM32 odometry response %" PRIu64
                  " because it contains non-finite motion data.",
                  result.request_id);
      return false;
    }

    pf_->predict(current_yaw_rad_, result.request_yaw_rad, delta_x_global_m,
                 delta_y_global_m, delta_theta_rad);
    return true;
  }

  void handleGatewayOdomResponse(std::uint64_t request_id,
                                 double request_yaw_rad,
                                 Stm32CommandFuture future) {
    GatewayOdomResult result;
    result.request_id = request_id;
    result.request_yaw_rad = request_yaw_rad;

    try {
      const auto response = future.get();
      result.success = response->success;
      result.status = response->status;
      result.message = response->message;
      result.dx_cm = response->dx;
      result.dy_cm = response->dy;
      result.dtheta_deg = response->dtheta;
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
    pending_gateway_result_ = result;
  }

  void issueGatewayOdomRequest() {
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
      request_id = active_request_id_;
      request_yaw_rad = active_request_yaw_rad_;
    }

    try {
      stm32_command_client_->async_send_request(
          request,
          [this, request_id, request_yaw_rad](Stm32CommandFuture future) {
            handleGatewayOdomResponse(request_id, request_yaw_rad, future);
          });
    } catch (const std::exception &ex) {
      std::lock_guard<std::mutex> lock(gateway_mutex_);
      if (active_request_id_ == request_id) {
        gateway_request_in_flight_ = false;
        active_request_id_ = 0;
        active_request_yaw_rad_ = 0.0;
        active_request_sent_time_ = std::chrono::steady_clock::time_point{};
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
    if (use_stm32_gateway_odometry_) {
      expireTimedOutGatewayRequest();

      const std::optional<GatewayOdomResult> gateway_result =
          consumePendingGatewayResult();
      if (gateway_result.has_value()) {
        if (gateway_result->success && gateway_result->status == "ok" &&
            yaw_initialized_) {
          motion_prediction_applied =
              applyGatewayMotionPrediction(*gateway_result);
        } else {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "STM32 odometry request failed with success=%s, status='%s'. "
              "Using random diffusion fallback.",
              gateway_result->success ? "true" : "false",
              gateway_result->status.c_str());
        }
      }
    } else if (odom_initialized_) {
      const double dx = odom_x_ - prev_odom_x_;
      const double dy = odom_y_ - prev_odom_y_;
      const double delta_theta = normalizeAngle(odom_theta_ - prev_odom_theta_);
      const double request_yaw = prev_odom_theta_;

      prev_odom_x_ = odom_x_;
      prev_odom_y_ = odom_y_;
      prev_odom_theta_ = odom_theta_;

      pf_->predict(current_yaw_rad_, request_yaw, dx, dy, delta_theta);
      motion_prediction_applied = true;
    }

    if (!motion_prediction_applied) {
      pf_->predict(current_yaw_rad_);
    }

    pf_->updateWeights(current_observations);

    const std::vector<rcj_loc::Particle> posterior_particles =
        pf_->getParticles();
    const rcj_loc::Particle posterior_best_pose = pf_->getBestPose();

    publishVisualizationsAndTF(posterior_particles, posterior_best_pose);
    pf_->resample();

    const auto filter_end = std::chrono::steady_clock::now();
    const auto filter_duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(filter_end -
                                                              filter_start)
            .count();
    publishAndLogFilterTiming(filter_duration_us);

    if (use_stm32_gateway_odometry_) {
      issueGatewayOdomRequest();
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
                             const rcj_loc::Particle &best_pose) {
    const rclcpp::Time now = this->now();

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

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = best_pose.x;
    pose_msg.pose.pose.position.y = best_pose.y;

    tf2::Quaternion q_best;
    q_best.setRPY(0.0, 0.0, best_pose.theta);
    pose_msg.pose.pose.orientation.x = q_best.x();
    pose_msg.pose.pose.orientation.y = q_best.y();
    pose_msg.pose.pose.orientation.z = q_best.z();
    pose_msg.pose.pose.orientation.w = q_best.w();
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = best_pose.x;
    transform.transform.translation.y = best_pose.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = pose_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      odom_sub_;
  rclcpp::Client<Stm32Command>::SharedPtr stm32_command_client_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fake_yaw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      debug_pointcloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processing_time_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr fake_yaw_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;

  std::mutex obs_mutex_;
  std::mutex gateway_mutex_;
  std::vector<rcj_loc::Point2D> latest_observations_;
  std::unique_ptr<rcj_loc::ParticleFilterAmclFusion> pf_;
  std::optional<GatewayOdomResult> pending_gateway_result_;

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
  std::string map_topic_;
  std::string yaw_topic_;
  bool use_fake_yaw_ = false;
  double fake_yaw_degrees_ = 0.0;
  std::string odom_topic_;
  bool use_stm32_gateway_odometry_ = false;
  std::string stm32_command_service_;
  int stm32_request_timeout_ms_ = 200;
  double current_yaw_rad_ = 0.0;
  bool yaw_initialized_ = false;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_theta_ = 0.0;
  double prev_odom_x_ = 0.0;
  double prev_odom_y_ = 0.0;
  double prev_odom_theta_ = 0.0;
  bool odom_initialized_ = false;
  bool gateway_request_in_flight_ = false;
  std::uint64_t next_gateway_request_id_ = 1;
  std::uint64_t active_request_id_ = 0;
  std::chrono::steady_clock::time_point active_request_sent_time_{};
  double active_request_yaw_rad_ = 0.0;
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
