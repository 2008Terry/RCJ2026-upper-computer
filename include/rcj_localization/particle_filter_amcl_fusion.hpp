#pragma once

#include <random>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

#include "rcj_localization/particle_filter.hpp"

namespace rcj_loc {

struct AxisMotionNoiseConfig {
  double from_x = 0.0;
  double from_y = 0.0;
  double from_theta = 0.0;
  double bias = 0.0;
};

struct ParticleFilterAmclFusionConfig {
  int num_particles = 1000;
  double sigma_hit = 0.10;
  double noise_xy = 0.05;
  double noise_theta = 0.10;
  double alpha_fast_rate = 0.1;
  double alpha_slow_rate = 0.001;
  double random_injection_max_ratio = 0.25;
  double off_map_penalty = 1.0;
  int occupancy_threshold = 50;
  int distance_transform_mask_size = 5;
  double init_field_width = 2.0;
  double init_field_height = 3.0;
  AxisMotionNoiseConfig odom_noise_x{0.08, 0.02, 0.0025, 0.000064};
  AxisMotionNoiseConfig odom_noise_y{0.02, 0.16, 0.0049, 0.000144};
  AxisMotionNoiseConfig odom_noise_theta{0.30, 0.60, 0.09, 0.000304617};
};

class ParticleFilterAmclFusion {
public:
  explicit ParticleFilterAmclFusion(
      const ParticleFilterAmclFusionConfig &config);

  void setConfig(const ParticleFilterAmclFusionConfig &config);
  const ParticleFilterAmclFusionConfig &getConfig() const { return config_; }

  void initRandom();
  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg);
  void predict(double absolute_yaw);
  void predict(double absolute_yaw, double request_yaw, double delta_x_global_m,
               double delta_y_global_m, double delta_theta_rad);
  bool updateWeights(const std::vector<Point2D> &local_observations);
  void resample();

  const std::vector<Particle> &getParticles() const { return particles_; }
  Particle getBestPose() const;
  bool hasMap() const { return map_initialized_; }

private:
  ParticleFilterAmclFusionConfig config_;
  std::vector<Particle> particles_;

  cv::Mat distance_map_;
  double map_resolution_ = 0.0;
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  bool map_initialized_ = false;

  std::mt19937 gen_;

  double alpha_slow_ = 0.0;
  double alpha_fast_ = 0.0;
};

} // namespace rcj_loc
