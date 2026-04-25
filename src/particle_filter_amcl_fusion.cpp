#include "rcj_localization/particle_filter_amcl_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rcj_loc {

namespace {

double safeExp(double value) {
  if (value < -700.0) {
    return 0.0;
  }
  return std::exp(value);
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

double computeAxisVariance(const AxisMotionNoiseConfig &config,
                           double delta_x_body_m, double delta_y_body_m,
                           double delta_theta_rad) {
  const double variance =
      (config.from_x * delta_x_body_m * delta_x_body_m) +
      (config.from_y * delta_y_body_m * delta_y_body_m) +
      (config.from_theta * delta_theta_rad * delta_theta_rad) + config.bias;
  return std::max(0.0, variance);
}

} // namespace

ParticleFilterAmclFusion::ParticleFilterAmclFusion(
    const ParticleFilterAmclFusionConfig &config)
    : config_(config) {
  std::random_device rd;
  gen_ = std::mt19937(rd());
  initRandom();
}

void ParticleFilterAmclFusion::setConfig(
    const ParticleFilterAmclFusionConfig &config) {
  config_ = config;
}

void ParticleFilterAmclFusion::initRandom() {
  std::uniform_real_distribution<double> dist_x(-config_.init_field_width / 2.0,
                                                config_.init_field_width / 2.0);
  std::uniform_real_distribution<double> dist_y(
      -config_.init_field_height / 2.0, config_.init_field_height / 2.0);
  std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

  particles_.clear();
  particles_.reserve(static_cast<std::size_t>(config_.num_particles));
  for (int i = 0; i < config_.num_particles; ++i) {
    particles_.push_back({dist_x(gen_), dist_y(gen_), dist_theta(gen_),
                          1.0 / config_.num_particles});
  }

  alpha_slow_ = 0.0;
  alpha_fast_ = 0.0;
}

void ParticleFilterAmclFusion::setMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg) {
  map_resolution_ = map_msg->info.resolution;
  map_origin_x_ = map_msg->info.origin.position.x;
  map_origin_y_ = map_msg->info.origin.position.y;
  const int width = static_cast<int>(map_msg->info.width);
  const int height = static_cast<int>(map_msg->info.height);

  cv::Mat binary_map(height, width, CV_8UC1);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int i = x + (y * width);
      const int cell_value = map_msg->data[i];
      binary_map.at<uchar>(y, x) =
          cell_value > config_.occupancy_threshold ? 0 : 255;
    }
  }

  cv::distanceTransform(binary_map, distance_map_, cv::DIST_L2,
                        config_.distance_transform_mask_size);

  map_initialized_ = true;
}

void ParticleFilterAmclFusion::predict(double absolute_yaw) {
  std::normal_distribution<double> noise_xy(0.0, config_.noise_xy);
  std::normal_distribution<double> noise_theta(0.0, config_.noise_theta);

  for (auto &particle : particles_) {
    particle.x += noise_xy(gen_);
    particle.y += noise_xy(gen_);
    particle.theta = normalizeAngle(absolute_yaw + noise_theta(gen_));
  }
}

void ParticleFilterAmclFusion::predict(double absolute_yaw, double request_yaw,
                                       double delta_x_global_m,
                                       double delta_y_global_m,
                                       double delta_theta_rad) {
  const double yaw_mid = normalizeAngle(request_yaw + (0.5 * delta_theta_rad));
  const double cos_mid = std::cos(yaw_mid);
  const double sin_mid = std::sin(yaw_mid);

  const double delta_x_body_m =
      (cos_mid * delta_x_global_m) + (sin_mid * delta_y_global_m);
  const double delta_y_body_m =
      (-sin_mid * delta_x_global_m) + (cos_mid * delta_y_global_m);

  const double x_variance = computeAxisVariance(
      config_.odom_noise_x, delta_x_body_m, delta_y_body_m, delta_theta_rad);
  const double y_variance = computeAxisVariance(
      config_.odom_noise_y, delta_x_body_m, delta_y_body_m, delta_theta_rad);
  const double theta_variance =
      computeAxisVariance(config_.odom_noise_theta, delta_x_body_m,
                          delta_y_body_m, delta_theta_rad);

  std::normal_distribution<double> noise_x_body(0.0, std::sqrt(x_variance));
  std::normal_distribution<double> noise_y_body(0.0, std::sqrt(y_variance));
  std::normal_distribution<double> noise_theta(0.0, std::sqrt(theta_variance));

  for (auto &particle : particles_) {
    const double sampled_x_body = delta_x_body_m + noise_x_body(gen_);
    const double sampled_y_body = delta_y_body_m + noise_y_body(gen_);
    const double sampled_x_global =
        (cos_mid * sampled_x_body) - (sin_mid * sampled_y_body);
    const double sampled_y_global =
        (sin_mid * sampled_x_body) + (cos_mid * sampled_y_body);

    particle.x += sampled_x_global;
    particle.y += sampled_y_global;
    particle.theta = normalizeAngle(absolute_yaw + noise_theta(gen_));
  }
}

bool ParticleFilterAmclFusion::updateWeights(
    const std::vector<Point2D> &local_observations) {
  if (!map_initialized_ || local_observations.empty() || particles_.empty()) {
    return false;
  }

  const int particle_count = static_cast<int>(particles_.size());
  const bool use_parallel =
      static_cast<std::size_t>(particle_count) * local_observations.size() >=
      16384;

  std::vector<double> mean_log_likelihoods(particles_.size(), 0.0);
  double max_mean_log_likelihood = -std::numeric_limits<double>::infinity();
  double raw_weight_sum = 0.0;

#pragma omp parallel for if(use_parallel) reduction(max:max_mean_log_likelihood) reduction(+:raw_weight_sum)
  for (int i = 0; i < particle_count; ++i) {
    auto &particle = particles_[i];
    double log_weight_sum = 0.0;

    const double cos_t = std::cos(particle.theta);
    const double sin_t = std::sin(particle.theta);

    for (const auto &observation : local_observations) {
      const double global_x =
          particle.x + (observation.x * cos_t) - (observation.y * sin_t);
      const double global_y =
          particle.y + (observation.x * sin_t) + (observation.y * cos_t);

      const int px = static_cast<int>(
          std::round((global_x - map_origin_x_) / map_resolution_));
      const int py = static_cast<int>(
          std::round((global_y - map_origin_y_) / map_resolution_));

      double dist_to_line = config_.off_map_penalty;
      if (px >= 0 && px < distance_map_.cols && py >= 0 &&
          py < distance_map_.rows) {
        dist_to_line = distance_map_.at<float>(py, px) * map_resolution_;
      }

      log_weight_sum += -(dist_to_line * dist_to_line) /
                        (2.0 * config_.sigma_hit * config_.sigma_hit);
    }

    const double mean_log_likelihood =
        log_weight_sum / static_cast<double>(local_observations.size());
    mean_log_likelihoods[i] = mean_log_likelihood;
    max_mean_log_likelihood =
        std::max(max_mean_log_likelihood, mean_log_likelihood);
    raw_weight_sum += safeExp(mean_log_likelihood);
  }

  const double avg_weight =
      raw_weight_sum / static_cast<double>(particles_.size());
  if (alpha_slow_ == 0.0) {
    alpha_slow_ = avg_weight;
    alpha_fast_ = avg_weight;
  } else {
    alpha_fast_ += config_.alpha_fast_rate * (avg_weight - alpha_fast_);
    alpha_slow_ += config_.alpha_slow_rate * (avg_weight - alpha_slow_);
  }

  double normalized_weight_sum = 0.0;
#pragma omp parallel for if (use_parallel) reduction(+ : normalized_weight_sum)
  for (int i = 0; i < particle_count; ++i) {
    particles_[i].weight =
        safeExp(mean_log_likelihoods[i] - max_mean_log_likelihood);
    normalized_weight_sum += particles_[i].weight;
  }

  if (normalized_weight_sum > 0.0) {
#pragma omp parallel for if (use_parallel)
    for (int i = 0; i < particle_count; ++i) {
      particles_[i].weight /= normalized_weight_sum;
    }
  } else {
    const double uniform_weight = 1.0 / static_cast<double>(particles_.size());
#pragma omp parallel for if (use_parallel)
    for (int i = 0; i < particle_count; ++i) {
      particles_[i].weight = uniform_weight;
    }
  }

  return true;
}

void ParticleFilterAmclFusion::resample() {
  if (particles_.empty()) {
    return;
  }

  std::vector<Particle> new_particles;
  new_particles.reserve(particles_.size());

  std::uniform_real_distribution<double> random_dist(
      0.0, 1.0 / static_cast<double>(particles_.size()));
  double r = random_dist(gen_);
  double c = particles_.front().weight;
  std::size_t i = 0;

  for (std::size_t m = 0; m < particles_.size(); ++m) {
    const double u =
        r + static_cast<double>(m) / static_cast<double>(particles_.size());
    while (u > c && !particles_.empty()) {
      i = (i + 1) % particles_.size();
      c += particles_[i].weight;
    }
    Particle survivor = particles_[i];
    survivor.weight = 1.0 / static_cast<double>(particles_.size());
    new_particles.push_back(survivor);
  }

  double p_random = 0.0;
  if (alpha_slow_ > std::numeric_limits<double>::epsilon()) {
    p_random = std::max(0.0, 1.0 - (alpha_fast_ / alpha_slow_));
  }
  p_random = std::min(p_random, config_.random_injection_max_ratio);

  const int random_count =
      static_cast<int>(static_cast<double>(particles_.size()) * p_random);

  double min_x = -config_.init_field_width / 2.0;
  double max_x = config_.init_field_width / 2.0;
  double min_y = -config_.init_field_height / 2.0;
  double max_y = config_.init_field_height / 2.0;
  if (map_initialized_) {
    min_x = map_origin_x_;
    max_x = map_origin_x_ + (distance_map_.cols * map_resolution_);
    min_y = map_origin_y_;
    max_y = map_origin_y_ + (distance_map_.rows * map_resolution_);
  }

  std::uniform_real_distribution<double> dist_x(min_x, max_x);
  std::uniform_real_distribution<double> dist_y(min_y, max_y);
  std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

  for (int j = 0; j < random_count; ++j) {
    new_particles[new_particles.size() - 1 - j] = {
        dist_x(gen_), dist_y(gen_), dist_theta(gen_),
        1.0 / static_cast<double>(particles_.size())};
  }

  particles_ = new_particles;
}

Particle ParticleFilterAmclFusion::getBestPose() const {
  Particle best = particles_.front();
  for (const auto &particle : particles_) {
    if (particle.weight > best.weight) {
      best = particle;
    }
  }
  return best;
}

} // namespace rcj_loc
