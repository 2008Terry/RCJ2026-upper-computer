#pragma once

#include <random>
#include <vector>

#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "rcj_localization/particle_filter_v2.hpp"

namespace rcj_loc {

using ParticleFilterAmclFusionConfig = ParticleFilterV2Config;

class ParticleFilterAmclFusion {
public:
    explicit ParticleFilterAmclFusion(const ParticleFilterAmclFusionConfig &config);

    void setConfig(const ParticleFilterAmclFusionConfig &config);
    const ParticleFilterAmclFusionConfig &getConfig() const { return config_; }

    void initRandom();
    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg);
    void predict(double absolute_yaw);
    void predict(double absolute_yaw, double dx, double dy);
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

}  // namespace rcj_loc
