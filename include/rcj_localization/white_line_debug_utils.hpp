#pragma once

#include <chrono>
#include <limits>
#include <optional>
#include <string>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>

namespace rcj_loc::vision::debug {

struct ComponentStatsFilter {
    int min_area = 0;
    int max_area = std::numeric_limits<int>::max();
    int min_major_axis = 0;
    int max_minor_axis = std::numeric_limits<int>::max();
    double min_aspect_ratio = 1.0;
};

int makeOdd(int value, int minimum = 3);
bool consumeFpsGate(
    double max_fps,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point &last_publish_time);

cv::Mat filterComponentsByStats(const cv::Mat &binary, const ComponentStatsFilter &filter);
cv::Mat createMaskOverlay(
    const cv::Mat &frame,
    const cv::Mat &mask,
    const cv::Scalar &color = cv::Scalar(0, 255, 0),
    double alpha = 0.45);
std::optional<sensor_msgs::msg::CompressedImage> encodeJpegCompressedImage(
    const std_msgs::msg::Header &header,
    const std::string &encoding,
    const cv::Mat &image,
    int jpeg_quality,
    long long *encode_duration_us = nullptr);

}  // namespace rcj_loc::vision::debug
