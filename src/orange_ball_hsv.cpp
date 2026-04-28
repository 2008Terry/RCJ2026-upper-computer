#include "rcj_localization/orange_ball_hsv.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace rcj_localization
{

int clampOrangeHue(int value)
{
  return std::clamp(value, 0, kOrangeBallHueMax);
}

int clampOrangeByte(int value)
{
  return std::clamp(value, 0, kOrangeBallByteMax);
}

int normalizeOrangeBallMorphKernelSize(int value)
{
  return std::max(1, value) | 1;
}

OrangeBallHsvConfig sanitizeOrangeBallHsvConfig(OrangeBallHsvConfig config)
{
  config.h_min = clampOrangeHue(config.h_min);
  config.h_max = clampOrangeHue(config.h_max);
  config.s_min = clampOrangeByte(config.s_min);
  config.v_min = clampOrangeByte(config.v_min);
  config.morph_kernel_size = normalizeOrangeBallMorphKernelSize(config.morph_kernel_size);
  return config;
}

void buildOrangeBallThresholdMask(
  const cv::Mat & bgr,
  const OrangeBallHsvConfig & raw_config,
  cv::Mat & mask,
  const cv::Mat & allowed_mask)
{
  const OrangeBallHsvConfig config = sanitizeOrangeBallHsvConfig(raw_config);

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  if (config.h_min <= config.h_max) {
    cv::inRange(
      hsv,
      cv::Scalar(config.h_min, config.s_min, config.v_min),
      cv::Scalar(config.h_max, kOrangeBallByteMax, kOrangeBallByteMax),
      mask);
  } else {
    cv::Mat low_mask;
    cv::Mat high_mask;
    cv::inRange(
      hsv,
      cv::Scalar(0, config.s_min, config.v_min),
      cv::Scalar(config.h_max, kOrangeBallByteMax, kOrangeBallByteMax),
      low_mask);
    cv::inRange(
      hsv,
      cv::Scalar(config.h_min, config.s_min, config.v_min),
      cv::Scalar(kOrangeBallHueMax, kOrangeBallByteMax, kOrangeBallByteMax),
      high_mask);
    cv::bitwise_or(low_mask, high_mask, mask);
  }

  if (!allowed_mask.empty()) {
    CV_Assert(allowed_mask.size() == mask.size());
    cv::bitwise_and(mask, allowed_mask, mask);
  }
}

void applyOrangeBallMorphOpen(cv::Mat & mask, const OrangeBallHsvConfig & raw_config)
{
  const OrangeBallHsvConfig config = sanitizeOrangeBallHsvConfig(raw_config);
  if (!config.enable_morph_open || config.morph_kernel_size <= 1) {
    return;
  }

  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(config.morph_kernel_size, config.morph_kernel_size));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
}

void buildOrangeBallMask(
  const cv::Mat & bgr,
  const OrangeBallHsvConfig & config,
  cv::Mat & mask,
  const cv::Mat & allowed_mask)
{
  buildOrangeBallThresholdMask(bgr, config, mask, allowed_mask);
  applyOrangeBallMorphOpen(mask, config);
}

}  // namespace rcj_localization
