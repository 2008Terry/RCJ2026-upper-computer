#pragma once

#include <opencv2/core.hpp>

namespace rcj_localization
{

constexpr int kOrangeBallHueMax = 179;
constexpr int kOrangeBallByteMax = 255;

struct OrangeBallHsvConfig
{
  int h_min = 5;
  int h_max = 30;
  int s_min = 100;
  int v_min = 60;
  bool enable_morph_open = true;
  int morph_kernel_size = 3;
};

int clampOrangeHue(int value);
int clampOrangeByte(int value);
int normalizeOrangeBallMorphKernelSize(int value);
OrangeBallHsvConfig sanitizeOrangeBallHsvConfig(OrangeBallHsvConfig config);

void buildOrangeBallThresholdMask(
  const cv::Mat & bgr,
  const OrangeBallHsvConfig & config,
  cv::Mat & mask,
  const cv::Mat & allowed_mask = cv::Mat());

void applyOrangeBallMorphOpen(cv::Mat & mask, const OrangeBallHsvConfig & config);

void buildOrangeBallMask(
  const cv::Mat & bgr,
  const OrangeBallHsvConfig & config,
  cv::Mat & mask,
  const cv::Mat & allowed_mask = cv::Mat());

}  // namespace rcj_localization
