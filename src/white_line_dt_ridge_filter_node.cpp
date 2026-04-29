#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "rcj_localization/white_line_debug_utils.hpp"

namespace {

constexpr char kMorphWindow[] = "DT Ridge Filter Input Morph Mask";
constexpr char kDistanceTransformWindow[] = "DT Ridge Filter Distance Transform";
constexpr char kGreenWindow[] = "DT Ridge Filter Input Green Mask";
constexpr char kBlackWindow[] = "DT Ridge Filter Input Black Mask";
constexpr char kNoiseWindow[] = "DT Ridge Filter Input Noise Mask";
constexpr char kRidgeWindow[] = "DT Ridge Filter Ridge";
constexpr char kCandidatePrefilterWindow[] = "DT Ridge Filter Candidate Prefilter";
constexpr char kOrientationValidWindow[] = "DT Ridge Filter Orientation Valid";
constexpr char kSideSupportSeedWindow[] = "DT Ridge Filter Side Support Seeds";
constexpr char kSideSupportWindow[] = "DT Ridge Filter Side Color Filter";
constexpr char kWidthSupportedWindow[] = "DT Ridge Filter Width Filter";
constexpr char kLengthFilteredRidgeWindow[] = "DT Ridge Filter Length-Filtered Ridge";
constexpr char kReconstructedWindow[] = "DT Ridge Filter Reconstruction";
constexpr char kWhiteFinalMaskWindow[] = "DT Ridge Filter White Final Mask";
constexpr char kDebugWindow[] = "DT Ridge Filter Composite Debug";
constexpr bool kDefaultShowLengthFilteredRidgeMask = true;
constexpr uchar kLabelOther = 0;
constexpr uchar kLabelGreen = 1;
constexpr uchar kLabelBlack = 2;
constexpr uchar kLabelNoise = 3;

struct SeedPoint
{
  cv::Point point;
  int dir_bin = 0;
  int start_offset = 1;
  int tangent_half_span = 1;
};

struct SeedCandidate
{
  cv::Point point;
  int start_offset = 1;
  int tangent_half_span = 1;
};

struct RidgeFrameStats
{
  double ridge_point_count = 0.0;
  double prefiltered_ridge_point_count = 0.0;
  double orientation_valid_seed_count = 0.0;
  double width_supported_point_count = 0.0;
  double seed_point_count = 0.0;
  double supported_seed_point_count = 0.0;
  double total_side_samples = 0.0;
  double avg_samples_per_seed = 0.0;
  double max_local_width_px = 0.0;
};

cv::Size fitWithinBounds(const cv::Size & image_size, int max_width, int max_height)
{
  const int safe_max_width = std::max(1, max_width);
  const int safe_max_height = std::max(1, max_height);
  if (image_size.width <= 0 || image_size.height <= 0) {
    return cv::Size(safe_max_width, safe_max_height);
  }

  const double width_scale =
    static_cast<double>(safe_max_width) / static_cast<double>(image_size.width);
  const double height_scale =
    static_cast<double>(safe_max_height) / static_cast<double>(image_size.height);
  const double scale = std::min(1.0, std::min(width_scale, height_scale));

  return cv::Size(
    std::max(1, static_cast<int>(std::round(image_size.width * scale))),
    std::max(1, static_cast<int>(std::round(image_size.height * scale))));
}

void resizeWindowToFitImage(
  const std::string & window_name,
  const cv::Mat & image,
  int max_width,
  int max_height)
{
  const cv::Size fitted_size = fitWithinBounds(image.size(), max_width, max_height);
  cv::resizeWindow(window_name, fitted_size.width, fitted_size.height);
}

int computeStartOffset(float local_width_px, int side_margin_px)
{
  const float half_width = 0.5F * std::max(local_width_px, 0.0F);
  return std::max(1, static_cast<int>(std::round(half_width + static_cast<float>(side_margin_px))));
}

int computeTangentHalfSpan(float local_width_px)
{
  const float half_width = 0.5F * std::max(local_width_px, 0.0F);
  return std::max(1, static_cast<int>(std::round(std::max(1.0F, half_width))));
}

int quantizeDirectionBin(const cv::Point2f & tangent, int direction_bins)
{
  const int safe_bins = std::max(1, direction_bins);
  const double step = CV_PI / static_cast<double>(safe_bins);
  double angle = std::atan2(static_cast<double>(tangent.y), static_cast<double>(tangent.x));
  if (angle < 0.0) {
    angle += CV_PI;
  }
  if (angle >= CV_PI) {
    angle -= CV_PI;
  }
  int bin = static_cast<int>(std::floor(angle / step));
  if (bin >= safe_bins) {
    bin = safe_bins - 1;
  }
  return std::max(0, bin);
}

std::uint64_t makeSideTemplateKey(
  int dir_bin,
  int start_offset,
  int tangent_half_span,
  int side_sign)
{
  const std::uint64_t side_index = side_sign < 0 ? 0ULL : 1ULL;
  return (static_cast<std::uint64_t>(dir_bin) & 0xFFULL) |
         ((static_cast<std::uint64_t>(start_offset) & 0xFFFFULL) << 8ULL) |
         ((static_cast<std::uint64_t>(tangent_half_span) & 0xFFFFULL) << 24ULL) |
         (side_index << 40ULL);
}

std::vector<cv::Point> buildSideTemplateOffsets(
  int dir_bin,
  int direction_bins,
  int start_offset,
  int tangent_half_span,
  int side_band_depth_px,
  int side_sign)
{
  const int safe_bins = std::max(1, direction_bins);
  const double angle_step = CV_PI / static_cast<double>(safe_bins);
  const double angle = (static_cast<double>(dir_bin) + 0.5) * angle_step;
  const cv::Point2f tangent(
    static_cast<float>(std::cos(angle)),
    static_cast<float>(std::sin(angle)));
  const cv::Point2f normal(-tangent.y, tangent.x);

  std::vector<cv::Point> offsets;
  offsets.reserve(
    static_cast<std::size_t>((2 * tangent_half_span + 1) * std::max(1, side_band_depth_px)));
  for (int tangential_step = -tangent_half_span; tangential_step <= tangent_half_span;
    ++tangential_step)
  {
    const cv::Point2f tangential_offset = static_cast<float>(tangential_step) * tangent;
    for (int depth_step = 0; depth_step < std::max(1, side_band_depth_px); ++depth_step) {
      const float offset = static_cast<float>(start_offset + depth_step);
      const cv::Point2f sample =
        tangential_offset + static_cast<float>(side_sign) * offset * normal;
      offsets.emplace_back(
        static_cast<int>(std::round(sample.x)),
        static_cast<int>(std::round(sample.y)));
    }
  }
  return offsets;
}

bool estimateOrientation(
  const cv::Mat & centerline_mask,
  const cv::Point & center,
  const std::vector<cv::Point> & circle_offsets,
  int min_neighbors,
  cv::Point2f & tangent,
  cv::Point2f & normal)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  int count = 0;

  for (const cv::Point & offset : circle_offsets) {
    const int x = center.x + offset.x;
    const int y = center.y + offset.y;
    if (x < 0 || x >= centerline_mask.cols || y < 0 || y >= centerline_mask.rows) {
      continue;
    }
    if (centerline_mask.at<uchar>(y, x) == 0) {
      continue;
    }

    const double x_d = static_cast<double>(x);
    const double y_d = static_cast<double>(y);
    ++count;
    sum_x += x_d;
    sum_y += y_d;
    sum_xx += x_d * x_d;
    sum_xy += x_d * y_d;
    sum_yy += y_d * y_d;
  }

  if (count < std::max(2, min_neighbors)) {
    return false;
  }

  const double mean_x = sum_x / static_cast<double>(count);
  const double mean_y = sum_y / static_cast<double>(count);
  const double cov_xx = sum_xx - static_cast<double>(count) * mean_x * mean_x;
  const double cov_xy = sum_xy - static_cast<double>(count) * mean_x * mean_y;
  const double cov_yy = sum_yy - static_cast<double>(count) * mean_y * mean_y;

  const double trace = cov_xx + cov_yy;
  const double det = cov_xx * cov_yy - cov_xy * cov_xy;
  const double delta = std::sqrt(std::max(0.0, 0.25 * trace * trace - det));
  const double lambda1 = 0.5 * trace + delta;

  cv::Point2f eigenvector;
  if (std::abs(cov_xy) > 1e-6 || std::abs(lambda1 - cov_xx) > 1e-6) {
    eigenvector = cv::Point2f(
      static_cast<float>(cov_xy),
      static_cast<float>(lambda1 - cov_xx));
  } else if (cov_xx >= cov_yy) {
    eigenvector = cv::Point2f(1.0F, 0.0F);
  } else {
    eigenvector = cv::Point2f(0.0F, 1.0F);
  }

  const float norm = std::sqrt(eigenvector.x * eigenvector.x + eigenvector.y * eigenvector.y);
  if (norm < 1e-6F) {
    return false;
  }

  tangent = cv::Point2f(eigenvector.x / norm, eigenvector.y / norm);
  normal = cv::Point2f(-tangent.y, tangent.x);
  return true;
}

std::vector<cv::Point> buildOrientationCircleOffsets(int radius)
{
  const int clamped_radius = std::max(1, radius);
  const int radius_sq = clamped_radius * clamped_radius;
  std::vector<cv::Point> offsets;
  offsets.reserve(static_cast<std::size_t>((2 * clamped_radius + 1) * (2 * clamped_radius + 1)));
  for (int dy = -clamped_radius; dy <= clamped_radius; ++dy) {
    for (int dx = -clamped_radius; dx <= clamped_radius; ++dx) {
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }
      offsets.emplace_back(dx, dy);
    }
  }
  return offsets;
}

struct SideSampleStats
{
  int total_samples = 0;
  int green_samples = 0;
  int black_samples = 0;
  int noise_samples = 0;
  int outside_samples = 0;

  double greenRatio() const
  {
    return total_samples > 0 ? static_cast<double>(green_samples) / total_samples : 0.0;
  }

  double boundaryRatio() const
  {
    return total_samples > 0
             ? static_cast<double>(black_samples + noise_samples + outside_samples) / total_samples
             : 0.0;
  }
};

SideSampleStats sampleSideSupportFromTemplate(
  const cv::Point & center,
  const cv::Mat & label_map,
  const std::vector<cv::Point> & offsets)
{
  SideSampleStats stats;
  for (const cv::Point & offset : offsets) {
    const int sample_x = center.x + offset.x;
    const int sample_y = center.y + offset.y;
    ++stats.total_samples;
    if (sample_x < 0 || sample_x >= label_map.cols || sample_y < 0 || sample_y >= label_map.rows) {
      ++stats.outside_samples;
      continue;
    }

    const uchar label = label_map.at<uchar>(sample_y, sample_x);
    if (label == kLabelGreen) {
      ++stats.green_samples;
    } else if (label == kLabelBlack) {
      ++stats.black_samples;
    } else if (label == kLabelNoise) {
      ++stats.noise_samples;
    }
  }
  return stats;
}

cv::Mat filterMaskByLength(const cv::Mat & binary_mask, int min_length_pixels)
{
  CV_Assert(binary_mask.type() == CV_8UC1);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8, CV_32S);

  cv::Mat filtered = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
  for (int label = 1; label < stats.rows; ++label) {
    const int component_area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (component_area < min_length_pixels) {
      continue;
    }
    filtered.setTo(255, labels == label);
  }

  return filtered;
}

cv::Mat filterMaskByMinComponentArea(const cv::Mat & binary_mask, int min_component_area)
{
  CV_Assert(binary_mask.type() == CV_8UC1);
  if (min_component_area <= 1) {
    return binary_mask.clone();
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8, CV_32S);

  cv::Mat filtered = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
  for (int label = 1; label < stats.rows; ++label) {
    const int component_area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (component_area < min_component_area) {
      continue;
    }
    filtered.setTo(255, labels == label);
  }
  return filtered;
}

cv::Mat pruneMaskEndpoints(const cv::Mat & binary_mask, int rounds)
{
  CV_Assert(binary_mask.type() == CV_8UC1);
  cv::Mat pruned = binary_mask.clone();
  const int safe_rounds = std::max(0, rounds);
  for (int round = 0; round < safe_rounds; ++round) {
    cv::Mat to_remove = cv::Mat::zeros(pruned.size(), CV_8UC1);
    for (int y = 0; y < pruned.rows; ++y) {
      for (int x = 0; x < pruned.cols; ++x) {
        if (pruned.at<uchar>(y, x) == 0) {
          continue;
        }
        int neighbor_count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= pruned.cols || ny < 0 || ny >= pruned.rows) {
              continue;
            }
            if (pruned.at<uchar>(ny, nx) != 0) {
              ++neighbor_count;
            }
          }
        }
        if (neighbor_count <= 1) {
          to_remove.at<uchar>(y, x) = 255;
        }
      }
    }
    if (cv::countNonZero(to_remove) == 0) {
      break;
    }
    pruned.setTo(0, to_remove);
  }
  return pruned;
}

cv::Mat buildLabelMap(
  const cv::Mat & green_mask,
  const cv::Mat & black_mask,
  const cv::Mat & noise_mask)
{
  CV_Assert(green_mask.type() == CV_8UC1);
  CV_Assert(black_mask.type() == CV_8UC1);
  CV_Assert(noise_mask.type() == CV_8UC1);

  cv::Mat label_map(green_mask.size(), CV_8UC1, cv::Scalar(kLabelOther));
  label_map.setTo(cv::Scalar(kLabelGreen), green_mask);
  label_map.setTo(cv::Scalar(kLabelBlack), black_mask);
  label_map.setTo(cv::Scalar(kLabelNoise), noise_mask);
  return label_map;
}

bool isLocalMaxPair(float center, float first, float second)
{
  return center >= first && center >= second && (center > first || center > second);
}

cv::Mat extractDistanceTransformRidgeMask(
  const cv::Mat & morph_mask,
  const cv::Mat & distance_transform)
{
  CV_Assert(morph_mask.type() == CV_8UC1);
  CV_Assert(distance_transform.type() == CV_32FC1);

  cv::Mat ridge_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);

  auto sample_distance = [&](int x, int y) -> float {
      if (x < 0 || x >= distance_transform.cols || y < 0 || y >= distance_transform.rows) {
        return 0.0F;
      }
      return distance_transform.at<float>(y, x);
    };

  for (int y = 0; y < morph_mask.rows; ++y) {
    for (int x = 0; x < morph_mask.cols; ++x) {
      if (morph_mask.at<uchar>(y, x) == 0) {
        continue;
      }

      const float center = distance_transform.at<float>(y, x);
      if (center <= 0.0F) {
        continue;
      }

      const bool is_ridge =
        isLocalMaxPair(center, sample_distance(x - 1, y), sample_distance(x + 1, y)) ||
        isLocalMaxPair(center, sample_distance(x, y - 1), sample_distance(x, y + 1)) ||
        isLocalMaxPair(center, sample_distance(x - 1, y - 1), sample_distance(x + 1, y + 1)) ||
        isLocalMaxPair(center, sample_distance(x - 1, y + 1), sample_distance(x + 1, y - 1));

      if (is_ridge) {
        ridge_mask.at<uchar>(y, x) = 255;
      }
    }
  }

  return ridge_mask;
}

cv::Mat createDebugComposite(
  const cv::Mat & green_mask,
  const cv::Mat & black_mask,
  const cv::Mat & white_final_mask)
{
  cv::Mat debug_image(green_mask.size(), CV_8UC3, cv::Scalar(30, 70, 30));
  debug_image =
    rcj_loc::vision::debug::createMaskOverlay(debug_image, black_mask, cv::Scalar(0, 0, 255));
  debug_image =
    rcj_loc::vision::debug::createMaskOverlay(debug_image, green_mask, cv::Scalar(0, 200, 0));
  debug_image =
    rcj_loc::vision::debug::createMaskOverlay(debug_image, white_final_mask, cv::Scalar(255, 255, 255));
  return debug_image;
}

cv::Mat createDistanceTransformDebugImage(const cv::Mat & distance_transform)
{
  CV_Assert(distance_transform.type() == CV_32FC1);

  double min_value = 0.0;
  double max_value = 0.0;
  cv::minMaxLoc(distance_transform, &min_value, &max_value);

  cv::Mat normalized_u8;
  if (max_value <= 0.0) {
    normalized_u8 = cv::Mat::zeros(distance_transform.size(), CV_8UC1);
  } else {
    distance_transform.convertTo(normalized_u8, CV_8UC1, 255.0 / max_value);
  }

  cv::Mat colorized;
  cv::applyColorMap(normalized_u8, colorized, cv::COLORMAP_TURBO);
  return colorized;
}

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

long long elapsedUs(const TimePoint & start, const TimePoint & end)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

enum class RidgeTimingStage : std::size_t
{
  RuntimeSync = 0,
  CvBridgeConvert,
  DistanceTransform,
  DistanceTransformDebug,
  RidgeExtract,
  CandidatePrefilter,
  StatsCollect,
  OrientationEstimate,
  LabelMapBuild,
  WidthFilterPreSupport,
  SideSupportSeedSelect,
  SideTemplatePrepare,
  SideSupportSeedScan,
  SideSupportPropagate,
  LengthFilter,
  Reconstruct,
  FinalMask,
  DebugComposite,
  DebugCompression,
  PublishOutputs,
  GuiDisplay,
  UnaccountedOverhead,
  CallbackTotal,
  Count
};

constexpr std::array<const char *, static_cast<std::size_t>(RidgeTimingStage::Count)>
  kRidgeTimingLabels = {
    "runtime_sync",
    "cv_bridge",
    "distance_transform",
    "distance_transform_debug",
    "ridge_extract",
    "candidate_prefilter",
    "stats_collect",
    "orientation_seed_estimate",
    "label_map_build",
    "width_filter_pre_support",
    "side_support_seed_select",
    "side_template_prepare",
    "side_support_seed_scan",
    "side_support_propagate",
    "length_filter",
    "reconstruct",
    "final_mask",
    "debug_composite",
    "debug_compression",
    "publish_outputs",
    "gui_display",
    "unaccounted_overhead",
    "callback_total",
  };

using RidgeTimingArray =
  std::array<long long, static_cast<std::size_t>(RidgeTimingStage::Count)>;

enum class RidgeStatIndex : std::size_t
{
  RidgePoints = 0,
  PrefilteredRidgePoints,
  OrientationValidSeeds,
  WidthSupportedPoints,
  SeedPoints,
  SupportedSeedPoints,
  TotalSideSamples,
  AvgSamplesPerSeed,
  MaxLocalWidthPx,
  Count
};

constexpr std::array<const char *, static_cast<std::size_t>(RidgeStatIndex::Count)>
  kRidgeStatLabels = {
    "ridge_point_count",
    "prefiltered_ridge_point_count",
    "orientation_valid_seed_count",
    "width_supported_point_count",
    "seed_point_count",
    "supported_seed_point_count",
    "total_side_samples",
    "avg_samples_per_seed",
    "max_local_width_px",
  };

using RidgeStatArray =
  std::array<double, static_cast<std::size_t>(RidgeStatIndex::Count)>;

struct RidgeFrameTiming
{
  RidgeTimingArray stage_us{};
};

RidgeStatArray ridgeStatsToArray(const RidgeFrameStats & stats)
{
  RidgeStatArray values{};
  values[static_cast<std::size_t>(RidgeStatIndex::RidgePoints)] = stats.ridge_point_count;
  values[static_cast<std::size_t>(RidgeStatIndex::PrefilteredRidgePoints)] =
    stats.prefiltered_ridge_point_count;
  values[static_cast<std::size_t>(RidgeStatIndex::OrientationValidSeeds)] =
    stats.orientation_valid_seed_count;
  values[static_cast<std::size_t>(RidgeStatIndex::WidthSupportedPoints)] =
    stats.width_supported_point_count;
  values[static_cast<std::size_t>(RidgeStatIndex::SeedPoints)] = stats.seed_point_count;
  values[static_cast<std::size_t>(RidgeStatIndex::SupportedSeedPoints)] =
    stats.supported_seed_point_count;
  values[static_cast<std::size_t>(RidgeStatIndex::TotalSideSamples)] = stats.total_side_samples;
  values[static_cast<std::size_t>(RidgeStatIndex::AvgSamplesPerSeed)] = stats.avg_samples_per_seed;
  values[static_cast<std::size_t>(RidgeStatIndex::MaxLocalWidthPx)] = stats.max_local_width_px;
  return values;
}

void recordStageDuration(
  RidgeTimingArray & target,
  RidgeTimingStage stage,
  const TimePoint & start,
  const TimePoint & end)
{
  target[static_cast<std::size_t>(stage)] = elapsedUs(start, end);
}

long long sumTimingStagesExcludingCallback(const RidgeTimingArray & values)
{
  long long total_us = 0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (
      i == static_cast<std::size_t>(RidgeTimingStage::UnaccountedOverhead) ||
      i == static_cast<std::size_t>(RidgeTimingStage::CallbackTotal))
    {
      continue;
    }
    total_us += values[i];
  }
  return total_us;
}

template<typename PublisherT>
std::size_t subscriptionCount(const std::shared_ptr<PublisherT> & publisher)
{
  return publisher == nullptr ? 0U : publisher->get_subscription_count();
}

template<typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> & publisher)
{
  return subscriptionCount(publisher) > 0U;
}

template<typename PublisherT>
bool publishImageIfSubscribed(
  const std::shared_ptr<PublisherT> & publisher,
  const std_msgs::msg::Header & header,
  const std::string & encoding,
  const cv::Mat & image)
{
  if (!hasSubscribers(publisher)) {
    return false;
  }
  publisher->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
  return true;
}

void appendTimingTable(
  std::ostringstream & oss,
  const RidgeFrameTiming & timing,
  const RidgeTimingArray & interval_totals,
  std::size_t interval_frame_count)
{
  const double callback_average_ms =
    interval_frame_count > 0
      ? static_cast<double>(
      interval_totals[static_cast<std::size_t>(RidgeTimingStage::CallbackTotal)]) /
      static_cast<double>(interval_frame_count) / 1000.0
      : 0.0;

  oss << std::left << std::setw(22) << "阶段"
      << std::right << std::setw(12) << "当前ms"
      << std::setw(14) << "区间平均ms"
      << std::setw(12) << "占比%" << '\n';

  for (std::size_t i = 0; i < kRidgeTimingLabels.size(); ++i) {
    const double current_ms = static_cast<double>(timing.stage_us[i]) / 1000.0;
    const double interval_average_ms =
      interval_frame_count > 0
        ? static_cast<double>(interval_totals[i]) / static_cast<double>(interval_frame_count) /
        1000.0
        : 0.0;
    const double ratio =
      callback_average_ms > 0.0 ? (interval_average_ms / callback_average_ms) * 100.0 : 0.0;

    oss << std::left << std::setw(22) << kRidgeTimingLabels[i]
        << std::right << std::setw(12) << std::fixed << std::setprecision(3) << current_ms
        << std::setw(14) << std::fixed << std::setprecision(3) << interval_average_ms
        << std::setw(12) << std::fixed << std::setprecision(1) << ratio << '\n';
  }
}

void appendStatsTable(
  std::ostringstream & oss,
  const RidgeFrameStats & stats,
  const RidgeStatArray & interval_totals,
  std::size_t interval_frame_count)
{
  const RidgeStatArray current = ridgeStatsToArray(stats);
  oss << '\n'
      << std::left << std::setw(28) << "统计项"
      << std::right << std::setw(14) << "当前"
      << std::setw(16) << "区间平均" << '\n';

  for (std::size_t i = 0; i < kRidgeStatLabels.size(); ++i) {
    const double interval_average =
      interval_frame_count > 0 ? interval_totals[i] / static_cast<double>(interval_frame_count) : 0.0;
    oss << std::left << std::setw(28) << kRidgeStatLabels[i]
        << std::right << std::setw(14) << std::fixed << std::setprecision(2) << current[i]
        << std::setw(16) << std::fixed << std::setprecision(2) << interval_average << '\n';
  }
}

}  // namespace

class WhiteLineDtRidgeFilterNode : public rclcpp::Node
{
public:
  WhiteLineDtRidgeFilterNode()
  : Node("white_line_dt_ridge_filter_node")
  {
    declare_parameter<std::string>("morph_mask_topic", "/white_line_hsv_white_node/white_mask");
    declare_parameter<std::string>("green_mask_topic", "/white_line_hsv_white_node/green_mask");
    declare_parameter<std::string>("black_mask_topic", "/white_line_hsv_white_node/black_mask");
    declare_parameter<std::string>("noise_mask_topic", "/white_line_hsv_white_node/noise_mask");
    declare_parameter("orientation_window_radius_px", 5);
    declare_parameter("min_orientation_neighbors", 6);
    declare_parameter("enable_orientation_estimate", false);
    declare_parameter("side_margin_px", 1);
    declare_parameter("side_band_depth_px", 4);
    declare_parameter("min_green_ratio", 0.35);
    declare_parameter("min_boundary_ratio", 0.35);
    declare_parameter("enable_boundary_mode", true);
    declare_parameter("width_floor_px", 2.0);
    declare_parameter("width_ceil_px", 40.0);
    declare_parameter("width_mad_scale", 2.5);
    declare_parameter("min_width_samples", 25);
    declare_parameter("enable_candidate_prefilter", false);
    declare_parameter("candidate_min_component_px", 3);
    declare_parameter("candidate_prune_rounds", 1);
    declare_parameter("side_scan_stride", 3);
    declare_parameter("side_template_direction_bins", 16);
    declare_parameter("enable_parallel_side_scan", true);
    declare_parameter("enable_parallel_orientation_estimate", true);
    declare_parameter("enable_length_filter", false);
    declare_parameter("min_skeleton_length_px", 12);
    declare_parameter("reconstruction_margin_px", 1.0);
    declare_parameter("enable_image_view", false);
    declare_parameter("show_morph_mask", true);
    declare_parameter("show_distance_transform", false);
    declare_parameter("show_green_mask", false);
    declare_parameter("show_black_mask", false);
    declare_parameter("show_noise_mask", false);
    declare_parameter("show_ridge_mask", true);
    declare_parameter("show_skeleton_mask", true);
    declare_parameter("show_candidate_prefilter_mask", false);
    declare_parameter("show_orientation_valid_mask", true);
    declare_parameter("show_side_support_seed_mask", false);
    declare_parameter("show_side_support_mask", true);
    declare_parameter("show_width_supported_ridge_mask", true);
    declare_parameter("show_width_supported_skeleton_mask", true);
    declare_parameter("show_length_filtered_ridge_mask", kDefaultShowLengthFilteredRidgeMask);
    declare_parameter("show_length_filtered_skeleton_mask", kDefaultShowLengthFilteredRidgeMask);
    declare_parameter("show_supported_skeleton_mask", kDefaultShowLengthFilteredRidgeMask);
    declare_parameter("show_reconstructed_mask", true);
    declare_parameter("show_white_final_mask", true);
    declare_parameter("show_white_mask", true);
    declare_parameter("show_debug_image", true);
    declare_parameter("publish_debug_images", false);
    declare_parameter("publish_morph_mask", true);
    declare_parameter("publish_distance_transform", true);
    declare_parameter("publish_green_mask", true);
    declare_parameter("publish_black_mask", true);
    declare_parameter("publish_noise_mask", true);
    declare_parameter("publish_ridge_mask", true);
    declare_parameter("publish_candidate_prefilter_mask", true);
    declare_parameter("publish_orientation_valid_mask", true);
    declare_parameter("publish_side_support_seed_mask", true);
    declare_parameter("publish_side_support_mask", true);
    declare_parameter("publish_width_supported_ridge_mask", true);
    declare_parameter("publish_length_filtered_ridge_mask", true);
    declare_parameter("publish_reconstructed_mask", true);
    declare_parameter("publish_white_final_mask", true);
    declare_parameter("publish_debug_image", true);
    declare_parameter("debug_jpeg_quality", 80);
    declare_parameter("debug_image_max_fps", 5.0);
    declare_parameter("enable_timing_debug", false);
    declare_parameter("timing_summary_interval", 10);
    declare_parameter("display_max_width", 960);
    declare_parameter("display_max_height", 720);

    syncImageViewState();
    setupSubscribers();

    ridge_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/ridge_mask", 10);
    distance_transform_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/distance_transform_image", 10);
    legacy_skeleton_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/skeleton_mask", 10);
    candidate_prefilter_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/candidate_prefilter_mask", 10);
    orientation_valid_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/orientation_valid_mask", 10);
    side_support_seed_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/side_support_seed_mask", 10);
    side_support_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/side_support_mask", 10);
    width_supported_ridge_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/width_supported_ridge_mask", 10);
    legacy_width_supported_skeleton_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/width_supported_skeleton_mask", 10);
    length_filtered_ridge_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/length_filtered_ridge_mask", 10);
    legacy_length_filtered_skeleton_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/length_filtered_skeleton_mask", 10);
    legacy_supported_skeleton_mask_pub_ =
      create_publisher<sensor_msgs::msg::Image>("~/supported_skeleton_mask", 10);
    reconstructed_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/reconstructed_mask", 10);
    white_final_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/white_final_mask", 10);
    legacy_white_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/white_mask", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);
    debug_morph_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/morph_mask", 10);
    debug_green_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/green_mask", 10);
    debug_black_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/black_mask", 10);
    debug_noise_mask_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug/noise_mask", 10);
    ridge_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/ridge_mask/compressed", 10);
    distance_transform_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/distance_transform_image/compressed", 10);
    legacy_skeleton_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/skeleton_mask/compressed", 10);
    candidate_prefilter_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/candidate_prefilter_mask/compressed", 10);
    orientation_valid_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/orientation_valid_mask/compressed", 10);
    side_support_seed_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/side_support_seed_mask/compressed", 10);
    side_support_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/side_support_mask/compressed", 10);
    width_supported_ridge_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/width_supported_ridge_mask/compressed", 10);
    legacy_width_supported_skeleton_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/width_supported_skeleton_mask/compressed", 10);
    length_filtered_ridge_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/length_filtered_ridge_mask/compressed", 10);
    legacy_length_filtered_skeleton_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/length_filtered_skeleton_mask/compressed", 10);
    legacy_supported_skeleton_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/supported_skeleton_mask/compressed", 10);
    reconstructed_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/reconstructed_mask/compressed", 10);
    white_final_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/white_final_mask/compressed", 10);
    legacy_white_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/white_mask/compressed", 10);
    debug_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/debug_image/compressed", 10);
    debug_morph_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/debug/morph_mask/compressed", 10);
    debug_green_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/debug/green_mask/compressed", 10);
    debug_black_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/debug/black_mask/compressed", 10);
    debug_noise_mask_compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/debug/noise_mask/compressed", 10);

    RCLCPP_INFO(
      get_logger(),
      "white_line_dt_ridge_filter_node started. enable_image_view=%s, enable_timing_debug=%s, "
      "timing_summary_interval=%d. Waiting for morph masks on '%s'.",
      enable_image_view_ ? "true" : "false",
      enable_timing_debug_ ? "true" : "false",
      timing_summary_interval_,
      morph_mask_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "DT ridge fixed-width mode active. Parameters 'width_mad_scale' and 'min_width_samples' "
      "are currently ignored.");
  }

  ~WhiteLineDtRidgeFilterNode() override
  {
    destroyDebugWindows();
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using ExactPolicy =
    message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg>;

  message_filters::Subscriber<ImageMsg> morph_sub_;
  message_filters::Subscriber<ImageMsg> green_sub_;
  message_filters::Subscriber<ImageMsg> black_sub_;
  message_filters::Subscriber<ImageMsg> noise_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> synchronizer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ridge_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr distance_transform_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_skeleton_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr candidate_prefilter_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orientation_valid_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr side_support_seed_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr side_support_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr width_supported_ridge_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_width_supported_skeleton_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr length_filtered_ridge_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_length_filtered_skeleton_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_supported_skeleton_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_final_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_white_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_morph_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_green_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_black_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_noise_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ridge_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr distance_transform_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr legacy_skeleton_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr candidate_prefilter_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr orientation_valid_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr side_support_seed_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr side_support_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr width_supported_ridge_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr legacy_width_supported_skeleton_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr length_filtered_ridge_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr legacy_length_filtered_skeleton_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr legacy_supported_skeleton_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr reconstructed_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr white_final_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr legacy_white_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_morph_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_green_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_black_mask_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_noise_mask_compressed_pub_;

  std::string morph_mask_topic_;
  std::string green_mask_topic_;
  std::string black_mask_topic_;
  std::string noise_mask_topic_;
  int orientation_window_radius_px_ = 5;
  int min_orientation_neighbors_ = 6;
  bool enable_orientation_estimate_ = false;
  int side_margin_px_ = 1;
  int side_band_depth_px_ = 4;
  double min_green_ratio_ = 0.35;
  double min_boundary_ratio_ = 0.35;
  bool enable_boundary_mode_ = true;
  double width_floor_px_ = 2.0;
  double width_ceil_px_ = 40.0;
  double width_mad_scale_ = 2.5;
  int min_width_samples_ = 25;
  bool enable_candidate_prefilter_ = false;
  int candidate_min_component_px_ = 3;
  int candidate_prune_rounds_ = 1;
  int side_scan_stride_ = 3;
  int side_template_direction_bins_ = 16;
  bool enable_parallel_side_scan_ = true;
  bool enable_parallel_orientation_estimate_ = true;
  bool enable_length_filter_ = false;
  int min_skeleton_length_px_ = 12;
  double reconstruction_margin_px_ = 1.0;
  bool enable_image_view_ = false;
  bool show_morph_mask_ = true;
  bool show_distance_transform_ = false;
  bool show_green_mask_ = false;
  bool show_black_mask_ = false;
  bool show_noise_mask_ = false;
  bool show_ridge_mask_ = true;
  bool show_candidate_prefilter_mask_ = false;
  bool show_orientation_valid_mask_ = true;
  bool show_side_support_seed_mask_ = false;
  bool show_side_support_mask_ = true;
  bool show_width_supported_ridge_mask_ = true;
  bool show_length_filtered_ridge_mask_ = kDefaultShowLengthFilteredRidgeMask;
  bool show_reconstructed_mask_ = true;
  bool show_white_final_mask_ = true;
  bool show_debug_image_ = true;
  bool publish_debug_images_ = false;
  bool publish_morph_mask_ = true;
  bool publish_distance_transform_ = true;
  bool publish_green_mask_ = true;
  bool publish_black_mask_ = true;
  bool publish_noise_mask_ = true;
  bool publish_ridge_mask_ = true;
  bool publish_candidate_prefilter_mask_ = true;
  bool publish_orientation_valid_mask_ = true;
  bool publish_side_support_seed_mask_ = true;
  bool publish_side_support_mask_ = true;
  bool publish_width_supported_ridge_mask_ = true;
  bool publish_length_filtered_ridge_mask_ = true;
  bool publish_reconstructed_mask_ = true;
  bool publish_white_final_mask_ = true;
  bool publish_debug_image_ = true;
  int debug_jpeg_quality_ = 80;
  double debug_image_max_fps_ = 5.0;
  bool enable_timing_debug_ = false;
  bool morph_window_created_ = false;
  bool distance_transform_window_created_ = false;
  bool green_window_created_ = false;
  bool black_window_created_ = false;
  bool noise_window_created_ = false;
  bool ridge_window_created_ = false;
  bool candidate_prefilter_window_created_ = false;
  bool orientation_valid_window_created_ = false;
  bool side_support_seed_window_created_ = false;
  bool side_support_window_created_ = false;
  bool width_supported_window_created_ = false;
  bool length_filtered_ridge_window_created_ = false;
  bool reconstructed_window_created_ = false;
  bool white_final_mask_window_created_ = false;
  bool debug_window_created_ = false;
  bool warned_deprecated_white_mask_topic_ = false;
  int display_max_width_ = 960;
  int display_max_height_ = 720;
  unsigned long long frame_count_ = 0;
  int timing_summary_interval_ = 10;
  std::size_t timing_frames_in_interval_ = 0;
  unsigned long long timing_interval_start_frame_ = 0;
  long long timing_interval_total_us_ = 0;
  long long timing_interval_max_us_ = 0;
  RidgeTimingArray timing_stage_interval_totals_{};
  RidgeStatArray stats_interval_totals_{};
  int cached_orientation_radius_ = -1;
  std::vector<cv::Point> orientation_circle_offsets_;
  int cached_template_direction_bins_ = -1;
  int cached_template_side_band_depth_ = -1;
  std::unordered_map<std::uint64_t, std::vector<cv::Point>> side_template_cache_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> debug_last_publish_times_;

  bool isParameterOverridden(const char * name)
  {
    const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
    return overrides.find(name) != overrides.end();
  }

  bool resolveBoolParameter(const char * current_name, std::initializer_list<const char *> legacy_names)
  {
    if (isParameterOverridden(current_name)) {
      return get_parameter(current_name).as_bool();
    }
    for (const char * legacy_name : legacy_names) {
      if (isParameterOverridden(legacy_name)) {
        return get_parameter(legacy_name).as_bool();
      }
    }
    return get_parameter(current_name).as_bool();
  }

  void loadRuntimeParameters()
  {
    morph_mask_topic_ = get_parameter("morph_mask_topic").as_string();
    green_mask_topic_ = get_parameter("green_mask_topic").as_string();
    black_mask_topic_ = get_parameter("black_mask_topic").as_string();
    noise_mask_topic_ = get_parameter("noise_mask_topic").as_string();
    orientation_window_radius_px_ =
      std::max(1, static_cast<int>(get_parameter("orientation_window_radius_px").as_int()));
    min_orientation_neighbors_ =
      std::max(2, static_cast<int>(get_parameter("min_orientation_neighbors").as_int()));
    enable_orientation_estimate_ = get_parameter("enable_orientation_estimate").as_bool();
    side_margin_px_ = std::max(0, static_cast<int>(get_parameter("side_margin_px").as_int()));
    side_band_depth_px_ =
      std::max(1, static_cast<int>(get_parameter("side_band_depth_px").as_int()));
    min_green_ratio_ = std::clamp(get_parameter("min_green_ratio").as_double(), 0.0, 1.0);
    min_boundary_ratio_ = std::clamp(get_parameter("min_boundary_ratio").as_double(), 0.0, 1.0);
    enable_boundary_mode_ = get_parameter("enable_boundary_mode").as_bool();
    width_floor_px_ = std::max(0.0, get_parameter("width_floor_px").as_double());
    width_ceil_px_ = std::max(width_floor_px_, get_parameter("width_ceil_px").as_double());
    width_mad_scale_ = std::max(0.0, get_parameter("width_mad_scale").as_double());
    min_width_samples_ = std::max(1, static_cast<int>(get_parameter("min_width_samples").as_int()));
    enable_candidate_prefilter_ = get_parameter("enable_candidate_prefilter").as_bool();
    candidate_min_component_px_ =
      std::max(1, static_cast<int>(get_parameter("candidate_min_component_px").as_int()));
    candidate_prune_rounds_ =
      std::max(0, static_cast<int>(get_parameter("candidate_prune_rounds").as_int()));
    side_scan_stride_ = std::max(1, static_cast<int>(get_parameter("side_scan_stride").as_int()));
    side_template_direction_bins_ =
      std::max(4, static_cast<int>(get_parameter("side_template_direction_bins").as_int()));
    enable_parallel_side_scan_ = get_parameter("enable_parallel_side_scan").as_bool();
    enable_parallel_orientation_estimate_ =
      get_parameter("enable_parallel_orientation_estimate").as_bool();
    enable_length_filter_ = get_parameter("enable_length_filter").as_bool();
    min_skeleton_length_px_ =
      std::max(1, static_cast<int>(get_parameter("min_skeleton_length_px").as_int()));
    reconstruction_margin_px_ =
      std::max(0.0, get_parameter("reconstruction_margin_px").as_double());
    enable_image_view_ = get_parameter("enable_image_view").as_bool();
    show_morph_mask_ = get_parameter("show_morph_mask").as_bool();
    show_distance_transform_ = get_parameter("show_distance_transform").as_bool();
    show_green_mask_ = get_parameter("show_green_mask").as_bool();
    show_black_mask_ = get_parameter("show_black_mask").as_bool();
    show_noise_mask_ = get_parameter("show_noise_mask").as_bool();
    show_ridge_mask_ = resolveBoolParameter("show_ridge_mask", {"show_skeleton_mask"});
    show_candidate_prefilter_mask_ = get_parameter("show_candidate_prefilter_mask").as_bool();
    show_orientation_valid_mask_ = get_parameter("show_orientation_valid_mask").as_bool();
    show_side_support_seed_mask_ = get_parameter("show_side_support_seed_mask").as_bool();
    show_side_support_mask_ = get_parameter("show_side_support_mask").as_bool();
    show_width_supported_ridge_mask_ = resolveBoolParameter(
      "show_width_supported_ridge_mask",
      {"show_width_supported_skeleton_mask"});
    show_length_filtered_ridge_mask_ = resolveBoolParameter(
      "show_length_filtered_ridge_mask",
      {"show_length_filtered_skeleton_mask", "show_supported_skeleton_mask"});
    show_reconstructed_mask_ = get_parameter("show_reconstructed_mask").as_bool();
    show_white_final_mask_ = resolveBoolParameter("show_white_final_mask", {"show_white_mask"});
    show_debug_image_ = get_parameter("show_debug_image").as_bool();
    publish_debug_images_ = get_parameter("publish_debug_images").as_bool();
    publish_morph_mask_ = get_parameter("publish_morph_mask").as_bool();
    publish_distance_transform_ = get_parameter("publish_distance_transform").as_bool();
    publish_green_mask_ = get_parameter("publish_green_mask").as_bool();
    publish_black_mask_ = get_parameter("publish_black_mask").as_bool();
    publish_noise_mask_ = get_parameter("publish_noise_mask").as_bool();
    publish_ridge_mask_ = get_parameter("publish_ridge_mask").as_bool();
    publish_candidate_prefilter_mask_ = get_parameter("publish_candidate_prefilter_mask").as_bool();
    publish_orientation_valid_mask_ = get_parameter("publish_orientation_valid_mask").as_bool();
    publish_side_support_seed_mask_ = get_parameter("publish_side_support_seed_mask").as_bool();
    publish_side_support_mask_ = get_parameter("publish_side_support_mask").as_bool();
    publish_width_supported_ridge_mask_ = get_parameter("publish_width_supported_ridge_mask").as_bool();
    publish_length_filtered_ridge_mask_ = get_parameter("publish_length_filtered_ridge_mask").as_bool();
    publish_reconstructed_mask_ = get_parameter("publish_reconstructed_mask").as_bool();
    publish_white_final_mask_ = get_parameter("publish_white_final_mask").as_bool();
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
    debug_jpeg_quality_ =
      std::clamp(static_cast<int>(get_parameter("debug_jpeg_quality").as_int()), 1, 100);
    debug_image_max_fps_ = std::max(0.0, get_parameter("debug_image_max_fps").as_double());
    enable_timing_debug_ = get_parameter("enable_timing_debug").as_bool();
    timing_summary_interval_ =
      std::max(1, static_cast<int>(get_parameter("timing_summary_interval").as_int()));
    display_max_width_ = std::max(1, static_cast<int>(get_parameter("display_max_width").as_int()));
    display_max_height_ =
      std::max(1, static_cast<int>(get_parameter("display_max_height").as_int()));

    if (cached_orientation_radius_ != orientation_window_radius_px_) {
      orientation_circle_offsets_ = buildOrientationCircleOffsets(orientation_window_radius_px_);
      cached_orientation_radius_ = orientation_window_radius_px_;
    }

    if (
      cached_template_direction_bins_ != side_template_direction_bins_ ||
      cached_template_side_band_depth_ != side_band_depth_px_)
    {
      side_template_cache_.clear();
      cached_template_direction_bins_ = side_template_direction_bins_;
      cached_template_side_band_depth_ = side_band_depth_px_;
    }
  }

  void resetTimingSummary()
  {
    timing_frames_in_interval_ = 0;
    timing_interval_start_frame_ = 0;
    timing_interval_total_us_ = 0;
    timing_interval_max_us_ = 0;
    timing_stage_interval_totals_.fill(0);
    stats_interval_totals_.fill(0.0);
  }

  void maybeLogTimingSummary(
    const RidgeFrameTiming & timing,
    const RidgeFrameStats & stats,
    unsigned long long current_frame_index)
  {
    if (!enable_timing_debug_) {
      return;
    }

    const long long callback_duration_us =
      timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::CallbackTotal)];

    if (timing_frames_in_interval_ == 0U) {
      timing_interval_start_frame_ = current_frame_index;
    }

    ++timing_frames_in_interval_;
    timing_interval_total_us_ += callback_duration_us;
    timing_interval_max_us_ = std::max(timing_interval_max_us_, callback_duration_us);
    for (std::size_t i = 0; i < timing_stage_interval_totals_.size(); ++i) {
      timing_stage_interval_totals_[i] += timing.stage_us[i];
    }
    const RidgeStatArray current_stats = ridgeStatsToArray(stats);
    for (std::size_t i = 0; i < stats_interval_totals_.size(); ++i) {
      stats_interval_totals_[i] += current_stats[i];
    }

    if (timing_frames_in_interval_ < static_cast<std::size_t>(timing_summary_interval_)) {
      return;
    }

    const double avg_ms =
      static_cast<double>(timing_interval_total_us_) /
      static_cast<double>(timing_frames_in_interval_) / 1000.0;

    std::ostringstream oss;
    oss << "\n================ DT Ridge Profiling 摘要 ================\n";
    oss << "帧区间: " << timing_interval_start_frame_ << " - " << current_frame_index
        << " (" << timing_frames_in_interval_ << " 帧)\n";
    oss << "当前帧总耗时: " << static_cast<double>(callback_duration_us) / 1000.0
        << " ms, 区间平均: " << avg_ms
        << " ms, 区间最大: " << static_cast<double>(timing_interval_max_us_) / 1000.0 << " ms\n\n";
    appendTimingTable(oss, timing, timing_stage_interval_totals_, timing_frames_in_interval_);
    appendStatsTable(oss, stats, stats_interval_totals_, timing_frames_in_interval_);
    oss << "\n宽度模式: fixed_floor_ceil (width_mad_scale/min_width_samples ignored)\n";
    oss << "========================================================";
    RCLCPP_INFO_STREAM(get_logger(), oss.str());

    resetTimingSummary();
  }

  bool anyImageWindowRequested() const
  {
    return show_morph_mask_ || show_distance_transform_ || show_green_mask_ ||
           show_black_mask_ || show_noise_mask_ ||
           show_ridge_mask_ || show_candidate_prefilter_mask_ ||
           (enable_orientation_estimate_ && show_orientation_valid_mask_) ||
           show_side_support_seed_mask_ || show_side_support_mask_ ||
           show_width_supported_ridge_mask_ || show_length_filtered_ridge_mask_ ||
           show_reconstructed_mask_ || show_white_final_mask_ || show_debug_image_;
  }

  bool anyImageWindowCreated() const
  {
    return morph_window_created_ || distance_transform_window_created_ || green_window_created_ ||
           black_window_created_ ||
           noise_window_created_ || ridge_window_created_ || candidate_prefilter_window_created_ ||
           orientation_valid_window_created_ || side_support_seed_window_created_ ||
           side_support_window_created_ || width_supported_window_created_ ||
           length_filtered_ridge_window_created_ || reconstructed_window_created_ ||
           white_final_mask_window_created_ || debug_window_created_;
  }

  void syncWindow(const std::string & window_name, bool should_show, bool & created)
  {
    if (should_show && !created) {
      cv::namedWindow(window_name, cv::WINDOW_NORMAL);
      created = true;
    } else if (!should_show && created) {
      cv::destroyWindow(window_name);
      created = false;
    }
  }

  void destroyDebugWindows()
  {
    syncWindow(kMorphWindow, false, morph_window_created_);
    syncWindow(kDistanceTransformWindow, false, distance_transform_window_created_);
    syncWindow(kGreenWindow, false, green_window_created_);
    syncWindow(kBlackWindow, false, black_window_created_);
    syncWindow(kNoiseWindow, false, noise_window_created_);
    syncWindow(kRidgeWindow, false, ridge_window_created_);
    syncWindow(kCandidatePrefilterWindow, false, candidate_prefilter_window_created_);
    syncWindow(kOrientationValidWindow, false, orientation_valid_window_created_);
    syncWindow(kSideSupportSeedWindow, false, side_support_seed_window_created_);
    syncWindow(kSideSupportWindow, false, side_support_window_created_);
    syncWindow(kWidthSupportedWindow, false, width_supported_window_created_);
    syncWindow(kLengthFilteredRidgeWindow, false, length_filtered_ridge_window_created_);
    syncWindow(kReconstructedWindow, false, reconstructed_window_created_);
    syncWindow(kWhiteFinalMaskWindow, false, white_final_mask_window_created_);
    syncWindow(kDebugWindow, false, debug_window_created_);
  }

  void syncImageViewState()
  {
    loadRuntimeParameters();
    const bool master_enabled = enable_image_view_ && anyImageWindowRequested();
    syncWindow(kMorphWindow, master_enabled && show_morph_mask_, morph_window_created_);
    syncWindow(
      kDistanceTransformWindow,
      master_enabled && show_distance_transform_,
      distance_transform_window_created_);
    syncWindow(kGreenWindow, master_enabled && show_green_mask_, green_window_created_);
    syncWindow(kBlackWindow, master_enabled && show_black_mask_, black_window_created_);
    syncWindow(kNoiseWindow, master_enabled && show_noise_mask_, noise_window_created_);
    syncWindow(kRidgeWindow, master_enabled && show_ridge_mask_, ridge_window_created_);
    syncWindow(
      kCandidatePrefilterWindow,
      master_enabled && show_candidate_prefilter_mask_,
      candidate_prefilter_window_created_);
    syncWindow(
      kOrientationValidWindow,
      master_enabled && enable_orientation_estimate_ && show_orientation_valid_mask_,
      orientation_valid_window_created_);
    syncWindow(
      kSideSupportSeedWindow,
      master_enabled && show_side_support_seed_mask_,
      side_support_seed_window_created_);
    syncWindow(
      kSideSupportWindow,
      master_enabled && show_side_support_mask_,
      side_support_window_created_);
    syncWindow(
      kWidthSupportedWindow,
      master_enabled && show_width_supported_ridge_mask_,
      width_supported_window_created_);
    syncWindow(
      kLengthFilteredRidgeWindow,
      master_enabled && show_length_filtered_ridge_mask_,
      length_filtered_ridge_window_created_);
    syncWindow(
      kReconstructedWindow,
      master_enabled && show_reconstructed_mask_,
      reconstructed_window_created_);
    syncWindow(
      kWhiteFinalMaskWindow,
      master_enabled && show_white_final_mask_,
      white_final_mask_window_created_);
    syncWindow(kDebugWindow, master_enabled && show_debug_image_, debug_window_created_);
  }

  void setupSubscribers()
  {
    morph_sub_.subscribe(this, morph_mask_topic_, rmw_qos_profile_sensor_data);
    green_sub_.subscribe(this, green_mask_topic_, rmw_qos_profile_sensor_data);
    black_sub_.subscribe(this, black_mask_topic_, rmw_qos_profile_sensor_data);
    noise_sub_.subscribe(this, noise_mask_topic_, rmw_qos_profile_sensor_data);

    synchronizer_ = std::make_shared<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(10),
      morph_sub_,
      green_sub_,
      black_sub_,
      noise_sub_);
    synchronizer_->registerCallback(std::bind(
        &WhiteLineDtRidgeFilterNode::maskCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4));
  }

  std::vector<SeedCandidate> selectSeedPoints(
    const cv::Mat & width_supported_ridge_mask,
    const cv::Mat & local_width_map) const
  {
    CV_Assert(width_supported_ridge_mask.type() == CV_8UC1);
    CV_Assert(local_width_map.type() == CV_32FC1);

    std::vector<SeedCandidate> seeds;
    if (cv::countNonZero(width_supported_ridge_mask) == 0) {
      return seeds;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    cv::connectedComponentsWithStats(width_supported_ridge_mask, labels, stats, centroids, 8, CV_32S);
    std::vector<int> component_counts(static_cast<std::size_t>(stats.rows), 0);
    seeds.reserve(static_cast<std::size_t>(cv::countNonZero(width_supported_ridge_mask)));
    for (int y = 0; y < width_supported_ridge_mask.rows; ++y) {
      for (int x = 0; x < width_supported_ridge_mask.cols; ++x) {
        if (width_supported_ridge_mask.at<uchar>(y, x) == 0) {
          continue;
        }
        const int label = labels.at<int>(y, x);
        if (label <= 0) {
          continue;
        }
        const int component_index = component_counts[static_cast<std::size_t>(label)]++;
        if ((component_index % side_scan_stride_) != 0) {
          continue;
        }
        const float local_width_px = local_width_map.at<float>(y, x);
        seeds.push_back(SeedCandidate{
            cv::Point(x, y),
            computeStartOffset(local_width_px, side_margin_px_),
            computeTangentHalfSpan(local_width_px)});
      }
    }
    return seeds;
  }

  void ensureSideTemplates(const std::vector<SeedPoint> & seeds)
  {
    for (const SeedPoint & seed : seeds) {
      const std::uint64_t left_key = makeSideTemplateKey(
        seed.dir_bin,
        seed.start_offset,
        seed.tangent_half_span,
        -1);
      if (side_template_cache_.find(left_key) == side_template_cache_.end()) {
        side_template_cache_.emplace(
          left_key,
          buildSideTemplateOffsets(
            seed.dir_bin,
            side_template_direction_bins_,
            seed.start_offset,
            seed.tangent_half_span,
            side_band_depth_px_,
            -1));
      }

      const std::uint64_t right_key = makeSideTemplateKey(
        seed.dir_bin,
        seed.start_offset,
        seed.tangent_half_span,
        1);
      if (side_template_cache_.find(right_key) == side_template_cache_.end()) {
        side_template_cache_.emplace(
          right_key,
          buildSideTemplateOffsets(
            seed.dir_bin,
            side_template_direction_bins_,
            seed.start_offset,
            seed.tangent_half_span,
            side_band_depth_px_,
            1));
      }
    }
  }

  template<typename PublisherT>
  bool shouldPublishDebugImage(const std::shared_ptr<PublisherT> & publisher, bool image_enabled) const
  {
    if (!publish_debug_images_ || !image_enabled || !hasSubscribers(publisher)) {
      return false;
    }
    return debugFpsGateAllows(publisher->get_topic_name(), SteadyClock::now());
  }

  bool debugFpsGateAllows(const std::string & key, const TimePoint & now) const
  {
    if (debug_image_max_fps_ <= 0.0 || !std::isfinite(debug_image_max_fps_)) {
      return true;
    }
    const auto it = debug_last_publish_times_.find(key);
    if (it == debug_last_publish_times_.end()) {
      return true;
    }
    const auto min_interval = std::chrono::duration<double>(1.0 / debug_image_max_fps_);
    return now - it->second >= min_interval;
  }

  bool consumeDebugFpsGate(const std::string & key, const TimePoint & now)
  {
    if (!debugFpsGateAllows(key, now)) {
      return false;
    }
    debug_last_publish_times_[key] = now;
    return true;
  }

  template<typename PublisherT>
  bool consumeDebugPublishPermit(
    const std::shared_ptr<PublisherT> & publisher,
    bool image_enabled,
    const TimePoint & now)
  {
    return publish_debug_images_ && image_enabled && hasSubscribers(publisher) &&
           consumeDebugFpsGate(publisher->get_topic_name(), now);
  }

  bool publishDebugImageIfNeeded(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    bool image_enabled,
    const std_msgs::msg::Header & header,
    const std::string & encoding,
    const cv::Mat & image,
    const TimePoint & now)
  {
    if (!consumeDebugPublishPermit(publisher, image_enabled, now)) {
      return false;
    }
    publisher->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
    return true;
  }

  bool publishCompressedDebugImageIfNeeded(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher,
    bool image_enabled,
    const std_msgs::msg::Header & header,
    const std::string & encoding,
    const cv::Mat & image,
    const TimePoint & now,
    long long & compressed_debug_us)
  {
    if (!consumeDebugPublishPermit(publisher, image_enabled, now)) {
      return false;
    }
    long long encode_us = 0;
    auto compressed_msg = rcj_loc::vision::debug::encodeJpegCompressedImage(
      header,
      encoding,
      image,
      debug_jpeg_quality_,
      &encode_us);
    compressed_debug_us += encode_us;
    if (!compressed_msg.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to JPEG-compress ridge debug image with encoding '%s'.",
        encoding.c_str());
      return false;
    }
    publisher->publish(*compressed_msg);
    return true;
  }

  bool publishOutputs(
    const std_msgs::msg::Header & header,
    const cv::Mat & morph_mask,
    const cv::Mat & distance_transform_debug_image,
    const cv::Mat & green_mask,
    const cv::Mat & black_mask,
    const cv::Mat & noise_mask,
    const cv::Mat & ridge_mask,
    const cv::Mat & candidate_prefilter_mask,
    const cv::Mat & orientation_valid_mask,
    const cv::Mat & side_support_seed_mask,
    const cv::Mat & side_support_mask,
    const cv::Mat & width_supported_ridge_mask,
    const cv::Mat & length_filtered_ridge_mask,
    const cv::Mat & reconstructed_mask,
    const cv::Mat & white_final_mask,
    const cv::Mat & debug_image,
    long long & compressed_debug_us)
  {
    const TimePoint now = SteadyClock::now();
    bool published_any = false;

    published_any |=
      publishImageIfSubscribed(white_final_mask_pub_, header, "mono8", white_final_mask);
    published_any |= publishCompressedDebugImageIfNeeded(
      white_final_mask_compressed_pub_,
      publish_white_final_mask_,
      header,
      "mono8",
      white_final_mask,
      now,
      compressed_debug_us);

    if (hasSubscribers(legacy_white_mask_pub_)) {
      if (!warned_deprecated_white_mask_topic_) {
        RCLCPP_WARN(
          get_logger(),
          "Topic '~/white_mask' is deprecated. Use '~/white_final_mask' instead.");
        warned_deprecated_white_mask_topic_ = true;
      }
      legacy_white_mask_pub_->publish(
        *cv_bridge::CvImage(header, "mono8", white_final_mask).toImageMsg());
      published_any = true;
    }
    published_any |= publishCompressedDebugImageIfNeeded(
      legacy_white_mask_compressed_pub_,
      publish_white_final_mask_,
      header,
      "mono8",
      white_final_mask,
      now,
      compressed_debug_us);

    published_any |= publishDebugImageIfNeeded(
      debug_morph_mask_pub_, publish_morph_mask_, header, "mono8", morph_mask, now);
    published_any |= publishCompressedDebugImageIfNeeded(
      debug_morph_mask_compressed_pub_, publish_morph_mask_, header, "mono8", morph_mask, now, compressed_debug_us);
    published_any |= publishDebugImageIfNeeded(
      debug_green_mask_pub_, publish_green_mask_, header, "mono8", green_mask, now);
    published_any |= publishCompressedDebugImageIfNeeded(
      debug_green_mask_compressed_pub_, publish_green_mask_, header, "mono8", green_mask, now, compressed_debug_us);
    published_any |= publishDebugImageIfNeeded(
      debug_black_mask_pub_, publish_black_mask_, header, "mono8", black_mask, now);
    published_any |= publishCompressedDebugImageIfNeeded(
      debug_black_mask_compressed_pub_, publish_black_mask_, header, "mono8", black_mask, now, compressed_debug_us);
    published_any |= publishDebugImageIfNeeded(
      debug_noise_mask_pub_, publish_noise_mask_, header, "mono8", noise_mask, now);
    published_any |= publishCompressedDebugImageIfNeeded(
      debug_noise_mask_compressed_pub_, publish_noise_mask_, header, "mono8", noise_mask, now, compressed_debug_us);

    if (publish_debug_images_ && publish_ridge_mask_) {
      published_any |= publishDebugImageIfNeeded(
        ridge_mask_pub_, publish_ridge_mask_, header, "mono8", ridge_mask, now);
      published_any |= publishDebugImageIfNeeded(
        legacy_skeleton_mask_pub_,
        publish_ridge_mask_,
        header,
        "mono8",
        ridge_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        ridge_mask_compressed_pub_, publish_ridge_mask_, header, "mono8", ridge_mask, now, compressed_debug_us);
      published_any |= publishCompressedDebugImageIfNeeded(
        legacy_skeleton_mask_compressed_pub_, publish_ridge_mask_, header, "mono8", ridge_mask, now, compressed_debug_us);
    }
    if (publish_debug_images_ && publish_candidate_prefilter_mask_) {
      published_any |= publishDebugImageIfNeeded(
        candidate_prefilter_mask_pub_,
        publish_candidate_prefilter_mask_,
        header,
        "mono8",
        candidate_prefilter_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        candidate_prefilter_mask_compressed_pub_,
        publish_candidate_prefilter_mask_,
        header,
        "mono8",
        candidate_prefilter_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_distance_transform_ && !distance_transform_debug_image.empty()) {
      published_any |= publishDebugImageIfNeeded(
        distance_transform_pub_,
        publish_distance_transform_,
        header,
        "bgr8",
        distance_transform_debug_image,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        distance_transform_compressed_pub_,
        publish_distance_transform_,
        header,
        "bgr8",
        distance_transform_debug_image,
        now,
        compressed_debug_us);
    }
    if (enable_orientation_estimate_ && publish_debug_images_ && publish_orientation_valid_mask_) {
      published_any |= publishDebugImageIfNeeded(
        orientation_valid_mask_pub_,
        publish_orientation_valid_mask_,
        header,
        "mono8",
        orientation_valid_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        orientation_valid_mask_compressed_pub_,
        publish_orientation_valid_mask_,
        header,
        "mono8",
        orientation_valid_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_side_support_seed_mask_) {
      published_any |= publishDebugImageIfNeeded(
        side_support_seed_mask_pub_,
        publish_side_support_seed_mask_,
        header,
        "mono8",
        side_support_seed_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        side_support_seed_mask_compressed_pub_,
        publish_side_support_seed_mask_,
        header,
        "mono8",
        side_support_seed_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_side_support_mask_) {
      published_any |= publishDebugImageIfNeeded(
        side_support_mask_pub_,
        publish_side_support_mask_,
        header,
        "mono8",
        side_support_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        side_support_mask_compressed_pub_,
        publish_side_support_mask_,
        header,
        "mono8",
        side_support_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_width_supported_ridge_mask_) {
      published_any |= publishDebugImageIfNeeded(
        width_supported_ridge_mask_pub_,
        publish_width_supported_ridge_mask_,
        header,
        "mono8",
        width_supported_ridge_mask,
        now);
      published_any |= publishDebugImageIfNeeded(
        legacy_width_supported_skeleton_mask_pub_,
        publish_width_supported_ridge_mask_,
        header,
        "mono8",
        width_supported_ridge_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        width_supported_ridge_mask_compressed_pub_,
        publish_width_supported_ridge_mask_,
        header,
        "mono8",
        width_supported_ridge_mask,
        now,
        compressed_debug_us);
      published_any |= publishCompressedDebugImageIfNeeded(
        legacy_width_supported_skeleton_mask_compressed_pub_,
        publish_width_supported_ridge_mask_,
        header,
        "mono8",
        width_supported_ridge_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_length_filtered_ridge_mask_) {
      published_any |= publishDebugImageIfNeeded(
        length_filtered_ridge_mask_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now);
      published_any |= publishDebugImageIfNeeded(
        legacy_length_filtered_skeleton_mask_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now);
      published_any |= publishDebugImageIfNeeded(
        legacy_supported_skeleton_mask_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        length_filtered_ridge_mask_compressed_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now,
        compressed_debug_us);
      published_any |= publishCompressedDebugImageIfNeeded(
        legacy_length_filtered_skeleton_mask_compressed_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now,
        compressed_debug_us);
      published_any |= publishCompressedDebugImageIfNeeded(
        legacy_supported_skeleton_mask_compressed_pub_,
        publish_length_filtered_ridge_mask_,
        header,
        "mono8",
        length_filtered_ridge_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_reconstructed_mask_) {
      published_any |= publishDebugImageIfNeeded(
        reconstructed_mask_pub_,
        publish_reconstructed_mask_,
        header,
        "mono8",
        reconstructed_mask,
        now);
      published_any |= publishCompressedDebugImageIfNeeded(
        reconstructed_mask_compressed_pub_,
        publish_reconstructed_mask_,
        header,
        "mono8",
        reconstructed_mask,
        now,
        compressed_debug_us);
    }
    if (publish_debug_images_ && publish_debug_image_ && !debug_image.empty()) {
      published_any |= publishDebugImageIfNeeded(
        debug_pub_, publish_debug_image_, header, "bgr8", debug_image, now);
      published_any |= publishCompressedDebugImageIfNeeded(
        debug_compressed_pub_, publish_debug_image_, header, "bgr8", debug_image, now, compressed_debug_us);
    }

    return published_any;
  }

  bool displayOutputs(
    const cv::Mat & morph_mask,
    const cv::Mat & distance_transform_debug_image,
    const cv::Mat & ridge_mask,
    const cv::Mat & candidate_prefilter_mask,
    const cv::Mat & orientation_valid_mask,
    const cv::Mat & side_support_seed_mask,
    const cv::Mat & side_support_mask,
    const cv::Mat & width_supported_ridge_mask,
    const cv::Mat & length_filtered_ridge_mask,
    const cv::Mat & reconstructed_mask,
    const cv::Mat & white_final_mask,
    const cv::Mat & green_mask,
    const cv::Mat & black_mask,
    const cv::Mat & noise_mask,
    const cv::Mat & debug_image)
  {
    bool displayed_any_window = false;
    if (morph_window_created_) {
      cv::imshow(kMorphWindow, morph_mask);
      resizeWindowToFitImage(kMorphWindow, morph_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (distance_transform_window_created_ && !distance_transform_debug_image.empty()) {
      cv::imshow(kDistanceTransformWindow, distance_transform_debug_image);
      resizeWindowToFitImage(
        kDistanceTransformWindow,
        distance_transform_debug_image,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (green_window_created_) {
      cv::imshow(kGreenWindow, green_mask);
      resizeWindowToFitImage(kGreenWindow, green_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (black_window_created_) {
      cv::imshow(kBlackWindow, black_mask);
      resizeWindowToFitImage(kBlackWindow, black_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (noise_window_created_) {
      cv::imshow(kNoiseWindow, noise_mask);
      resizeWindowToFitImage(kNoiseWindow, noise_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (ridge_window_created_) {
      cv::imshow(kRidgeWindow, ridge_mask);
      resizeWindowToFitImage(kRidgeWindow, ridge_mask, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (candidate_prefilter_window_created_) {
      cv::imshow(kCandidatePrefilterWindow, candidate_prefilter_mask);
      resizeWindowToFitImage(
        kCandidatePrefilterWindow,
        candidate_prefilter_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (enable_orientation_estimate_ && orientation_valid_window_created_) {
      cv::imshow(kOrientationValidWindow, orientation_valid_mask);
      resizeWindowToFitImage(
        kOrientationValidWindow,
        orientation_valid_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (side_support_seed_window_created_) {
      cv::imshow(kSideSupportSeedWindow, side_support_seed_mask);
      resizeWindowToFitImage(
        kSideSupportSeedWindow,
        side_support_seed_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (side_support_window_created_) {
      cv::imshow(kSideSupportWindow, side_support_mask);
      resizeWindowToFitImage(
        kSideSupportWindow,
        side_support_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (width_supported_window_created_) {
      cv::imshow(kWidthSupportedWindow, width_supported_ridge_mask);
      resizeWindowToFitImage(
        kWidthSupportedWindow,
        width_supported_ridge_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (length_filtered_ridge_window_created_) {
      cv::imshow(kLengthFilteredRidgeWindow, length_filtered_ridge_mask);
      resizeWindowToFitImage(
        kLengthFilteredRidgeWindow,
        length_filtered_ridge_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (reconstructed_window_created_) {
      cv::imshow(kReconstructedWindow, reconstructed_mask);
      resizeWindowToFitImage(
        kReconstructedWindow,
        reconstructed_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (white_final_mask_window_created_) {
      cv::imshow(kWhiteFinalMaskWindow, white_final_mask);
      resizeWindowToFitImage(
        kWhiteFinalMaskWindow,
        white_final_mask,
        display_max_width_,
        display_max_height_);
      displayed_any_window = true;
    }
    if (debug_window_created_ && !debug_image.empty()) {
      cv::imshow(kDebugWindow, debug_image);
      resizeWindowToFitImage(kDebugWindow, debug_image, display_max_width_, display_max_height_);
      displayed_any_window = true;
    }
    if (displayed_any_window) {
      cv::waitKey(1);
    }
    return displayed_any_window;
  }

  void maskCallback(
    const ImageMsg::ConstSharedPtr & morph_msg,
    const ImageMsg::ConstSharedPtr & green_msg,
    const ImageMsg::ConstSharedPtr & black_msg,
    const ImageMsg::ConstSharedPtr & noise_msg)
  {
    const bool previous_timing_debug = enable_timing_debug_;
    const int previous_timing_summary_interval = timing_summary_interval_;
    syncImageViewState();
    const bool timing_enabled = enable_timing_debug_;
    RidgeFrameTiming timing;
    const auto callback_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    auto stage_start = timing_enabled ? callback_start : TimePoint{};
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::RuntimeSync,
        stage_start,
        SteadyClock::now());
    }
    if (
      previous_timing_debug != enable_timing_debug_ ||
      previous_timing_summary_interval != timing_summary_interval_)
    {
      resetTimingSummary();
    }

    cv::Mat morph_mask;
    cv::Mat green_mask;
    cv::Mat black_mask;
    cv::Mat noise_mask;
    try {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      morph_mask = cv_bridge::toCvCopy(morph_msg, "mono8")->image;
      green_mask = cv_bridge::toCvCopy(green_msg, "mono8")->image;
      black_mask = cv_bridge::toCvCopy(black_msg, "mono8")->image;
      noise_mask = cv_bridge::toCvCopy(noise_msg, "mono8")->image;
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::CvBridgeConvert,
          stage_start,
          SteadyClock::now());
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge failed: %s", e.what());
      return;
    }

    if (
      morph_mask.size() != green_mask.size() || morph_mask.size() != black_mask.size() ||
      morph_mask.size() != noise_mask.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "DT ridge filter received mismatched mask sizes.");
      return;
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    cv::Mat distance_transform;
    cv::distanceTransform(morph_mask, distance_transform, cv::DIST_L2, 3);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::DistanceTransform,
        stage_start,
        SteadyClock::now());
    }

    const bool distance_transform_debug_image_needed =
      (show_distance_transform_ && distance_transform_window_created_) ||
      shouldPublishDebugImage(distance_transform_pub_, publish_distance_transform_) ||
      shouldPublishDebugImage(distance_transform_compressed_pub_, publish_distance_transform_);
    cv::Mat distance_transform_debug_image;
    if (distance_transform_debug_image_needed) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      distance_transform_debug_image = createDistanceTransformDebugImage(distance_transform);
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::DistanceTransformDebug,
          stage_start,
          SteadyClock::now());
      }
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    const cv::Mat ridge_mask = extractDistanceTransformRidgeMask(morph_mask, distance_transform);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::RidgeExtract,
        stage_start,
        SteadyClock::now());
    }

    RidgeFrameStats frame_stats;
    cv::Mat candidate_prefilter_mask;
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    if (enable_candidate_prefilter_) {
      candidate_prefilter_mask =
        filterMaskByMinComponentArea(ridge_mask, candidate_min_component_px_);
      candidate_prefilter_mask = pruneMaskEndpoints(candidate_prefilter_mask, candidate_prune_rounds_);
    } else {
      candidate_prefilter_mask = ridge_mask.clone();
    }
    if (timing_enabled) {
      if (enable_candidate_prefilter_) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::CandidatePrefilter,
          stage_start,
          SteadyClock::now());
      } else {
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::CandidatePrefilter)] = 0;
      }
    }
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    frame_stats.ridge_point_count = static_cast<double>(cv::countNonZero(ridge_mask));
    frame_stats.prefiltered_ridge_point_count =
      static_cast<double>(cv::countNonZero(candidate_prefilter_mask));
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::StatsCollect,
        stage_start,
        SteadyClock::now());
    }

    cv::Mat orientation_valid_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
    cv::Mat side_support_seed_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
    cv::Mat side_support_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
    cv::Mat local_width_map = cv::Mat::zeros(morph_mask.size(), CV_32FC1);
    cv::Mat width_supported_ridge_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
    const cv::Mat & ridge_processing_mask = enable_candidate_prefilter_ ? candidate_prefilter_mask :
      ridge_mask;
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    std::vector<cv::Point> ridge_points;
    cv::findNonZero(ridge_processing_mask, ridge_points);
    for (const cv::Point & point : ridge_points) {
      const float local_width_px = 2.0F * distance_transform.at<float>(point.y, point.x);
      if (local_width_px <= 0.0F) {
        continue;
      }
      local_width_map.at<float>(point.y, point.x) = local_width_px;
      if (local_width_px < width_floor_px_ || local_width_px > width_ceil_px_) {
        continue;
      }
      width_supported_ridge_mask.at<uchar>(point.y, point.x) = 255;
      frame_stats.width_supported_point_count += 1.0;
      frame_stats.max_local_width_px = std::max(
        frame_stats.max_local_width_px,
        static_cast<double>(local_width_px));
    }
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::WidthFilterPreSupport,
        stage_start,
        SteadyClock::now());
    }

    std::vector<SeedPoint> oriented_seed_points;
    if (enable_orientation_estimate_) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      const std::vector<SeedCandidate> seed_candidates =
        selectSeedPoints(width_supported_ridge_mask, local_width_map);
      frame_stats.seed_point_count = static_cast<double>(seed_candidates.size());
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::SideSupportSeedSelect,
          stage_start,
          SteadyClock::now());
      }

      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      oriented_seed_points.reserve(seed_candidates.size());
      if (enable_parallel_orientation_estimate_ && !seed_candidates.empty()) {
#ifdef _OPENMP
#pragma omp parallel
        {
          std::vector<SeedPoint> local_oriented;
          local_oriented.reserve(64);
#pragma omp for schedule(static) nowait
          for (int i = 0; i < static_cast<int>(seed_candidates.size()); ++i) {
            const SeedCandidate & seed_candidate = seed_candidates[static_cast<std::size_t>(i)];
            cv::Point2f tangent;
            cv::Point2f normal;
            if (!estimateOrientation(
                  width_supported_ridge_mask,
                  seed_candidate.point,
                  orientation_circle_offsets_,
                  min_orientation_neighbors_,
                  tangent,
                  normal))
            {
              continue;
            }
            local_oriented.push_back(SeedPoint{
                seed_candidate.point,
                quantizeDirectionBin(tangent, side_template_direction_bins_),
                seed_candidate.start_offset,
                seed_candidate.tangent_half_span});
          }
#pragma omp critical
          {
            oriented_seed_points.insert(
              oriented_seed_points.end(),
              local_oriented.begin(),
              local_oriented.end());
          }
        }
#else
        for (const SeedCandidate & seed_candidate : seed_candidates) {
          cv::Point2f tangent;
          cv::Point2f normal;
          if (!estimateOrientation(
                width_supported_ridge_mask,
                seed_candidate.point,
                orientation_circle_offsets_,
                min_orientation_neighbors_,
                tangent,
                normal))
          {
            continue;
          }
          oriented_seed_points.push_back(SeedPoint{
              seed_candidate.point,
              quantizeDirectionBin(tangent, side_template_direction_bins_),
              seed_candidate.start_offset,
              seed_candidate.tangent_half_span});
        }
#endif
      } else {
        for (const SeedCandidate & seed_candidate : seed_candidates) {
          cv::Point2f tangent;
          cv::Point2f normal;
          if (!estimateOrientation(
                width_supported_ridge_mask,
                seed_candidate.point,
                orientation_circle_offsets_,
                min_orientation_neighbors_,
                tangent,
                normal))
          {
            continue;
          }
          oriented_seed_points.push_back(SeedPoint{
              seed_candidate.point,
              quantizeDirectionBin(tangent, side_template_direction_bins_),
              seed_candidate.start_offset,
              seed_candidate.tangent_half_span});
        }
      }
      for (const SeedPoint & seed : oriented_seed_points) {
        orientation_valid_mask.at<uchar>(seed.point.y, seed.point.x) = 255;
      }
      frame_stats.orientation_valid_seed_count = static_cast<double>(oriented_seed_points.size());
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::OrientationEstimate,
          stage_start,
          SteadyClock::now());
      }

      cv::Mat label_map;
      if (!oriented_seed_points.empty()) {
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        label_map = buildLabelMap(green_mask, black_mask, noise_mask);
        if (timing_enabled) {
          recordStageDuration(
            timing.stage_us,
            RidgeTimingStage::LabelMapBuild,
            stage_start,
            SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        ensureSideTemplates(oriented_seed_points);
        if (timing_enabled) {
          recordStageDuration(
            timing.stage_us,
            RidgeTimingStage::SideTemplatePrepare,
            stage_start,
            SteadyClock::now());
        }
      } else if (timing_enabled) {
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::LabelMapBuild)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::SideTemplatePrepare)] = 0;
      }

      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      std::vector<cv::Point> supported_seed_points;
      supported_seed_points.reserve(oriented_seed_points.size());
      long long total_side_samples = 0;
      auto scan_seed_range = [&](int begin, int end, std::vector<cv::Point> & local_supported,
          long long & local_total_samples) {
          for (int i = begin; i < end; ++i) {
            const SeedPoint & seed = oriented_seed_points[static_cast<std::size_t>(i)];
            const auto & left_offsets = side_template_cache_.at(makeSideTemplateKey(
                seed.dir_bin,
                seed.start_offset,
                seed.tangent_half_span,
                -1));
            const auto & right_offsets = side_template_cache_.at(makeSideTemplateKey(
                seed.dir_bin,
                seed.start_offset,
                seed.tangent_half_span,
                1));

            const SideSampleStats left_stats =
              sampleSideSupportFromTemplate(seed.point, label_map, left_offsets);
            const SideSampleStats right_stats =
              sampleSideSupportFromTemplate(seed.point, label_map, right_offsets);
            local_total_samples += left_stats.total_samples + right_stats.total_samples;

            const bool interior_supported =
              left_stats.greenRatio() >= min_green_ratio_ &&
              right_stats.greenRatio() >= min_green_ratio_;
            const bool boundary_supported =
              enable_boundary_mode_ &&
              ((left_stats.greenRatio() >= min_green_ratio_ &&
              right_stats.boundaryRatio() >= min_boundary_ratio_) ||
              (right_stats.greenRatio() >= min_green_ratio_ &&
              left_stats.boundaryRatio() >= min_boundary_ratio_));
            if (interior_supported || boundary_supported) {
              local_supported.push_back(seed.point);
            }
          }
        };

      if (enable_parallel_side_scan_ && !oriented_seed_points.empty()) {
#ifdef _OPENMP
#pragma omp parallel
        {
          std::vector<cv::Point> local_supported;
          local_supported.reserve(64);
          long long local_total_samples = 0;
#pragma omp for schedule(static) nowait
          for (int i = 0; i < static_cast<int>(oriented_seed_points.size()); ++i) {
            scan_seed_range(i, i + 1, local_supported, local_total_samples);
          }
#pragma omp critical
          {
            total_side_samples += local_total_samples;
            supported_seed_points.insert(
              supported_seed_points.end(),
              local_supported.begin(),
              local_supported.end());
          }
        }
#else
        scan_seed_range(
          0,
          static_cast<int>(oriented_seed_points.size()),
          supported_seed_points,
          total_side_samples);
#endif
      } else {
        scan_seed_range(
          0,
          static_cast<int>(oriented_seed_points.size()),
          supported_seed_points,
          total_side_samples);
      }
      for (const cv::Point & point : supported_seed_points) {
        side_support_seed_mask.at<uchar>(point.y, point.x) = 255;
      }
      frame_stats.supported_seed_point_count = static_cast<double>(supported_seed_points.size());
      frame_stats.total_side_samples = static_cast<double>(total_side_samples);
      frame_stats.avg_samples_per_seed =
        oriented_seed_points.empty() ? 0.0 : static_cast<double>(total_side_samples) /
        static_cast<double>(oriented_seed_points.size());
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::SideSupportSeedScan,
          stage_start,
          SteadyClock::now());
      }

      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      if (!supported_seed_points.empty()) {
        cv::Mat propagated_seed_mask;
        const cv::Mat kernel =
          cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(side_support_seed_mask, propagated_seed_mask, kernel);
        cv::bitwise_and(propagated_seed_mask, width_supported_ridge_mask, side_support_mask);
      }
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::SideSupportPropagate,
          stage_start,
          SteadyClock::now());
      }
    } else {
      side_support_mask = width_supported_ridge_mask.clone();
      if (timing_enabled) {
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::OrientationEstimate)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::LabelMapBuild)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::SideSupportSeedSelect)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::SideTemplatePrepare)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::SideSupportSeedScan)] = 0;
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::SideSupportPropagate)] = 0;
      }
    }

    cv::Mat length_filtered_ridge_mask;
    if (enable_length_filter_) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      length_filtered_ridge_mask =
        filterMaskByLength(side_support_mask, min_skeleton_length_px_);
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::LengthFilter,
          stage_start,
          SteadyClock::now());
      }
    } else {
      length_filtered_ridge_mask = side_support_mask.clone();
      if (timing_enabled) {
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::LengthFilter)] = 0;
      }
    }

    cv::Mat reconstructed_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    for (int y = 0; y < length_filtered_ridge_mask.rows; ++y) {
      for (int x = 0; x < length_filtered_ridge_mask.cols; ++x) {
        if (length_filtered_ridge_mask.at<uchar>(y, x) == 0) {
          continue;
        }
        const float local_width_px = local_width_map.at<float>(y, x);
        const int radius = std::max(
          1,
          static_cast<int>(std::round(0.5 * static_cast<double>(local_width_px) + reconstruction_margin_px_)));
        cv::circle(
          reconstructed_mask,
          cv::Point(x, y),
          radius,
          cv::Scalar(255),
          cv::FILLED);
      }
    }
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::Reconstruct,
        stage_start,
        SteadyClock::now());
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    cv::Mat white_final_mask;
    cv::bitwise_and(reconstructed_mask, morph_mask, white_final_mask);
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::FinalMask,
        stage_start,
        SteadyClock::now());
    }
    const unsigned long long current_frame_index = ++frame_count_;

    const bool debug_image_needed =
      (show_debug_image_ && debug_window_created_) ||
      shouldPublishDebugImage(debug_pub_, publish_debug_image_) ||
      shouldPublishDebugImage(debug_compressed_pub_, publish_debug_image_);
    cv::Mat debug_image;
    if (debug_image_needed) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      debug_image = createDebugComposite(green_mask, black_mask, white_final_mask);
      if (timing_enabled) {
        recordStageDuration(
          timing.stage_us,
          RidgeTimingStage::DebugComposite,
          stage_start,
          SteadyClock::now());
      }
    }

    stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
    long long compressed_debug_us = 0;
    const bool published_any = publishOutputs(
      morph_msg->header,
      morph_mask,
      distance_transform_debug_image,
      green_mask,
      black_mask,
      noise_mask,
      ridge_mask,
      candidate_prefilter_mask,
      orientation_valid_mask,
      side_support_seed_mask,
      side_support_mask,
      width_supported_ridge_mask,
      length_filtered_ridge_mask,
      reconstructed_mask,
      white_final_mask,
      debug_image,
      compressed_debug_us);
    if (timing_enabled) {
      const long long publish_total_us = elapsedUs(stage_start, SteadyClock::now());
      timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::DebugCompression)] =
        compressed_debug_us;
      timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::PublishOutputs)] =
        std::max(0LL, publish_total_us - compressed_debug_us);
    }

    const bool display_requested = anyImageWindowCreated();
    bool displayed_any = false;
    if (display_requested) {
      stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
      displayed_any = displayOutputs(
        morph_mask,
        distance_transform_debug_image,
        ridge_mask,
        candidate_prefilter_mask,
        orientation_valid_mask,
        side_support_seed_mask,
        side_support_mask,
        width_supported_ridge_mask,
        length_filtered_ridge_mask,
        reconstructed_mask,
        white_final_mask,
        green_mask,
        black_mask,
        noise_mask,
        debug_image);
    }
    if (timing_enabled && displayed_any) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::GuiDisplay,
        stage_start,
        SteadyClock::now());
    }
    if (timing_enabled && !published_any) {
      timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::PublishOutputs)] = 0;
    }
    if (timing_enabled) {
      recordStageDuration(
        timing.stage_us,
        RidgeTimingStage::CallbackTotal,
        callback_start,
        SteadyClock::now());
      const long long accounted_stage_us = sumTimingStagesExcludingCallback(timing.stage_us);
      timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::UnaccountedOverhead)] =
        std::max(
        0LL,
        timing.stage_us[static_cast<std::size_t>(RidgeTimingStage::CallbackTotal)] -
        accounted_stage_us);
    }

    maybeLogTimingSummary(timing, frame_stats, current_frame_index);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhiteLineDtRidgeFilterNode>());
  rclcpp::shutdown();
  return 0;
}
