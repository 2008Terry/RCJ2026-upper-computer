#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
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

constexpr char kMorphWindow[] = "Skeleton Filter Input Morph Mask";
constexpr char kGreenWindow[] = "Skeleton Filter Input Green Mask";
constexpr char kBlackWindow[] = "Skeleton Filter Input Black Mask";
constexpr char kNoiseWindow[] = "Skeleton Filter Input Noise Mask";
constexpr char kSkeletonWindow[] = "Skeleton Filter Skeletonization";
constexpr char kOrientationValidWindow[] = "Skeleton Filter Orientation Valid";
constexpr char kSideSupportWindow[] = "Skeleton Filter Side Color Filter";
constexpr char kWidthSupportedWindow[] = "Skeleton Filter Width Filter";
constexpr char kLengthFilteredSkeletonWindow[] = "Skeleton Filter Length-Filtered Skeleton";
constexpr char kReconstructedWindow[] = "Skeleton Filter Reconstruction";
constexpr char kWhiteFinalMaskWindow[] = "Skeleton Filter White Final Mask";
constexpr char kDebugWindow[] = "Skeleton Filter Composite Debug";
    constexpr bool kDefaultShowLengthFilteredSkeletonMask = true;

cv::Size fitWithinBounds(const cv::Size &image_size, int max_width, int max_height) {
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
    const std::string &window_name,
    const cv::Mat &image,
    int max_width,
    int max_height) {
    const cv::Size fitted_size = fitWithinBounds(image.size(), max_width, max_height);
    cv::resizeWindow(window_name, fitted_size.width, fitted_size.height);
}

void thinningIteration(cv::Mat &binary, int iteration) {
    cv::Mat marker = cv::Mat::zeros(binary.size(), CV_8UC1);

    for (int y = 1; y < binary.rows - 1; ++y) {
        for (int x = 1; x < binary.cols - 1; ++x) {
            const uchar p1 = binary.at<uchar>(y, x);
            if (p1 != 1) {
                continue;
            }

            const uchar p2 = binary.at<uchar>(y - 1, x);
            const uchar p3 = binary.at<uchar>(y - 1, x + 1);
            const uchar p4 = binary.at<uchar>(y, x + 1);
            const uchar p5 = binary.at<uchar>(y + 1, x + 1);
            const uchar p6 = binary.at<uchar>(y + 1, x);
            const uchar p7 = binary.at<uchar>(y + 1, x - 1);
            const uchar p8 = binary.at<uchar>(y, x - 1);
            const uchar p9 = binary.at<uchar>(y - 1, x - 1);

            const int neighbor_count = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if (neighbor_count < 2 || neighbor_count > 6) {
                continue;
            }

            const int transition_count =
                (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + (p4 == 0 && p5 == 1) +
                (p5 == 0 && p6 == 1) + (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            if (transition_count != 1) {
                continue;
            }

            if (iteration == 0) {
                if (p2 * p4 * p6 != 0 || p4 * p6 * p8 != 0) {
                    continue;
                }
            } else {
                if (p2 * p4 * p8 != 0 || p2 * p6 * p8 != 0) {
                    continue;
                }
            }

            marker.at<uchar>(y, x) = 1;
        }
    }

    binary &= ~marker;
}

cv::Mat skeletonize(const cv::Mat &binary_mask) {
    CV_Assert(binary_mask.type() == CV_8UC1);

    cv::Mat skeleton;
    cv::threshold(binary_mask, skeleton, 0, 1, cv::THRESH_BINARY);

    cv::Mat previous = cv::Mat::zeros(skeleton.size(), CV_8UC1);
    cv::Mat diff;
    do {
        thinningIteration(skeleton, 0);
        thinningIteration(skeleton, 1);
        cv::absdiff(skeleton, previous, diff);
        skeleton.copyTo(previous);
    } while (cv::countNonZero(diff) > 0);

    skeleton *= 255;
    return skeleton;
}

struct SideSampleStats {
    int total_samples = 0;
    int green_samples = 0;
    int black_samples = 0;
    int noise_samples = 0;
    int outside_samples = 0;

    double greenRatio() const {
        return total_samples > 0 ? static_cast<double>(green_samples) / total_samples : 0.0;
    }

    double boundaryRatio() const {
        return total_samples > 0
                   ? static_cast<double>(black_samples + noise_samples + outside_samples) /
                         total_samples
                   : 0.0;
    }
};

double computeMedian(std::vector<float> values) {
    if (values.empty()) {
        return 0.0;
    }

    const auto mid_it = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid_it, values.end());
    const double upper = *mid_it;
    if (values.size() % 2 == 1) {
        return upper;
    }

    const auto lower_it = std::max_element(values.begin(), mid_it);
    return 0.5 * (upper + *lower_it);
}

bool estimateOrientation(
    const cv::Mat &skeleton_mask,
    const cv::Point &center,
    int radius,
    int min_neighbors,
    cv::Point2f &tangent,
    cv::Point2f &normal) {
    const int clamped_radius = std::max(1, radius);
    const int x_min = std::max(0, center.x - clamped_radius);
    const int x_max = std::min(skeleton_mask.cols - 1, center.x + clamped_radius);
    const int y_min = std::max(0, center.y - clamped_radius);
    const int y_max = std::min(skeleton_mask.rows - 1, center.y + clamped_radius);
    const int radius_sq = clamped_radius * clamped_radius;

    std::vector<cv::Point2f> neighbors;
    neighbors.reserve((2 * clamped_radius + 1) * (2 * clamped_radius + 1));
    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            if (skeleton_mask.at<uchar>(y, x) == 0) {
                continue;
            }
            const int dx = x - center.x;
            const int dy = y - center.y;
            if (dx * dx + dy * dy > radius_sq) {
                continue;
            }
            neighbors.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }

    if (static_cast<int>(neighbors.size()) < std::max(2, min_neighbors)) {
        return false;
    }

    cv::Point2f mean(0.0f, 0.0f);
    for (const auto &neighbor : neighbors) {
        mean += neighbor;
    }
    mean.x /= static_cast<float>(neighbors.size());
    mean.y /= static_cast<float>(neighbors.size());

    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const auto &neighbor : neighbors) {
        const double dx = neighbor.x - mean.x;
        const double dy = neighbor.y - mean.y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }

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
        eigenvector = cv::Point2f(1.0f, 0.0f);
    } else {
        eigenvector = cv::Point2f(0.0f, 1.0f);
    }

    const float norm = std::sqrt(eigenvector.x * eigenvector.x + eigenvector.y * eigenvector.y);
    if (norm < 1e-6f) {
        return false;
    }

    tangent = cv::Point2f(eigenvector.x / norm, eigenvector.y / norm);
    normal = cv::Point2f(-tangent.y, tangent.x);
    return true;
}

SideSampleStats sampleSideSupport(
    const cv::Point &center,
    const cv::Point2f &tangent,
    const cv::Point2f &normal,
    float local_width_px,
    int side_sign,
    const cv::Mat &green_mask,
    const cv::Mat &black_mask,
    const cv::Mat &noise_mask,
    int side_margin_px,
    int side_band_depth_px) {
    SideSampleStats stats;

    const float half_width = 0.5f * std::max(local_width_px, 0.0f);
    const int start_offset = std::max(1, static_cast<int>(std::round(half_width + side_margin_px)));
    const int depth = std::max(1, side_band_depth_px);
    const int tangent_half_span =
        std::max(1, static_cast<int>(std::round(std::max(1.0f, half_width))));

    const cv::Point2f center_f(static_cast<float>(center.x), static_cast<float>(center.y));
    for (int tangential_step = -tangent_half_span; tangential_step <= tangent_half_span;
         ++tangential_step) {
        const cv::Point2f tangential_offset =
            static_cast<float>(tangential_step) * tangent;
        for (int depth_step = 0; depth_step < depth; ++depth_step) {
            const float offset = static_cast<float>(start_offset + depth_step);
            const cv::Point2f sample =
                center_f + tangential_offset + static_cast<float>(side_sign) * offset * normal;
            const int sample_x = static_cast<int>(std::round(sample.x));
            const int sample_y = static_cast<int>(std::round(sample.y));

            ++stats.total_samples;
            if (sample_x < 0 || sample_x >= green_mask.cols || sample_y < 0 ||
                sample_y >= green_mask.rows) {
                ++stats.outside_samples;
                continue;
            }

            if (green_mask.at<uchar>(sample_y, sample_x) != 0) {
                ++stats.green_samples;
            }
            if (black_mask.at<uchar>(sample_y, sample_x) != 0) {
                ++stats.black_samples;
            }
            if (noise_mask.at<uchar>(sample_y, sample_x) != 0) {
                ++stats.noise_samples;
            }
        }
    }

    return stats;
}

cv::Mat filterSkeletonByLength(const cv::Mat &skeleton_mask, int min_length_pixels) {
    CV_Assert(skeleton_mask.type() == CV_8UC1);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    cv::connectedComponentsWithStats(skeleton_mask, labels, stats, centroids, 8, CV_32S);

    cv::Mat filtered = cv::Mat::zeros(skeleton_mask.size(), CV_8UC1);
    for (int label = 1; label < stats.rows; ++label) {
        const int component_area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (component_area < min_length_pixels) {
            continue;
        }
        filtered.setTo(255, labels == label);
    }

    return filtered;
}

cv::Mat createDebugComposite(
    const cv::Mat &green_mask,
    const cv::Mat &black_mask,
    const cv::Mat &white_final_mask) {
    cv::Mat debug_image(green_mask.size(), CV_8UC3, cv::Scalar(30, 70, 30));
    debug_image =
        rcj_loc::vision::debug::createMaskOverlay(debug_image, black_mask, cv::Scalar(0, 0, 255));
    debug_image =
        rcj_loc::vision::debug::createMaskOverlay(debug_image, green_mask, cv::Scalar(0, 200, 0));
    debug_image =
        rcj_loc::vision::debug::createMaskOverlay(
            debug_image,
            white_final_mask,
            cv::Scalar(255, 255, 255));
    return debug_image;
}

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

long long elapsedUs(const TimePoint &start, const TimePoint &end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

enum class SkeletonTimingStage : std::size_t {
    RuntimeSync = 0,
    CvBridgeConvert,
    Skeletonize,
    DistanceTransform,
    SideSupportScan,
    WidthRangeEstimate,
    WidthFilter,
    LengthFilter,
    Reconstruct,
    FinalMask,
    DebugComposite,
    PublishOutputs,
    GuiDisplay,
    CallbackTotal,
    Count
};

constexpr std::array<const char *, static_cast<std::size_t>(SkeletonTimingStage::Count)>
    kSkeletonTimingLabels = {
        "runtime_sync",
        "cv_bridge",
        "skeletonize",
        "distance_transform",
        "side_support_scan",
        "width_range",
        "width_filter",
        "length_filter",
        "reconstruct",
        "final_mask",
        "debug_composite",
        "publish_outputs",
        "gui_display",
        "callback_total",
};

using SkeletonTimingArray =
    std::array<long long, static_cast<std::size_t>(SkeletonTimingStage::Count)>;

struct SkeletonFrameTiming {
    SkeletonTimingArray stage_us{};
};

void recordStageDuration(
    SkeletonTimingArray &target,
    SkeletonTimingStage stage,
    const TimePoint &start,
    const TimePoint &end) {
    target[static_cast<std::size_t>(stage)] = elapsedUs(start, end);
}

template <typename PublisherT>
std::size_t subscriptionCount(const std::shared_ptr<PublisherT> &publisher) {
    return publisher == nullptr ? 0U : publisher->get_subscription_count();
}

template <typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> &publisher) {
    return subscriptionCount(publisher) > 0U;
}

template <typename PublisherT>
bool publishImageIfSubscribed(
    const std::shared_ptr<PublisherT> &publisher,
    const std_msgs::msg::Header &header,
    const std::string &encoding,
    const cv::Mat &image) {
    if (!hasSubscribers(publisher)) {
        return false;
    }
    publisher->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
    return true;
}

void appendTimingTable(
    std::ostringstream &oss,
    const SkeletonFrameTiming &timing,
    const SkeletonTimingArray &interval_totals,
    std::size_t interval_frame_count) {
    const double callback_average_ms =
        interval_frame_count > 0
            ? static_cast<double>(
                  interval_totals[static_cast<std::size_t>(SkeletonTimingStage::CallbackTotal)]) /
                  static_cast<double>(interval_frame_count) / 1000.0
            : 0.0;

    oss << std::left << std::setw(22) << "阶段"
        << std::right << std::setw(12) << "当前ms"
        << std::setw(14) << "区间平均ms"
        << std::setw(12) << "占比%" << '\n';

    for (std::size_t i = 0; i < kSkeletonTimingLabels.size(); ++i) {
        const double current_ms = static_cast<double>(timing.stage_us[i]) / 1000.0;
        const double interval_average_ms =
            interval_frame_count > 0
                ? static_cast<double>(interval_totals[i]) /
                      static_cast<double>(interval_frame_count) / 1000.0
                : 0.0;
        const double ratio =
            callback_average_ms > 0.0 ? (interval_average_ms / callback_average_ms) * 100.0 : 0.0;

        oss << std::left << std::setw(22) << kSkeletonTimingLabels[i]
            << std::right << std::setw(12) << std::fixed << std::setprecision(3) << current_ms
            << std::setw(14) << std::fixed << std::setprecision(3) << interval_average_ms
            << std::setw(12) << std::fixed << std::setprecision(1) << ratio << '\n';
    }
}

}  // namespace

class WhiteLineSkeletonFilterNode : public rclcpp::Node {
public:
    WhiteLineSkeletonFilterNode() : Node("white_line_skeleton_filter_node") {
        this->declare_parameter<std::string>(
            "morph_mask_topic",
            "/white_line_lab_morph_node/white_morph_mask");
        this->declare_parameter<std::string>(
            "green_mask_topic",
            "/white_line_lab_morph_node/green_mask");
        this->declare_parameter<std::string>(
            "black_mask_topic",
            "/white_line_lab_morph_node/black_mask");
        this->declare_parameter<std::string>(
            "noise_mask_topic",
            "/white_line_lab_morph_node/noise_mask");
        // Orientation estimation parameters
        this->declare_parameter("orientation_window_radius_px", 5);
        this->declare_parameter("min_orientation_neighbors", 6);
        // Side support sampling parameters
        this->declare_parameter("side_margin_px", 1);
        this->declare_parameter("side_band_depth_px", 4);
        this->declare_parameter("min_green_ratio", 0.35);
        this->declare_parameter("min_boundary_ratio", 0.35);
        this->declare_parameter("enable_boundary_mode", true);
        // Width filtering parameters
        this->declare_parameter("width_floor_px", 2.0);
        this->declare_parameter("width_ceil_px", 40.0);
        this->declare_parameter("width_mad_scale", 2.5);
        this->declare_parameter("min_width_samples", 25);
        // Skeleton filtering parameters
        this->declare_parameter("min_skeleton_length_px", 12);
        // Reconstruction parameters
        this->declare_parameter("reconstruction_margin_px", 1.0);
        // Display parameters
        this->declare_parameter("enable_image_view", false);
        this->declare_parameter("show_morph_mask", true);
        this->declare_parameter("show_green_mask", false);
        this->declare_parameter("show_black_mask", false);
        this->declare_parameter("show_noise_mask", false);
        this->declare_parameter("show_skeleton_mask", true);
        this->declare_parameter("show_orientation_valid_mask", true);
        this->declare_parameter("show_side_support_mask", true);
        this->declare_parameter("show_width_supported_skeleton_mask", true);
        this->declare_parameter(
            "show_length_filtered_skeleton_mask",
            kDefaultShowLengthFilteredSkeletonMask);
        this->declare_parameter(
            "show_supported_skeleton_mask",
            kDefaultShowLengthFilteredSkeletonMask);
        this->declare_parameter("show_reconstructed_mask", true);
        this->declare_parameter("show_white_final_mask", true);
        this->declare_parameter("show_white_mask", true);
        this->declare_parameter("show_debug_image", true);
        this->declare_parameter("enable_timing_debug", false);
        this->declare_parameter("timing_summary_interval", 10);
        this->declare_parameter("display_max_width", 960);
        this->declare_parameter("display_max_height", 720);

        syncImageViewState();
        setupSubscribers();

        skeleton_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/skeleton_mask", 10);
        orientation_valid_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/orientation_valid_mask", 10);
        side_support_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/side_support_mask", 10);
        width_supported_skeleton_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/width_supported_skeleton_mask", 10);
        length_filtered_skeleton_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/length_filtered_skeleton_mask", 10);
        legacy_supported_skeleton_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/supported_skeleton_mask", 10);
        reconstructed_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/reconstructed_mask", 10);
        white_final_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/white_final_mask", 10);
        legacy_white_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/white_mask", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "white_line_skeleton_filter_node started. enable_image_view=%s, "
            "enable_timing_debug=%s, timing_summary_interval=%d. Waiting for morph masks on '%s'.",
            enable_image_view_ ? "true" : "false",
            enable_timing_debug_ ? "true" : "false",
            timing_summary_interval_,
            morph_mask_topic_.c_str());
    }

    ~WhiteLineSkeletonFilterNode() override {
        destroyDebugWindows();
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ExactPolicy = message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg>;

    message_filters::Subscriber<ImageMsg> morph_sub_;
    message_filters::Subscriber<ImageMsg> green_sub_;
    message_filters::Subscriber<ImageMsg> black_sub_;
    message_filters::Subscriber<ImageMsg> noise_sub_;
    std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr skeleton_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orientation_valid_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr side_support_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr width_supported_skeleton_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr length_filtered_skeleton_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_supported_skeleton_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_final_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr legacy_white_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    std::string morph_mask_topic_;
    std::string green_mask_topic_;
    std::string black_mask_topic_;
    std::string noise_mask_topic_;
    int orientation_window_radius_px_ = 5;
    int min_orientation_neighbors_ = 6;
    int side_margin_px_ = 1;
    int side_band_depth_px_ = 4;
    double min_green_ratio_ = 0.35;
    double min_boundary_ratio_ = 0.35;
    bool enable_boundary_mode_ = true;
    double width_floor_px_ = 2.0;
    double width_ceil_px_ = 40.0;
    double width_mad_scale_ = 2.5;
    int min_width_samples_ = 25;
    int min_skeleton_length_px_ = 12;
    double reconstruction_margin_px_ = 1.0;
    bool enable_image_view_ = false;
    bool show_morph_mask_ = true;
    bool show_green_mask_ = false;
    bool show_black_mask_ = false;
    bool show_noise_mask_ = false;
    bool show_skeleton_mask_ = true;
    bool show_orientation_valid_mask_ = true;
    bool show_side_support_mask_ = true;
    bool show_width_supported_skeleton_mask_ = true;
    bool show_length_filtered_skeleton_mask_ = kDefaultShowLengthFilteredSkeletonMask;
    bool show_reconstructed_mask_ = true;
    bool show_white_final_mask_ = true;
    bool show_debug_image_ = true;
    bool enable_timing_debug_ = false;
    bool morph_window_created_ = false;
    bool green_window_created_ = false;
    bool black_window_created_ = false;
    bool noise_window_created_ = false;
    bool skeleton_window_created_ = false;
    bool orientation_valid_window_created_ = false;
    bool side_support_window_created_ = false;
    bool width_supported_window_created_ = false;
    bool length_filtered_skeleton_window_created_ = false;
    bool reconstructed_window_created_ = false;
    bool white_final_mask_window_created_ = false;
    bool debug_window_created_ = false;
    bool warned_deprecated_show_supported_skeleton_mask_param_ = false;
    bool warned_deprecated_show_white_mask_param_ = false;
    bool warned_deprecated_white_mask_topic_ = false;
    int display_max_width_ = 960;
    int display_max_height_ = 720;
    unsigned long long frame_count_ = 0;
    int timing_summary_interval_ = 10;
    std::size_t timing_frames_in_interval_ = 0;
    unsigned long long timing_interval_start_frame_ = 0;
    long long timing_interval_total_us_ = 0;
    long long timing_interval_max_us_ = 0;
    SkeletonTimingArray timing_stage_interval_totals_{};

    bool isParameterOverridden(const char *name) {
        const auto &overrides = this->get_node_parameters_interface()->get_parameter_overrides();
        return overrides.find(name) != overrides.end();
    }

    bool loadLengthFilteredSkeletonMaskToggle() {
        const bool current_value =
            this->get_parameter("show_length_filtered_skeleton_mask").as_bool();
        const bool legacy_value =
            this->get_parameter("show_supported_skeleton_mask").as_bool();
        const bool current_overridden =
            isParameterOverridden("show_length_filtered_skeleton_mask");
        const bool legacy_overridden =
            isParameterOverridden("show_supported_skeleton_mask");

        if (current_overridden) {
            if (legacy_overridden && current_value != legacy_value &&
                !warned_deprecated_show_supported_skeleton_mask_param_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Both 'show_length_filtered_skeleton_mask' and deprecated "
                    "'show_supported_skeleton_mask' were provided. "
                    "Using 'show_length_filtered_skeleton_mask'.");
                warned_deprecated_show_supported_skeleton_mask_param_ = true;
            }
            return current_value;
        }

        if (legacy_overridden) {
            if (!warned_deprecated_show_supported_skeleton_mask_param_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Parameter 'show_supported_skeleton_mask' is deprecated. "
                    "Use 'show_length_filtered_skeleton_mask' instead.");
                warned_deprecated_show_supported_skeleton_mask_param_ = true;
            }
            return legacy_value;
        }

        return current_value;
    }

    bool loadWhiteFinalMaskToggle() {
        const bool current_value =
            this->get_parameter("show_white_final_mask").as_bool();
        const bool legacy_value =
            this->get_parameter("show_white_mask").as_bool();
        const bool current_overridden =
            isParameterOverridden("show_white_final_mask");
        const bool legacy_overridden =
            isParameterOverridden("show_white_mask");

        if (current_overridden) {
            if (legacy_overridden && current_value != legacy_value &&
                !warned_deprecated_show_white_mask_param_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Both 'show_white_final_mask' and deprecated 'show_white_mask' were "
                    "provided. Using 'show_white_final_mask'.");
                warned_deprecated_show_white_mask_param_ = true;
            }
            return current_value;
        }

        if (legacy_overridden) {
            if (!warned_deprecated_show_white_mask_param_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Parameter 'show_white_mask' is deprecated. Use "
                    "'show_white_final_mask' instead.");
                warned_deprecated_show_white_mask_param_ = true;
            }
            return legacy_value;
        }

        return current_value;
    }

    void loadRuntimeParameters() {
        morph_mask_topic_ = this->get_parameter("morph_mask_topic").as_string();
        green_mask_topic_ = this->get_parameter("green_mask_topic").as_string();
        black_mask_topic_ = this->get_parameter("black_mask_topic").as_string();
        noise_mask_topic_ = this->get_parameter("noise_mask_topic").as_string();
        orientation_window_radius_px_ =
            std::max(1, static_cast<int>(this->get_parameter("orientation_window_radius_px").as_int()));
        min_orientation_neighbors_ =
            std::max(2, static_cast<int>(this->get_parameter("min_orientation_neighbors").as_int()));
        side_margin_px_ = std::max(0, static_cast<int>(this->get_parameter("side_margin_px").as_int()));
        side_band_depth_px_ =
            std::max(1, static_cast<int>(this->get_parameter("side_band_depth_px").as_int()));
        min_green_ratio_ =
            std::clamp(this->get_parameter("min_green_ratio").as_double(), 0.0, 1.0);
        min_boundary_ratio_ =
            std::clamp(this->get_parameter("min_boundary_ratio").as_double(), 0.0, 1.0);
        enable_boundary_mode_ = this->get_parameter("enable_boundary_mode").as_bool();
        width_floor_px_ = std::max(0.0, this->get_parameter("width_floor_px").as_double());
        width_ceil_px_ = std::max(width_floor_px_, this->get_parameter("width_ceil_px").as_double());
        width_mad_scale_ = std::max(0.0, this->get_parameter("width_mad_scale").as_double());
        min_width_samples_ =
            std::max(1, static_cast<int>(this->get_parameter("min_width_samples").as_int()));
        min_skeleton_length_px_ =
            std::max(1, static_cast<int>(this->get_parameter("min_skeleton_length_px").as_int()));
        reconstruction_margin_px_ =
            std::max(0.0, this->get_parameter("reconstruction_margin_px").as_double());
        enable_image_view_ = this->get_parameter("enable_image_view").as_bool();
        show_morph_mask_ = this->get_parameter("show_morph_mask").as_bool();
        show_green_mask_ = this->get_parameter("show_green_mask").as_bool();
        show_black_mask_ = this->get_parameter("show_black_mask").as_bool();
        show_noise_mask_ = this->get_parameter("show_noise_mask").as_bool();
        show_skeleton_mask_ = this->get_parameter("show_skeleton_mask").as_bool();
        show_orientation_valid_mask_ = this->get_parameter("show_orientation_valid_mask").as_bool();
        show_side_support_mask_ = this->get_parameter("show_side_support_mask").as_bool();
        show_width_supported_skeleton_mask_ =
            this->get_parameter("show_width_supported_skeleton_mask").as_bool();
        show_length_filtered_skeleton_mask_ = loadLengthFilteredSkeletonMaskToggle();
        show_reconstructed_mask_ = this->get_parameter("show_reconstructed_mask").as_bool();
        show_white_final_mask_ = loadWhiteFinalMaskToggle();
        show_debug_image_ = this->get_parameter("show_debug_image").as_bool();
        enable_timing_debug_ = this->get_parameter("enable_timing_debug").as_bool();
        timing_summary_interval_ =
            std::max(1, static_cast<int>(this->get_parameter("timing_summary_interval").as_int()));
        display_max_width_ = std::max(1, static_cast<int>(this->get_parameter("display_max_width").as_int()));
        display_max_height_ =
            std::max(1, static_cast<int>(this->get_parameter("display_max_height").as_int()));
    }

    void resetTimingSummary() {
        timing_frames_in_interval_ = 0;
        timing_interval_start_frame_ = 0;
        timing_interval_total_us_ = 0;
        timing_interval_max_us_ = 0;
        timing_stage_interval_totals_.fill(0);
    }

    void maybeLogTimingSummary(
        const SkeletonFrameTiming &timing,
        unsigned long long current_frame_index) {
        if (!enable_timing_debug_) {
            return;
        }

        const long long skeleton_duration_us =
            timing.stage_us[static_cast<std::size_t>(SkeletonTimingStage::CallbackTotal)];

        if (timing_frames_in_interval_ == 0U) {
            timing_interval_start_frame_ = current_frame_index;
        }

        ++timing_frames_in_interval_;
        timing_interval_total_us_ += skeleton_duration_us;
        timing_interval_max_us_ = std::max(timing_interval_max_us_, skeleton_duration_us);
        for (std::size_t i = 0; i < timing_stage_interval_totals_.size(); ++i) {
            timing_stage_interval_totals_[i] += timing.stage_us[i];
        }

        if (timing_frames_in_interval_ < static_cast<std::size_t>(timing_summary_interval_)) {
            return;
        }

        const double avg_ms =
            static_cast<double>(timing_interval_total_us_) /
            static_cast<double>(timing_frames_in_interval_) / 1000.0;

        std::ostringstream oss;
        oss << "\n================ Skeleton Profiling 摘要 ================\n";
        oss << "帧区间: " << timing_interval_start_frame_ << " - " << current_frame_index
            << " (" << timing_frames_in_interval_ << " 帧)\n";
        oss << "当前帧总耗时: " << static_cast<double>(skeleton_duration_us) / 1000.0
            << " ms, 区间平均: " << avg_ms
            << " ms, 区间最大: " << static_cast<double>(timing_interval_max_us_) / 1000.0
            << " ms\n\n";
        appendTimingTable(oss, timing, timing_stage_interval_totals_, timing_frames_in_interval_);
        oss << "========================================================";
        RCLCPP_INFO_STREAM(this->get_logger(), oss.str());

        resetTimingSummary();
    }

    bool anyImageWindowRequested() const {
        return show_morph_mask_ || show_green_mask_ || show_black_mask_ || show_noise_mask_ ||
               show_skeleton_mask_ || show_orientation_valid_mask_ || show_side_support_mask_ ||
               show_width_supported_skeleton_mask_ || show_length_filtered_skeleton_mask_ ||
               show_reconstructed_mask_ || show_white_final_mask_ || show_debug_image_;
    }

    void syncWindow(const std::string &window_name, bool should_show, bool &created) {
        if (should_show && !created) {
            cv::namedWindow(window_name, cv::WINDOW_NORMAL);
            created = true;
        } else if (!should_show && created) {
            cv::destroyWindow(window_name);
            created = false;
        }
    }

    void destroyDebugWindows() {
        syncWindow(kMorphWindow, false, morph_window_created_);
        syncWindow(kGreenWindow, false, green_window_created_);
        syncWindow(kBlackWindow, false, black_window_created_);
        syncWindow(kNoiseWindow, false, noise_window_created_);
        syncWindow(kSkeletonWindow, false, skeleton_window_created_);
        syncWindow(kOrientationValidWindow, false, orientation_valid_window_created_);
        syncWindow(kSideSupportWindow, false, side_support_window_created_);
        syncWindow(kWidthSupportedWindow, false, width_supported_window_created_);
        syncWindow(
            kLengthFilteredSkeletonWindow,
            false,
            length_filtered_skeleton_window_created_);
        syncWindow(kReconstructedWindow, false, reconstructed_window_created_);
        syncWindow(kWhiteFinalMaskWindow, false, white_final_mask_window_created_);
        syncWindow(kDebugWindow, false, debug_window_created_);
    }

    void syncImageViewState() {
        loadRuntimeParameters();
        const bool master_enabled = enable_image_view_ && anyImageWindowRequested();
        syncWindow(kMorphWindow, master_enabled && show_morph_mask_, morph_window_created_);
        syncWindow(kGreenWindow, master_enabled && show_green_mask_, green_window_created_);
        syncWindow(kBlackWindow, master_enabled && show_black_mask_, black_window_created_);
        syncWindow(kNoiseWindow, master_enabled && show_noise_mask_, noise_window_created_);
        syncWindow(kSkeletonWindow, master_enabled && show_skeleton_mask_, skeleton_window_created_);
        syncWindow(
            kOrientationValidWindow,
            master_enabled && show_orientation_valid_mask_,
            orientation_valid_window_created_);
        syncWindow(
            kSideSupportWindow,
            master_enabled && show_side_support_mask_,
            side_support_window_created_);
        syncWindow(
            kWidthSupportedWindow,
            master_enabled && show_width_supported_skeleton_mask_,
            width_supported_window_created_);
        syncWindow(
            kLengthFilteredSkeletonWindow,
            master_enabled && show_length_filtered_skeleton_mask_,
            length_filtered_skeleton_window_created_);
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

    void setupSubscribers() {
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
            &WhiteLineSkeletonFilterNode::maskCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3,
            std::placeholders::_4));
    }

    bool publishOutputs(
        const std_msgs::msg::Header &header,
        const cv::Mat &skeleton_mask,
        const cv::Mat &orientation_valid_mask,
        const cv::Mat &side_support_mask,
        const cv::Mat &width_supported_skeleton_mask,
        const cv::Mat &length_filtered_skeleton_mask,
        const cv::Mat &reconstructed_mask,
        const cv::Mat &white_final_mask,
        const cv::Mat &debug_image) {
        bool published_any = false;
        const bool debug_outputs_enabled = enable_image_view_;

        published_any |=
            publishImageIfSubscribed(white_final_mask_pub_, header, "mono8", white_final_mask);

        if (hasSubscribers(legacy_white_mask_pub_)) {
            if (!warned_deprecated_white_mask_topic_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Topic '~/white_mask' is deprecated. Use '~/white_final_mask' instead.");
                warned_deprecated_white_mask_topic_ = true;
            }
            legacy_white_mask_pub_->publish(
                *cv_bridge::CvImage(header, "mono8", white_final_mask).toImageMsg());
            published_any = true;
        }

        if (!debug_outputs_enabled) {
            return published_any;
        }

        published_any |= publishImageIfSubscribed(skeleton_mask_pub_, header, "mono8", skeleton_mask);
        published_any |= publishImageIfSubscribed(
            orientation_valid_mask_pub_,
            header,
            "mono8",
            orientation_valid_mask);
        published_any |=
            publishImageIfSubscribed(side_support_mask_pub_, header, "mono8", side_support_mask);
        published_any |= publishImageIfSubscribed(
            width_supported_skeleton_mask_pub_,
            header,
            "mono8",
            width_supported_skeleton_mask);
        published_any |= publishImageIfSubscribed(
            length_filtered_skeleton_mask_pub_,
            header,
            "mono8",
            length_filtered_skeleton_mask);
        published_any |= publishImageIfSubscribed(
            legacy_supported_skeleton_mask_pub_,
            header,
            "mono8",
            length_filtered_skeleton_mask);
        published_any |= publishImageIfSubscribed(
            reconstructed_mask_pub_,
            header,
            "mono8",
            reconstructed_mask);
        if (!debug_image.empty()) {
            published_any |= publishImageIfSubscribed(debug_pub_, header, "bgr8", debug_image);
        }

        return published_any;
    }

    bool displayOutputs(
        const cv::Mat &morph_mask,
        const cv::Mat &skeleton_mask,
        const cv::Mat &orientation_valid_mask,
        const cv::Mat &side_support_mask,
        const cv::Mat &width_supported_skeleton_mask,
        const cv::Mat &length_filtered_skeleton_mask,
        const cv::Mat &reconstructed_mask,
        const cv::Mat &white_final_mask,
        const cv::Mat &green_mask,
        const cv::Mat &black_mask,
        const cv::Mat &noise_mask,
        const cv::Mat &debug_image) {
        bool displayed_any_window = false;
        if (morph_window_created_) {
            cv::imshow(kMorphWindow, morph_mask);
            resizeWindowToFitImage(kMorphWindow, morph_mask, display_max_width_, display_max_height_);
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
        if (skeleton_window_created_) {
            cv::imshow(kSkeletonWindow, skeleton_mask);
            resizeWindowToFitImage(kSkeletonWindow, skeleton_mask, display_max_width_, display_max_height_);
            displayed_any_window = true;
        }
        if (orientation_valid_window_created_) {
            cv::imshow(kOrientationValidWindow, orientation_valid_mask);
            resizeWindowToFitImage(
                kOrientationValidWindow,
                orientation_valid_mask,
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
            cv::imshow(kWidthSupportedWindow, width_supported_skeleton_mask);
            resizeWindowToFitImage(
                kWidthSupportedWindow,
                width_supported_skeleton_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (length_filtered_skeleton_window_created_) {
            cv::imshow(kLengthFilteredSkeletonWindow, length_filtered_skeleton_mask);
            resizeWindowToFitImage(
                kLengthFilteredSkeletonWindow,
                length_filtered_skeleton_mask,
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
        const ImageMsg::ConstSharedPtr &morph_msg,
        const ImageMsg::ConstSharedPtr &green_msg,
        const ImageMsg::ConstSharedPtr &black_msg,
        const ImageMsg::ConstSharedPtr &noise_msg) {
        SkeletonFrameTiming timing;
        const bool timing_enabled = enable_timing_debug_;
        const auto callback_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        auto stage_start = timing_enabled ? callback_start : TimePoint{};
        const bool previous_timing_debug = enable_timing_debug_;
        const int previous_timing_summary_interval = timing_summary_interval_;
        syncImageViewState();
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::RuntimeSync,
                stage_start,
                SteadyClock::now());
        }
        if (previous_timing_debug != enable_timing_debug_ ||
            previous_timing_summary_interval != timing_summary_interval_) {
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
                    SkeletonTimingStage::CvBridgeConvert,
                    stage_start,
                    SteadyClock::now());
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cv_bridge failed: %s",
                e.what());
            return;
        }

        if (morph_mask.size() != green_mask.size() || morph_mask.size() != black_mask.size() ||
            morph_mask.size() != noise_mask.size()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Skeleton filter received mismatched mask sizes.");
            return;
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const cv::Mat skeleton_mask = skeletonize(morph_mask);
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::Skeletonize,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::Mat distance_transform;
        cv::distanceTransform(morph_mask, distance_transform, cv::DIST_L2, 3);
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::DistanceTransform,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat orientation_valid_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
        cv::Mat side_support_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
        cv::Mat local_width_map = cv::Mat::zeros(morph_mask.size(), CV_32FC1);
        std::vector<float> supported_widths;
        supported_widths.reserve(static_cast<std::size_t>(cv::countNonZero(skeleton_mask)));

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        for (int y = 0; y < skeleton_mask.rows; ++y) {
            for (int x = 0; x < skeleton_mask.cols; ++x) {
                if (skeleton_mask.at<uchar>(y, x) == 0) {
                    continue;
                }

                const float local_width_px = 2.0f * distance_transform.at<float>(y, x);
                if (local_width_px <= 0.0f) {
                    continue;
                }

                cv::Point2f tangent;
                cv::Point2f normal;
                if (!estimateOrientation(
                        skeleton_mask,
                        cv::Point(x, y),
                        orientation_window_radius_px_,
                        min_orientation_neighbors_,
                        tangent,
                        normal)) {
                    continue;
                }

                orientation_valid_mask.at<uchar>(y, x) = 255;
                local_width_map.at<float>(y, x) = local_width_px;
                const SideSampleStats left_stats = sampleSideSupport(
                    cv::Point(x, y),
                    tangent,
                    normal,
                    local_width_px,
                    -1,
                    green_mask,
                    black_mask,
                    noise_mask,
                    side_margin_px_,
                    side_band_depth_px_);
                const SideSampleStats right_stats = sampleSideSupport(
                    cv::Point(x, y),
                    tangent,
                    normal,
                    local_width_px,
                    1,
                    green_mask,
                    black_mask,
                    noise_mask,
                    side_margin_px_,
                    side_band_depth_px_);

                const bool interior_supported =
                    left_stats.greenRatio() >= min_green_ratio_ &&
                    right_stats.greenRatio() >= min_green_ratio_;
                const bool boundary_supported =
                    enable_boundary_mode_ &&
                    ((left_stats.greenRatio() >= min_green_ratio_ &&
                      right_stats.boundaryRatio() >= min_boundary_ratio_) ||
                     (right_stats.greenRatio() >= min_green_ratio_ &&
                      left_stats.boundaryRatio() >= min_boundary_ratio_));

                if (!interior_supported && !boundary_supported) {
                    continue;
                }

                side_support_mask.at<uchar>(y, x) = 255;
                supported_widths.push_back(local_width_px);
            }
        }
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::SideSupportScan,
                stage_start,
                SteadyClock::now());
        }

        double width_lower_bound = width_floor_px_;
        double width_upper_bound = width_ceil_px_;
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        if (static_cast<int>(supported_widths.size()) >= min_width_samples_) {
            const double width_median = computeMedian(supported_widths);
            std::vector<float> deviations;
            deviations.reserve(supported_widths.size());
            for (const float width : supported_widths) {
                deviations.push_back(static_cast<float>(std::abs(width - width_median)));
            }
            const double width_mad = computeMedian(deviations);
            width_lower_bound = std::max(width_floor_px_, width_median - width_mad_scale_ * width_mad);
            width_upper_bound = std::min(width_ceil_px_, width_median + width_mad_scale_ * width_mad);
            if (width_upper_bound < width_lower_bound) {
                width_lower_bound = width_floor_px_;
                width_upper_bound = width_ceil_px_;
            }
        }
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::WidthRangeEstimate,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat width_supported_skeleton = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        for (int y = 0; y < side_support_mask.rows; ++y) {
            for (int x = 0; x < side_support_mask.cols; ++x) {
                if (side_support_mask.at<uchar>(y, x) == 0) {
                    continue;
                }
                const float local_width_px = local_width_map.at<float>(y, x);
                if (local_width_px < width_lower_bound || local_width_px > width_upper_bound) {
                    continue;
                }
                width_supported_skeleton.at<uchar>(y, x) = 255;
            }
        }
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::WidthFilter,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const cv::Mat length_filtered_skeleton_mask =
            filterSkeletonByLength(width_supported_skeleton, min_skeleton_length_px_);
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::LengthFilter,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat reconstructed_mask = cv::Mat::zeros(morph_mask.size(), CV_8UC1);
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        for (int y = 0; y < length_filtered_skeleton_mask.rows; ++y) {
            for (int x = 0; x < length_filtered_skeleton_mask.cols; ++x) {
                if (length_filtered_skeleton_mask.at<uchar>(y, x) == 0) {
                    continue;
                }
                const float local_width_px = local_width_map.at<float>(y, x);
                const int radius = std::max(
                    1,
                    static_cast<int>(std::round(0.5 * local_width_px + reconstruction_margin_px_)));
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
                SkeletonTimingStage::Reconstruct,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::Mat white_final_mask;
        cv::bitwise_and(reconstructed_mask, morph_mask, white_final_mask);
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::FinalMask,
                stage_start,
                SteadyClock::now());
        }
        const unsigned long long current_frame_index = ++frame_count_;

        const bool debug_image_needed =
            debug_window_created_ || (enable_image_view_ && hasSubscribers(debug_pub_));
        cv::Mat debug_image;
        if (debug_image_needed) {
            stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
            debug_image = createDebugComposite(green_mask, black_mask, white_final_mask);
            if (timing_enabled) {
                recordStageDuration(
                    timing.stage_us,
                    SkeletonTimingStage::DebugComposite,
                    stage_start,
                    SteadyClock::now());
            }
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const bool published_any = publishOutputs(
            morph_msg->header,
            skeleton_mask,
            orientation_valid_mask,
            side_support_mask,
            width_supported_skeleton,
            length_filtered_skeleton_mask,
            reconstructed_mask,
            white_final_mask,
            debug_image);
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::PublishOutputs,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const bool displayed_any = displayOutputs(
            morph_mask,
            skeleton_mask,
            orientation_valid_mask,
            side_support_mask,
            width_supported_skeleton,
            length_filtered_skeleton_mask,
            reconstructed_mask,
            white_final_mask,
            green_mask,
            black_mask,
            noise_mask,
            debug_image);
        if (timing_enabled && displayed_any) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::GuiDisplay,
                stage_start,
                SteadyClock::now());
        }
        if (timing_enabled && !published_any) {
            timing.stage_us[static_cast<std::size_t>(SkeletonTimingStage::PublishOutputs)] = 0;
        }
        if (timing_enabled) {
            recordStageDuration(
                timing.stage_us,
                SkeletonTimingStage::CallbackTotal,
                callback_start,
                SteadyClock::now());
        }

        maybeLogTimingSummary(timing, current_frame_index);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhiteLineSkeletonFilterNode>());
    rclcpp::shutdown();
    return 0;
}
