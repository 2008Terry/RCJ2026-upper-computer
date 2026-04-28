#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "rcj_localization/white_line_debug_utils.hpp"
#include "rcj_localization/white_line_preprocessing.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

constexpr int kByteMax = 255;
constexpr char kControlsWindow[] = "Lab Morph Controls";
constexpr char kOriginalWindow[] = "Lab Morph Original";
constexpr char kWhiteCandidateWindow[] = "Lab Morph White Candidate";
constexpr char kWhiteMorphWindow[] = "Lab Morph White Morph";
constexpr char kGreenMaskWindow[] = "Lab Morph Green Mask";
constexpr char kBlackMaskWindow[] = "Lab Morph Black Mask";
constexpr char kNoiseMaskWindow[] = "Lab Morph Noise Mask";
constexpr char kOverlayWindow[] = "Lab Morph Overlay";

enum class MainTimingStage : std::size_t {
    RuntimeSync = 0,
    RosToCv,
    Preprocess,
    GreenCandidate,
    WhiteCandidate,
    BlackCandidate,
    RemainingPartition,
    KernelCreate,
    MorphOpen,
    MorphClose,
    Overlay,
    Publish,
    Gui,
    UnaccountedOverhead,
    CallbackTotal,
    Count,
};

enum class WhiteCandidateTimingStage : std::size_t {
    BrightThreshold = 0,
    LabADelta,
    LabBDelta,
    AToleranceCheck,
    BToleranceCheck,
    NeutralCombine,
    WhiteCombine,
    Count,
};

constexpr std::size_t kMainTimingStageCount =
    static_cast<std::size_t>(MainTimingStage::Count);
constexpr std::size_t kWhiteCandidateTimingStageCount =
    static_cast<std::size_t>(WhiteCandidateTimingStage::Count);

using MainTimingArray = std::array<long long, kMainTimingStageCount>;
using MainHitArray = std::array<std::size_t, kMainTimingStageCount>;
using WhiteTimingArray = std::array<long long, kWhiteCandidateTimingStageCount>;
using WhiteHitArray = std::array<std::size_t, kWhiteCandidateTimingStageCount>;
using PreprocessTimingArray =
    std::array<long long, rcj_loc::vision::white_line::kPreprocessTimingStageCount>;
using PreprocessHitArray =
    std::array<std::size_t, rcj_loc::vision::white_line::kPreprocessTimingStageCount>;

constexpr std::array<const char *, kMainTimingStageCount> kMainTimingLabels = {
    "参数/窗口同步",
    "ROS图像转CV",
    "预处理总计",
    "绿色候选",
    "白色候选总计",
    "黑色候选",
    "remaining分类",
    "kernel创建",
    "morph open",
    "morph close",
    "overlay构建",
    "消息发布",
    "GUI显示/waitKey",
    "未归类开销",
    "callback总计",
};

constexpr std::array<const char *, rcj_loc::vision::white_line::kPreprocessTimingStageCount>
    kPreprocessTimingLabels = {
        "medianBlur",
        "BGR2Lab",
        "split",
        "亮度选择",
        "CLAHE创建",
        "CLAHE应用",
        "GaussianBlur",
        "sharpen",
    };

constexpr std::array<const char *, kWhiteCandidateTimingStageCount>
    kWhiteCandidateTimingLabels = {
        "亮度阈值",
        "lab_a偏差",
        "lab_b偏差",
        "a容差判断",
        "b容差判断",
        "neutral合并",
        "white合并",
    };

struct WhiteCandidateTiming {
    WhiteTimingArray stage_us{};
};

struct FrameTimingCapture {
    MainTimingArray main_stage_us{};
    rcj_loc::vision::white_line::PreprocessTiming preprocess_timing{};
    WhiteCandidateTiming white_candidate_timing{};
};

struct TimingSummaryContext {
    int image_width = 0;
    int image_height = 0;
    std::string encoding;
    std::size_t white_candidate_subscribers = 0;
    std::size_t white_morph_subscribers = 0;
    std::size_t green_subscribers = 0;
    std::size_t black_subscribers = 0;
    std::size_t noise_subscribers = 0;
    std::size_t debug_subscribers = 0;
    bool image_view_enabled = false;
    bool publish_debug_image = false;
    bool overlay_requested_for_window = false;
    bool overlay_requested_for_publish = false;
};

long long elapsedUs(const TimePoint &start, const TimePoint &end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

template <typename Enum, std::size_t N>
void recordStageDuration(
    std::array<long long, N> &target,
    Enum stage,
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

template <std::size_t N>
void resetInterval(std::array<long long, N> &totals, std::array<std::size_t, N> &hits) {
    totals.fill(0);
    hits.fill(0);
}

template <std::size_t N>
void accumulateInterval(
    const std::array<long long, N> &current,
    std::array<long long, N> &totals,
    std::array<std::size_t, N> &hits) {
    for (std::size_t i = 0; i < N; ++i) {
        if (current[i] <= 0) {
            continue;
        }
        totals[i] += current[i];
        ++hits[i];
    }
}

template <std::size_t N>
void appendTimingTable(
    std::ostringstream &oss,
    const std::string &title,
    const std::array<const char *, N> &labels,
    const std::array<long long, N> &current,
    const std::array<long long, N> &interval_totals,
    const std::array<std::size_t, N> &interval_hits,
    std::size_t interval_frame_count,
    double callback_average_ms,
    std::size_t always_show_index = N) {
    oss << title << '\n';
    oss << std::left << std::setw(22) << "阶段"
        << std::right << std::setw(12) << "当前ms"
        << std::setw(14) << "区间平均ms"
        << std::setw(12) << "占比%" << '\n';

    for (std::size_t i = 0; i < N; ++i) {
        const bool should_show =
            (i == always_show_index) || current[i] > 0 || interval_totals[i] > 0 ||
            interval_hits[i] > 0;
        if (!should_show) {
            continue;
        }

        const double current_ms = static_cast<double>(current[i]) / 1000.0;
        const double interval_average_ms = interval_frame_count > 0
                                               ? static_cast<double>(interval_totals[i]) /
                                                     static_cast<double>(interval_frame_count) /
                                                     1000.0
                                               : 0.0;
        const double ratio =
            callback_average_ms > 0.0 ? (interval_average_ms / callback_average_ms) * 100.0 : 0.0;

        oss << std::left << std::setw(22) << labels[i]
            << std::right << std::setw(12) << std::fixed << std::setprecision(3) << current_ms
            << std::setw(14) << std::fixed << std::setprecision(3) << interval_average_ms
            << std::setw(12) << std::fixed << std::setprecision(1) << ratio << '\n';
    }
}

long long sumMainTimingStagesExcludingCallback(const MainTimingArray &values) {
    long long total_us = 0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i == static_cast<std::size_t>(MainTimingStage::UnaccountedOverhead) ||
            i == static_cast<std::size_t>(MainTimingStage::CallbackTotal)) {
            continue;
        }
        total_us += values[i];
    }
    return total_us;
}

std::string toToggleText(bool enabled) {
    return enabled ? "开" : "关";
}

int clampToByte(int value) {
    return std::clamp(value, 0, kByteMax);
}

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

cv::Mat takeFromRemaining(const cv::Mat &candidate, cv::Mat &remaining) {
    cv::Mat assigned;
    cv::bitwise_and(candidate, remaining, assigned);

    cv::Mat assigned_inv;
    cv::bitwise_not(assigned, assigned_inv);
    cv::bitwise_and(remaining, assigned_inv, remaining);
    return assigned;
}

cv::Mat makeWhiteCandidate(
    const cv::Mat &enhanced,
    const cv::Mat &lab_a,
    const cv::Mat &lab_b,
    int enhanced_min,
    int a_center,
    int b_center,
    int a_tol,
    int b_tol,
    WhiteCandidateTiming *timing = nullptr) {
    if (timing != nullptr) {
        timing->stage_us.fill(0);
    }

    cv::Mat bright_mask;
    auto stage_start = SteadyClock::now();
    cv::inRange(enhanced, cv::Scalar(enhanced_min), cv::Scalar(kByteMax), bright_mask);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::BrightThreshold,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat a_delta;
    stage_start = SteadyClock::now();
    cv::absdiff(lab_a, cv::Scalar(a_center), a_delta);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::LabADelta,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat b_delta;
    stage_start = SteadyClock::now();
    cv::absdiff(lab_b, cv::Scalar(b_center), b_delta);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::LabBDelta,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat a_mask;
    stage_start = SteadyClock::now();
    cv::inRange(a_delta, cv::Scalar(0), cv::Scalar(a_tol), a_mask);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::AToleranceCheck,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat b_mask;
    stage_start = SteadyClock::now();
    cv::inRange(b_delta, cv::Scalar(0), cv::Scalar(b_tol), b_mask);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::BToleranceCheck,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat neutral_mask;
    stage_start = SteadyClock::now();
    cv::bitwise_and(a_mask, b_mask, neutral_mask);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::NeutralCombine,
            stage_start,
            SteadyClock::now());
    }

    cv::Mat white_candidate;
    stage_start = SteadyClock::now();
    cv::bitwise_and(bright_mask, neutral_mask, white_candidate);
    if (timing != nullptr) {
        recordStageDuration(
            timing->stage_us,
            WhiteCandidateTimingStage::WhiteCombine,
            stage_start,
            SteadyClock::now());
    }
    return white_candidate;
}

}  // namespace

class WhiteLineLabMorphNode : public rclcpp::Node {
public:
    WhiteLineLabMorphNode() : Node("white_line_lab_morph_node") {
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("robot_mask_topic", "");
        rcj_loc::vision::white_line::declarePreprocessParameters(*this);

        this->declare_parameter("white_enhanced_min", 170);
        this->declare_parameter("white_a_center", 128);
        this->declare_parameter("white_b_center", 128);
        this->declare_parameter("white_a_tol", 14);
        this->declare_parameter("white_b_tol", 14);
        this->declare_parameter("green_a_max", 115);
        this->declare_parameter("green_b_min", 83);
        this->declare_parameter("green_b_max", 170);
        this->declare_parameter("black_enhanced_max", 123);
        this->declare_parameter("white_open_kernel", 4);
        this->declare_parameter("white_close_kernel", 5);
        this->declare_parameter("enable_image_view", false);
        this->declare_parameter("show_input_image", true);
        this->declare_parameter("show_white_candidate_mask", true);
        this->declare_parameter("show_white_morph_mask", true);
        this->declare_parameter("show_green_mask", true);
        this->declare_parameter("show_black_mask", false);
        this->declare_parameter("show_noise_mask", false);
        this->declare_parameter("show_debug_image", false);
        this->declare_parameter("publish_debug_image", false);
        this->declare_parameter("enable_timing_debug", false);
        this->declare_parameter("timing_summary_interval", 10);
        this->declare_parameter("display_max_width", 960);
        this->declare_parameter("display_max_height", 720);

        loadTimingParameters();
        loadProcessingParameters();

        const auto topic = this->get_parameter("input_topic").as_string();
        const auto robot_mask_topic = this->get_parameter("robot_mask_topic").as_string();
        robot_mask_enabled_ = !robot_mask_topic.empty();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic,
            rclcpp::SensorDataQoS(),
            std::bind(&WhiteLineLabMorphNode::imageCallback, this, std::placeholders::_1));
        if (robot_mask_enabled_) {
            robot_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                robot_mask_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&WhiteLineLabMorphNode::robotMaskCallback, this, std::placeholders::_1));
        }

        white_candidate_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/white_candidate_mask", 10);
        white_morph_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/white_morph_mask", 10);
        green_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/green_mask", 10);
        black_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/black_mask", 10);
        noise_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/noise_mask", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);

        syncImageViewState();

        RCLCPP_INFO(
            this->get_logger(),
            "white_line_lab_morph_node started. robot_mask_topic=%s, enable_image_view=%s, "
            "enable_timing_debug=%s, publish_debug_image=%s",
            robot_mask_topic.empty() ? "<disabled>" : robot_mask_topic.c_str(),
            enable_image_view_ ? "true" : "false",
            enable_timing_debug_ ? "true" : "false",
            publish_debug_image_ ? "true" : "false");
    }

    ~WhiteLineLabMorphNode() override {
        destroyDebugWindows();
    }

private:
    void robotMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            robot_allowed_mask_ = cv_bridge::toCvCopy(msg, "mono8")->image;
            robot_mask_received_ = true;
            robot_mask_validated_ = false;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cv_bridge failed while reading robot mask: %s",
                e.what());
        }
    }

    bool validateRobotMaskForFrame(const cv::Mat &frame) {
        if (!robot_mask_enabled_) {
            return true;
        }

        if (!robot_mask_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "robot_mask_topic is configured but no robot mask has been received yet; dropping frame.");
            return false;
        }

        if (robot_mask_validated_) {
            return true;
        }

        if (robot_allowed_mask_.size() != frame.size()) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Robot mask size mismatch: mask=%dx%d image=%dx%d. Stopping node.",
                robot_allowed_mask_.cols,
                robot_allowed_mask_.rows,
                frame.cols,
                frame.rows);
            image_sub_.reset();
            rclcpp::shutdown();
            return false;
        }

        robot_mask_validated_ = true;
        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_mask_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_candidate_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_morph_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr green_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr black_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr noise_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

    int white_enhanced_min_;
    int white_a_center_;
    int white_b_center_;
    int white_a_tol_;
    int white_b_tol_;
    int green_a_max_;
    int green_b_min_;
    int green_b_max_;
    int black_enhanced_max_;
    int white_open_kernel_;
    int white_close_kernel_;
    bool enable_image_view_ = false;
    bool show_input_image_ = true;
    bool show_white_candidate_mask_ = true;
    bool show_white_morph_mask_ = true;
    bool show_green_mask_ = true;
    bool show_black_mask_ = false;
    bool show_noise_mask_ = false;
    bool show_debug_image_ = false;
    bool publish_debug_image_ = false;
    bool enable_timing_debug_ = false;
    int timing_summary_interval_ = 10;
    bool controls_window_created_ = false;
    bool original_window_created_ = false;
    bool white_candidate_window_created_ = false;
    bool white_morph_window_created_ = false;
    bool green_mask_window_created_ = false;
    bool black_mask_window_created_ = false;
    bool noise_mask_window_created_ = false;
    bool overlay_window_created_ = false;
    int display_max_width_ = 960;
    int display_max_height_ = 720;
    bool robot_mask_enabled_ = false;
    bool robot_mask_received_ = false;
    bool robot_mask_validated_ = false;
    cv::Mat robot_allowed_mask_;
    unsigned long long frame_count_ = 0;
    std::size_t timing_frames_in_interval_ = 0;
    unsigned long long timing_interval_start_frame_ = 0;
    MainTimingArray main_stage_interval_totals_{};
    MainHitArray main_stage_interval_hits_{};
    PreprocessTimingArray preprocess_stage_interval_totals_{};
    PreprocessHitArray preprocess_stage_interval_hits_{};
    WhiteTimingArray white_stage_interval_totals_{};
    WhiteHitArray white_stage_interval_hits_{};

    void loadTimingParameters() {
        enable_timing_debug_ = this->get_parameter("enable_timing_debug").as_bool();
        timing_summary_interval_ =
            std::max(1, static_cast<int>(this->get_parameter("timing_summary_interval").as_int()));
    }

    void loadProcessingParameters() {
        white_enhanced_min_ =
            clampToByte(static_cast<int>(this->get_parameter("white_enhanced_min").as_int()));
        white_a_center_ =
            clampToByte(static_cast<int>(this->get_parameter("white_a_center").as_int()));
        white_b_center_ =
            clampToByte(static_cast<int>(this->get_parameter("white_b_center").as_int()));
        white_a_tol_ =
            clampToByte(static_cast<int>(this->get_parameter("white_a_tol").as_int()));
        white_b_tol_ =
            clampToByte(static_cast<int>(this->get_parameter("white_b_tol").as_int()));
        green_a_max_ =
            clampToByte(static_cast<int>(this->get_parameter("green_a_max").as_int()));
        green_b_min_ =
            clampToByte(static_cast<int>(this->get_parameter("green_b_min").as_int()));
        green_b_max_ =
            clampToByte(static_cast<int>(this->get_parameter("green_b_max").as_int()));
        black_enhanced_max_ =
            clampToByte(static_cast<int>(this->get_parameter("black_enhanced_max").as_int()));
        white_open_kernel_ = static_cast<int>(this->get_parameter("white_open_kernel").as_int());
        white_close_kernel_ = static_cast<int>(this->get_parameter("white_close_kernel").as_int());
        enable_image_view_ = this->get_parameter("enable_image_view").as_bool();
        show_input_image_ = this->get_parameter("show_input_image").as_bool();
        show_white_candidate_mask_ = this->get_parameter("show_white_candidate_mask").as_bool();
        show_white_morph_mask_ = this->get_parameter("show_white_morph_mask").as_bool();
        show_green_mask_ = this->get_parameter("show_green_mask").as_bool();
        show_black_mask_ = this->get_parameter("show_black_mask").as_bool();
        show_noise_mask_ = this->get_parameter("show_noise_mask").as_bool();
        show_debug_image_ = this->get_parameter("show_debug_image").as_bool();
        publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
        display_max_width_ = std::max(1, static_cast<int>(this->get_parameter("display_max_width").as_int()));
        display_max_height_ = std::max(1, static_cast<int>(this->get_parameter("display_max_height").as_int()));
    }

    void resetTimingSummary() {
        timing_frames_in_interval_ = 0;
        timing_interval_start_frame_ = 0;
        resetInterval(main_stage_interval_totals_, main_stage_interval_hits_);
        resetInterval(preprocess_stage_interval_totals_, preprocess_stage_interval_hits_);
        resetInterval(white_stage_interval_totals_, white_stage_interval_hits_);
    }

    bool anyImageWindowRequested() const {
        return show_input_image_ || show_white_candidate_mask_ || show_white_morph_mask_ ||
               show_green_mask_ || show_black_mask_ || show_noise_mask_ || show_debug_image_;
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

    void syncControlsWindow(bool should_show) {
        if (should_show && !controls_window_created_) {
            cv::namedWindow(kControlsWindow, cv::WINDOW_AUTOSIZE);

            cv::createTrackbar("white_enh_min", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("white_a_center", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("white_b_center", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("white_a_tol", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("white_b_tol", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("green_a_max", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("green_b_min", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("green_b_max", kControlsWindow, nullptr, kByteMax);
            cv::createTrackbar("black_enh_max", kControlsWindow, nullptr, kByteMax);

            cv::setTrackbarPos("white_enh_min", kControlsWindow, white_enhanced_min_);
            cv::setTrackbarPos("white_a_center", kControlsWindow, white_a_center_);
            cv::setTrackbarPos("white_b_center", kControlsWindow, white_b_center_);
            cv::setTrackbarPos("white_a_tol", kControlsWindow, white_a_tol_);
            cv::setTrackbarPos("white_b_tol", kControlsWindow, white_b_tol_);
            cv::setTrackbarPos("green_a_max", kControlsWindow, green_a_max_);
            cv::setTrackbarPos("green_b_min", kControlsWindow, green_b_min_);
            cv::setTrackbarPos("green_b_max", kControlsWindow, green_b_max_);
            cv::setTrackbarPos("black_enh_max", kControlsWindow, black_enhanced_max_);

            controls_window_created_ = true;
        } else if (!should_show && controls_window_created_) {
            cv::destroyWindow(kControlsWindow);
            controls_window_created_ = false;
        }
    }

    void destroyDebugWindows() {
        syncWindow(kOriginalWindow, false, original_window_created_);
        syncWindow(kWhiteCandidateWindow, false, white_candidate_window_created_);
        syncWindow(kWhiteMorphWindow, false, white_morph_window_created_);
        syncWindow(kGreenMaskWindow, false, green_mask_window_created_);
        syncWindow(kBlackMaskWindow, false, black_mask_window_created_);
        syncWindow(kNoiseMaskWindow, false, noise_mask_window_created_);
        syncWindow(kOverlayWindow, false, overlay_window_created_);
        syncControlsWindow(false);
    }

    void syncImageViewState() {
        syncWindow(kOriginalWindow, enable_image_view_ && show_input_image_, original_window_created_);
        syncWindow(
            kWhiteCandidateWindow,
            enable_image_view_ && show_white_candidate_mask_,
            white_candidate_window_created_);
        syncWindow(
            kWhiteMorphWindow,
            enable_image_view_ && show_white_morph_mask_,
            white_morph_window_created_);
        syncWindow(kGreenMaskWindow, enable_image_view_ && show_green_mask_, green_mask_window_created_);
        syncWindow(kBlackMaskWindow, enable_image_view_ && show_black_mask_, black_mask_window_created_);
        syncWindow(kNoiseMaskWindow, enable_image_view_ && show_noise_mask_, noise_mask_window_created_);
        syncWindow(kOverlayWindow, enable_image_view_ && show_debug_image_, overlay_window_created_);
        syncControlsWindow(enable_image_view_ && anyImageWindowRequested());
    }

    void readThresholdsFromTrackbars() {
        if (!controls_window_created_) {
            return;
        }
        white_enhanced_min_ = cv::getTrackbarPos("white_enh_min", kControlsWindow);
        white_a_center_ = cv::getTrackbarPos("white_a_center", kControlsWindow);
        white_b_center_ = cv::getTrackbarPos("white_b_center", kControlsWindow);
        white_a_tol_ = cv::getTrackbarPos("white_a_tol", kControlsWindow);
        white_b_tol_ = cv::getTrackbarPos("white_b_tol", kControlsWindow);
        green_a_max_ = cv::getTrackbarPos("green_a_max", kControlsWindow);
        green_b_min_ = cv::getTrackbarPos("green_b_min", kControlsWindow);
        green_b_max_ = cv::getTrackbarPos("green_b_max", kControlsWindow);
        black_enhanced_max_ = cv::getTrackbarPos("black_enh_max", kControlsWindow);
    }

    void logCurrentThresholds() const {
        RCLCPP_INFO(
            this->get_logger(),
            "Current Lab morph thresholds: white_enhanced_min=%d, white_a_center=%d, white_b_center=%d, white_a_tol=%d, white_b_tol=%d, green_a_max=%d, green_b_min=%d, green_b_max=%d, black_enhanced_max=%d, white_open_kernel=%d, white_close_kernel=%d",
            white_enhanced_min_,
            white_a_center_,
            white_b_center_,
            white_a_tol_,
            white_b_tol_,
            green_a_max_,
            green_b_min_,
            green_b_max_,
            black_enhanced_max_,
            white_open_kernel_,
            white_close_kernel_);
    }

    void maybeLogTimingSummary(
        const FrameTimingCapture &timing,
        const TimingSummaryContext &context,
        unsigned long long current_frame_index) {
        if (!enable_timing_debug_) {
            return;
        }

        if (timing_frames_in_interval_ == 0U) {
            timing_interval_start_frame_ = current_frame_index;
        }

        accumulateInterval(timing.main_stage_us, main_stage_interval_totals_, main_stage_interval_hits_);
        accumulateInterval(
            timing.preprocess_timing.stage_us,
            preprocess_stage_interval_totals_,
            preprocess_stage_interval_hits_);
        accumulateInterval(
            timing.white_candidate_timing.stage_us,
            white_stage_interval_totals_,
            white_stage_interval_hits_);
        ++timing_frames_in_interval_;

        if (timing_frames_in_interval_ < static_cast<std::size_t>(timing_summary_interval_)) {
            return;
        }

        const double callback_average_ms =
            static_cast<double>(
                main_stage_interval_totals_[static_cast<std::size_t>(MainTimingStage::CallbackTotal)]) /
            static_cast<double>(timing_frames_in_interval_) / 1000.0;

        std::ostringstream oss;
        oss << "\n================ Morph Profiling 摘要 ================\n";
        oss << "帧区间: " << timing_interval_start_frame_ << " - " << current_frame_index
            << " (" << timing_frames_in_interval_ << " 帧)\n";
        oss << "图像尺寸: " << context.image_width << "x" << context.image_height
            << "  encoding=" << context.encoding << '\n';
        oss << "开关状态: 时序调试=" << toToggleText(enable_timing_debug_)
            << "  图像窗口=" << toToggleText(context.image_view_enabled)
            << "  发布debug_image=" << toToggleText(context.publish_debug_image)
            << "  overlay窗口请求=" << toToggleText(context.overlay_requested_for_window)
            << "  overlay发布请求=" << toToggleText(context.overlay_requested_for_publish) << '\n';
        oss << "订阅数量: white_candidate=" << context.white_candidate_subscribers
            << "  white_morph=" << context.white_morph_subscribers
            << "  green=" << context.green_subscribers
            << "  black=" << context.black_subscribers
            << "  noise=" << context.noise_subscribers
            << "  debug=" << context.debug_subscribers << '\n';

        appendTimingTable(
            oss,
            "\n[主流程阶段]",
            kMainTimingLabels,
            timing.main_stage_us,
            main_stage_interval_totals_,
            main_stage_interval_hits_,
            timing_frames_in_interval_,
            callback_average_ms,
            static_cast<std::size_t>(MainTimingStage::CallbackTotal));
        appendTimingTable(
            oss,
            "\n[预处理子阶段]",
            kPreprocessTimingLabels,
            timing.preprocess_timing.stage_us,
            preprocess_stage_interval_totals_,
            preprocess_stage_interval_hits_,
            timing_frames_in_interval_,
            callback_average_ms);
        appendTimingTable(
            oss,
            "\n[白色候选子阶段]",
            kWhiteCandidateTimingLabels,
            timing.white_candidate_timing.stage_us,
            white_stage_interval_totals_,
            white_stage_interval_hits_,
            timing_frames_in_interval_,
            callback_average_ms);
        oss << "=====================================================";

        RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
        resetTimingSummary();
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        const bool previous_timing_debug = enable_timing_debug_;
        const int previous_timing_summary_interval = timing_summary_interval_;

        loadTimingParameters();
        if (previous_timing_debug != enable_timing_debug_ ||
            previous_timing_summary_interval != timing_summary_interval_) {
            resetTimingSummary();
        }

        FrameTimingCapture timing;
        const bool timing_enabled = enable_timing_debug_;
        const auto callback_start = timing_enabled ? SteadyClock::now() : TimePoint{};

        auto stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        loadProcessingParameters();
        syncImageViewState();
        if (controls_window_created_) {
            readThresholdsFromTrackbars();
        }
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::RuntimeSync,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat frame;
        try {
            stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            if (timing_enabled) {
                recordStageDuration(
                    timing.main_stage_us,
                    MainTimingStage::RosToCv,
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

        if (!validateRobotMaskForFrame(frame)) {
            return;
        }

        const unsigned long long current_frame_index = ++frame_count_;
        const auto preprocess_params = rcj_loc::vision::white_line::getPreprocessParams(*this);
        rcj_loc::vision::white_line::PreprocessTiming *preprocess_timing_ptr =
            timing_enabled ? &timing.preprocess_timing : nullptr;

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const auto preprocessed = rcj_loc::vision::white_line::preprocessFrame(
            frame,
            preprocess_params,
            preprocess_timing_ptr);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::Preprocess,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat green_candidate;
        const int green_b_low = std::min(green_b_min_, green_b_max_);
        const int green_b_high = std::max(green_b_min_, green_b_max_);
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::Mat green_a_mask;
        cv::Mat green_b_mask;
        cv::inRange(preprocessed.lab_a, cv::Scalar(0), cv::Scalar(green_a_max_), green_a_mask);
        cv::inRange(preprocessed.lab_b, cv::Scalar(green_b_low), cv::Scalar(green_b_high), green_b_mask);
        cv::bitwise_and(green_a_mask, green_b_mask, green_candidate);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::GreenCandidate,
                stage_start,
                SteadyClock::now());
        }

        WhiteCandidateTiming *white_timing_ptr =
            timing_enabled ? &timing.white_candidate_timing : nullptr;
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const cv::Mat white_candidate = makeWhiteCandidate(
            preprocessed.enhanced,
            preprocessed.lab_a,
            preprocessed.lab_b,
            white_enhanced_min_,
            white_a_center_,
            white_b_center_,
            white_a_tol_,
            white_b_tol_,
            white_timing_ptr);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::WhiteCandidate,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat black_candidate;
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::inRange(
            preprocessed.enhanced,
            cv::Scalar(0),
            cv::Scalar(black_enhanced_max_),
            black_candidate);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::BlackCandidate,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::Mat remaining =
            robot_mask_enabled_
                ? robot_allowed_mask_.clone()
                : cv::Mat(frame.size(), CV_8UC1, cv::Scalar(255));
        const cv::Mat green_mask = takeFromRemaining(green_candidate, remaining);
        const cv::Mat raw_white_mask = takeFromRemaining(white_candidate, remaining);
        const cv::Mat black_mask = takeFromRemaining(black_candidate, remaining);
        const cv::Mat noise_mask = remaining;
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::RemainingPartition,
                stage_start,
                SteadyClock::now());
        }

        const int white_open_kernel = rcj_loc::vision::debug::makeOdd(white_open_kernel_, 1);
        const int white_close_kernel = rcj_loc::vision::debug::makeOdd(white_close_kernel_, 1);
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        const cv::Mat open_element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(white_open_kernel, white_open_kernel));
        const cv::Mat close_element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(white_close_kernel, white_close_kernel));
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::KernelCreate,
                stage_start,
                SteadyClock::now());
        }

        cv::Mat white_morph_mask = raw_white_mask.clone();
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::morphologyEx(white_morph_mask, white_morph_mask, cv::MORPH_OPEN, open_element);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::MorphOpen,
                stage_start,
                SteadyClock::now());
        }

        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        cv::morphologyEx(white_morph_mask, white_morph_mask, cv::MORPH_CLOSE, close_element);
        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::MorphClose,
                stage_start,
                SteadyClock::now());
        }

        const bool overlay_needed_for_window = overlay_window_created_;
        const bool overlay_needed_for_publish =
            enable_image_view_ && publish_debug_image_ && hasSubscribers(debug_pub_);
        const bool should_build_overlay = overlay_needed_for_window || overlay_needed_for_publish;

        cv::Mat overlay;
        if (should_build_overlay) {
            stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
            overlay = preprocessed.filtered_bgr.clone();
            overlay =
                rcj_loc::vision::debug::createMaskOverlay(overlay, black_mask, cv::Scalar(0, 0, 255));
            overlay =
                rcj_loc::vision::debug::createMaskOverlay(overlay, green_mask, cv::Scalar(0, 255, 0));
            overlay =
                rcj_loc::vision::debug::createMaskOverlay(overlay, noise_mask, cv::Scalar(255, 0, 255));
            overlay = rcj_loc::vision::debug::createMaskOverlay(
                overlay,
                white_morph_mask,
                cv::Scalar(255, 255, 255));
            if (timing_enabled) {
                recordStageDuration(
                    timing.main_stage_us,
                    MainTimingStage::Overlay,
                    stage_start,
                    SteadyClock::now());
            }
        }

        bool published_any = false;
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        if (enable_image_view_) {
            published_any |= publishImageIfSubscribed(
                white_candidate_mask_pub_,
                msg->header,
                "mono8",
                raw_white_mask);
        }
        published_any |= publishImageIfSubscribed(
            white_morph_mask_pub_,
            msg->header,
            "mono8",
            white_morph_mask);
        published_any |= publishImageIfSubscribed(green_mask_pub_, msg->header, "mono8", green_mask);
        published_any |= publishImageIfSubscribed(black_mask_pub_, msg->header, "mono8", black_mask);
        published_any |= publishImageIfSubscribed(noise_mask_pub_, msg->header, "mono8", noise_mask);
        if (overlay_needed_for_publish && !overlay.empty()) {
            published_any |= publishImageIfSubscribed(debug_pub_, msg->header, "bgr8", overlay);
        }
        if (timing_enabled && published_any) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::Publish,
                stage_start,
                SteadyClock::now());
        }

        bool displayed_any_window = false;
        stage_start = timing_enabled ? SteadyClock::now() : TimePoint{};
        if (original_window_created_) {
            cv::imshow(kOriginalWindow, frame);
            resizeWindowToFitImage(kOriginalWindow, frame, display_max_width_, display_max_height_);
            displayed_any_window = true;
        }
        if (white_candidate_window_created_) {
            cv::imshow(kWhiteCandidateWindow, raw_white_mask);
            resizeWindowToFitImage(
                kWhiteCandidateWindow,
                raw_white_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (white_morph_window_created_) {
            cv::imshow(kWhiteMorphWindow, white_morph_mask);
            resizeWindowToFitImage(
                kWhiteMorphWindow,
                white_morph_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (green_mask_window_created_) {
            cv::imshow(kGreenMaskWindow, green_mask);
            resizeWindowToFitImage(
                kGreenMaskWindow,
                green_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (black_mask_window_created_) {
            cv::imshow(kBlackMaskWindow, black_mask);
            resizeWindowToFitImage(
                kBlackMaskWindow,
                black_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (noise_mask_window_created_) {
            cv::imshow(kNoiseMaskWindow, noise_mask);
            resizeWindowToFitImage(
                kNoiseMaskWindow,
                noise_mask,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (overlay_window_created_ && !overlay.empty()) {
            cv::imshow(kOverlayWindow, overlay);
            resizeWindowToFitImage(
                kOverlayWindow,
                overlay,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }

        if (displayed_any_window || controls_window_created_) {
            const int key = cv::waitKey(1);
            if (key == 'p' || key == 'P') {
                logCurrentThresholds();
            }
        }
        if (timing_enabled && (displayed_any_window || controls_window_created_)) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::Gui,
                stage_start,
                SteadyClock::now());
        }

        if (timing_enabled) {
            recordStageDuration(
                timing.main_stage_us,
                MainTimingStage::CallbackTotal,
                callback_start,
                SteadyClock::now());
            const long long accounted_stage_us =
                sumMainTimingStagesExcludingCallback(timing.main_stage_us);
            timing.main_stage_us[static_cast<std::size_t>(MainTimingStage::UnaccountedOverhead)] =
                std::max(
                    0LL,
                    timing.main_stage_us[static_cast<std::size_t>(MainTimingStage::CallbackTotal)] -
                        accounted_stage_us);
        }

        TimingSummaryContext summary_context;
        summary_context.image_width = frame.cols;
        summary_context.image_height = frame.rows;
        summary_context.encoding = msg->encoding;
        summary_context.white_candidate_subscribers = subscriptionCount(white_candidate_mask_pub_);
        summary_context.white_morph_subscribers = subscriptionCount(white_morph_mask_pub_);
        summary_context.green_subscribers = subscriptionCount(green_mask_pub_);
        summary_context.black_subscribers = subscriptionCount(black_mask_pub_);
        summary_context.noise_subscribers = subscriptionCount(noise_mask_pub_);
        summary_context.debug_subscribers = subscriptionCount(debug_pub_);
        summary_context.image_view_enabled = enable_image_view_;
        summary_context.publish_debug_image = publish_debug_image_;
        summary_context.overlay_requested_for_window = overlay_needed_for_window;
        summary_context.overlay_requested_for_publish = overlay_needed_for_publish;
        maybeLogTimingSummary(timing, summary_context, current_frame_index);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhiteLineLabMorphNode>());
    rclcpp::shutdown();
    return 0;
}
