#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr char kInputWindowName[] = "Black Feature Detector Input";
constexpr char kDebugWindowName[] = "Black Feature Detector Debug";
constexpr double kPi = 3.14159265358979323846;

struct CircleArcRecord {
    cv::Point2f center;
    float radius = 0.0f;
    double start_deg = 0.0;
    double end_deg = 0.0;
};

struct CircleArcCandidate {
    cv::Mat mask;
    double start_deg = 0.0;
    double end_deg = 0.0;
};

struct CircleEvaluationResult {
    bool accepted = false;
    cv::Point2f center;
    float radius = 0.0f;
    std::vector<CircleArcCandidate> arcs;
};

struct BinSegment {
    int start = 0;
    int end = 0;
};

int makeOddKernel(int value, int minimum_value = 1) {
    const int clamped = std::max(minimum_value, value);
    return clamped % 2 == 0 ? clamped + 1 : clamped;
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

template <typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> &publisher) {
    return publisher != nullptr && publisher->get_subscription_count() > 0U;
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

double normalizeAngleDeg(double angle_deg) {
    double normalized = std::fmod(angle_deg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

bool angleInRange(double angle_deg, double start_deg, double end_deg) {
    const double angle = normalizeAngleDeg(angle_deg);
    const double start = normalizeAngleDeg(start_deg);
    const double end = normalizeAngleDeg(end_deg);

    if (std::abs(start - end) < 1e-6) {
        return true;
    }
    if (start < end) {
        return angle >= start && angle < end;
    }
    return angle >= start || angle < end;
}

std::vector<BinSegment> collectOccupiedSegments(const std::vector<int> &bin_counts) {
    std::vector<BinSegment> segments;
    const int total_bins = static_cast<int>(bin_counts.size());
    int idx = 0;
    while (idx < total_bins) {
        while (idx < total_bins && bin_counts[idx] == 0) {
            ++idx;
        }
        if (idx >= total_bins) {
            break;
        }
        const int start = idx;
        while (idx < total_bins && bin_counts[idx] > 0) {
            ++idx;
        }
        segments.push_back({start, idx - 1});
    }

    if (segments.size() > 1 && bin_counts.front() > 0 && bin_counts.back() > 0) {
        const BinSegment first = segments.front();
        const BinSegment last = segments.back();
        segments.front() = {last.start, first.end + total_bins};
        segments.pop_back();
    }

    return segments;
}

bool contourTouchesBorder(const std::vector<cv::Point> &contour, const cv::Size &image_size) {
    for (const auto &point : contour) {
        if (point.x <= 0 || point.y <= 0 || point.x >= image_size.width - 1 ||
            point.y >= image_size.height - 1) {
            return true;
        }
    }
    return false;
}

double contourCircularity(const std::vector<cv::Point> &contour, double area) {
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 1e-6) {
        return 0.0;
    }
    return (4.0 * kPi * area) / (perimeter * perimeter);
}

double contourSolidity(const std::vector<cv::Point> &contour, double area) {
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    const double hull_area = cv::contourArea(hull);
    if (hull_area <= 1e-6) {
        return 0.0;
    }
    return area / hull_area;
}

double contourAspectRatio(const std::vector<cv::Point> &contour) {
    const cv::Rect bbox = cv::boundingRect(contour);
    const int min_side = std::max(1, std::min(bbox.width, bbox.height));
    const int max_side = std::max(bbox.width, bbox.height);
    return static_cast<double>(max_side) / static_cast<double>(min_side);
}

cv::Mat createMaskOverlay(
    const cv::Mat &image,
    const cv::Mat &mask,
    const cv::Scalar &color,
    double alpha) {
    cv::Mat overlay = image.clone();
    cv::Mat color_image(image.size(), CV_8UC3, color);
    cv::Mat blended;
    cv::addWeighted(image, 1.0 - alpha, color_image, alpha, 0.0, blended);
    blended.copyTo(overlay, mask);
    return overlay;
}

void addPanelLabel(cv::Mat &panel, const std::string &label) {
    cv::rectangle(panel, cv::Rect(0, 0, std::min(panel.cols, 260), 34), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
        panel,
        label,
        cv::Point(10, 23),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);
}

cv::Mat removeBorderTouchingComponents(const cv::Mat &mask) {
    cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
        if (contourTouchesBorder(contour, mask.size())) {
            continue;
        }
        cv::drawContours(filtered, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
    }
    return filtered;
}

}  // namespace

class BlackFeatureDetectorNode : public rclcpp::Node {
public:
    BlackFeatureDetectorNode() : Node("black_feature_detector_node") {
        this->declare_parameter<std::string>(
            "input_topic", "/black_feature_input_remap_node/image_remapped");
        this->declare_parameter<std::string>("robot_mask_topic", "");
        this->declare_parameter("enable_image_view", false);
        this->declare_parameter("show_input_image", false);
        this->declare_parameter("show_debug_image", true);
        this->declare_parameter("display_max_width", 960);
        this->declare_parameter("display_max_height", 720);

        this->declare_parameter("use_clahe", false);
        this->declare_parameter("clahe_clip_limit", 2.5);
        this->declare_parameter("blur_kernel", 5);

        this->declare_parameter("black_l_max", 95);
        this->declare_parameter("use_v_aux_gate", false);
        this->declare_parameter("black_v_max", 80);

        this->declare_parameter("open_kernel", 3);
        this->declare_parameter("close_kernel", 5);

        this->declare_parameter("dot_min_area_px", 8);
        this->declare_parameter("dot_max_area_px", 220);
        this->declare_parameter("dot_min_circularity", 0.45);
        this->declare_parameter("dot_min_solidity", 0.75);

        this->declare_parameter("circle_min_radius_px", 10.0);
        this->declare_parameter("circle_max_radius_px", 140.0);
        this->declare_parameter("circle_ring_width_px", 8.0);
        this->declare_parameter("circle_inner_guard_px", 8.0);
        this->declare_parameter("circle_outer_guard_px", 8.0);
        this->declare_parameter("circle_min_ring_black_ratio", 0.18);
        this->declare_parameter("circle_max_inner_black_ratio", 0.08);
        this->declare_parameter("circle_max_outer_black_ratio", 0.12);
        this->declare_parameter("circle_max_radial_residual_px", 5.5);
        this->declare_parameter("circle_min_arc_span_deg", 45.0);
        this->declare_parameter("circle_angle_bin_deg", 6.0);

        syncImageViewState();

        const auto input_topic = this->get_parameter("input_topic").as_string();
        const auto robot_mask_topic = this->get_parameter("robot_mask_topic").as_string();
        robot_mask_enabled_ = !robot_mask_topic.empty();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&BlackFeatureDetectorNode::imageCallback, this, std::placeholders::_1));
        if (robot_mask_enabled_) {
            robot_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                robot_mask_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&BlackFeatureDetectorNode::robotMaskCallback, this, std::placeholders::_1));
        }

        black_final_mask_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/black_final_mask", 10);
        debug_image_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "black_feature_detector_node started. input_topic='%s', robot_mask_topic='%s', "
            "use_v_aux_gate=%s, "
            "show_debug_image=%s",
            input_topic.c_str(),
            robot_mask_topic.empty() ? "<disabled>" : robot_mask_topic.c_str(),
            this->get_parameter("use_v_aux_gate").as_bool() ? "true" : "false",
            show_debug_image_ ? "true" : "false");
    }

    ~BlackFeatureDetectorNode() override {
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
        syncWindow(kInputWindowName, false, input_window_created_);
        syncWindow(kDebugWindowName, false, debug_window_created_);
    }

    void syncImageViewState() {
        enable_image_view_ = this->get_parameter("enable_image_view").as_bool();
        show_input_image_ = this->get_parameter("show_input_image").as_bool();
        show_debug_image_ = this->get_parameter("show_debug_image").as_bool();
        display_max_width_ =
            std::max(1, static_cast<int>(this->get_parameter("display_max_width").as_int()));
        display_max_height_ =
            std::max(1, static_cast<int>(this->get_parameter("display_max_height").as_int()));

        const bool display_available =
            std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
        if (enable_image_view_ && !display_available) {
            if (!headless_warned_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "enable_image_view=true but no DISPLAY/WAYLAND_DISPLAY is available; "
                    "disabling OpenCV image view for this process.");
                headless_warned_ = true;
            }
            enable_image_view_ = false;
        }

        syncWindow(kInputWindowName, enable_image_view_ && show_input_image_, input_window_created_);
        syncWindow(kDebugWindowName, enable_image_view_ && show_debug_image_, debug_window_created_);
    }

    cv::Mat preprocessLuminance(const cv::Mat &lab_l) const {
        cv::Mat processed = lab_l.clone();
        if (this->get_parameter("use_clahe").as_bool()) {
            const double clip_limit =
                std::max(0.1, this->get_parameter("clahe_clip_limit").as_double());
            auto clahe = cv::createCLAHE(clip_limit, cv::Size(8, 8));
            clahe->apply(processed, processed);
        }

        const int blur_kernel =
            makeOddKernel(static_cast<int>(this->get_parameter("blur_kernel").as_int()), 1);
        if (blur_kernel > 1) {
            cv::GaussianBlur(
                processed,
                processed,
                cv::Size(blur_kernel, blur_kernel),
                0.0,
                0.0,
                cv::BORDER_REPLICATE);
        }
        return processed;
    }

    cv::Mat buildBlackCandidateMask(const cv::Mat &lab_l, const cv::Mat &hsv_v) const {
        cv::Mat black_candidate;
        cv::inRange(
            lab_l,
            cv::Scalar(0),
            cv::Scalar(this->get_parameter("black_l_max").as_int()),
            black_candidate);

        if (this->get_parameter("use_v_aux_gate").as_bool()) {
            cv::Mat value_mask;
            cv::inRange(
                hsv_v,
                cv::Scalar(0),
                cv::Scalar(this->get_parameter("black_v_max").as_int()),
                value_mask);
            cv::bitwise_and(black_candidate, value_mask, black_candidate);
        }

        const int open_kernel =
            makeOddKernel(static_cast<int>(this->get_parameter("open_kernel").as_int()), 1);
        const int close_kernel =
            makeOddKernel(static_cast<int>(this->get_parameter("close_kernel").as_int()), 1);

        if (open_kernel > 1) {
            const cv::Mat open_element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(open_kernel, open_kernel));
            cv::morphologyEx(black_candidate, black_candidate, cv::MORPH_OPEN, open_element);
        }

        if (close_kernel > 1) {
            const cv::Mat close_element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(close_kernel, close_kernel));
            cv::morphologyEx(black_candidate, black_candidate, cv::MORPH_CLOSE, close_element);
        }

        return black_candidate;
    }

    bool isDotCandidate(const std::vector<cv::Point> &contour) const {
        const double area = cv::contourArea(contour);
        const int min_area = static_cast<int>(this->get_parameter("dot_min_area_px").as_int());
        const int max_area = static_cast<int>(this->get_parameter("dot_max_area_px").as_int());
        if (area < static_cast<double>(min_area) || area > static_cast<double>(max_area)) {
            return false;
        }

        const double circularity = contourCircularity(contour, area);
        const double solidity = contourSolidity(contour, area);
        const double aspect_ratio = contourAspectRatio(contour);

        return circularity >= this->get_parameter("dot_min_circularity").as_double() &&
               solidity >= this->get_parameter("dot_min_solidity").as_double() &&
               aspect_ratio <= 1.8;
    }

    CircleEvaluationResult evaluateCircleContour(
        const std::vector<cv::Point> &contour,
        const cv::Mat &component_mask,
        const cv::Mat &clean_mask) const {
        CircleEvaluationResult result;
        cv::minEnclosingCircle(contour, result.center, result.radius);

        const double min_radius = this->get_parameter("circle_min_radius_px").as_double();
        const double max_radius = this->get_parameter("circle_max_radius_px").as_double();
        if (result.radius < min_radius || result.radius > max_radius) {
            return result;
        }

        std::vector<cv::Point> component_points;
        cv::findNonZero(component_mask, component_points);
        if (component_points.size() < 10U) {
            return result;
        }

        std::vector<float> radial_residuals;
        radial_residuals.reserve(component_points.size());
        const double ring_half_width =
            0.5 * std::max(1.0, this->get_parameter("circle_ring_width_px").as_double());
        const double residual_limit =
            std::max(0.5, this->get_parameter("circle_max_radial_residual_px").as_double());
        const double angle_bin_deg =
            std::clamp(this->get_parameter("circle_angle_bin_deg").as_double(), 1.0, 45.0);
        const int total_bins =
            std::max(8, static_cast<int>(std::ceil(360.0 / angle_bin_deg)));
        std::vector<int> bin_counts(static_cast<std::size_t>(total_bins), 0);

        for (const auto &point : component_points) {
            const double dx = static_cast<double>(point.x) - static_cast<double>(result.center.x);
            const double dy = static_cast<double>(point.y) - static_cast<double>(result.center.y);
            const double distance = std::hypot(dx, dy);
            const float residual =
                static_cast<float>(std::abs(distance - static_cast<double>(result.radius)));
            radial_residuals.push_back(residual);

            if (residual <= ring_half_width + 1.0) {
                const double angle_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / kPi);
                int bin_idx = static_cast<int>(std::floor(angle_deg / angle_bin_deg));
                bin_idx = std::clamp(bin_idx, 0, total_bins - 1);
                ++bin_counts[static_cast<std::size_t>(bin_idx)];
            }
        }

        if (computeMedian(radial_residuals) > residual_limit) {
            return result;
        }

        const double min_arc_span_deg =
            std::clamp(this->get_parameter("circle_min_arc_span_deg").as_double(), angle_bin_deg, 360.0);
        const double inner_guard =
            std::max(0.0, this->get_parameter("circle_inner_guard_px").as_double());
        const double outer_guard =
            std::max(0.0, this->get_parameter("circle_outer_guard_px").as_double());
        const double min_ring_ratio =
            std::clamp(this->get_parameter("circle_min_ring_black_ratio").as_double(), 0.0, 1.0);
        const double max_inner_ratio =
            std::clamp(this->get_parameter("circle_max_inner_black_ratio").as_double(), 0.0, 1.0);
        const double max_outer_ratio =
            std::clamp(this->get_parameter("circle_max_outer_black_ratio").as_double(), 0.0, 1.0);

        const auto segments = collectOccupiedSegments(bin_counts);
        if (segments.empty()) {
            return result;
        }

        const double roi_radius =
            static_cast<double>(result.radius) + ring_half_width + std::max(inner_guard, outer_guard) + 3.0;
        const int x_min = std::max(
            0,
            static_cast<int>(std::floor(static_cast<double>(result.center.x) - roi_radius)));
        const int x_max = std::min(
            component_mask.cols - 1,
            static_cast<int>(std::ceil(static_cast<double>(result.center.x) + roi_radius)));
        const int y_min = std::max(
            0,
            static_cast<int>(std::floor(static_cast<double>(result.center.y) - roi_radius)));
        const int y_max = std::min(
            component_mask.rows - 1,
            static_cast<int>(std::ceil(static_cast<double>(result.center.y) + roi_radius)));

        for (const auto &segment : segments) {
            const double segment_start_deg = static_cast<double>(segment.start) * angle_bin_deg;
            const double segment_end_deg = static_cast<double>(segment.end + 1) * angle_bin_deg;
            const double segment_span_deg =
                static_cast<double>(segment.end - segment.start + 1) * angle_bin_deg;
            if (segment_span_deg < min_arc_span_deg) {
                continue;
            }

            int ring_total = 0;
            int ring_black = 0;
            int inner_total = 0;
            int inner_black = 0;
            int outer_total = 0;
            int outer_black = 0;
            cv::Mat arc_mask = cv::Mat::zeros(component_mask.size(), CV_8UC1);

            for (int y = y_min; y <= y_max; ++y) {
                for (int x = x_min; x <= x_max; ++x) {
                    const double dx =
                        static_cast<double>(x) - static_cast<double>(result.center.x);
                    const double dy =
                        static_cast<double>(y) - static_cast<double>(result.center.y);
                    const double distance = std::hypot(dx, dy);
                    const double angle_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / kPi);
                    if (!angleInRange(angle_deg, segment_start_deg, segment_end_deg)) {
                        continue;
                    }

                    const double radial_delta =
                        std::abs(distance - static_cast<double>(result.radius));
                    if (radial_delta <= ring_half_width) {
                        ++ring_total;
                        if (component_mask.at<uchar>(y, x) != 0) {
                            ++ring_black;
                        }
                    } else if (distance >= static_cast<double>(result.radius) - ring_half_width - inner_guard &&
                               distance < static_cast<double>(result.radius) - ring_half_width) {
                        ++inner_total;
                        if (clean_mask.at<uchar>(y, x) != 0) {
                            ++inner_black;
                        }
                    } else if (distance > static_cast<double>(result.radius) + ring_half_width &&
                               distance <= static_cast<double>(result.radius) + ring_half_width + outer_guard) {
                        ++outer_total;
                        if (clean_mask.at<uchar>(y, x) != 0) {
                            ++outer_black;
                        }
                    }

                    if (component_mask.at<uchar>(y, x) != 0 &&
                        radial_delta <= ring_half_width + residual_limit) {
                        arc_mask.at<uchar>(y, x) = 255;
                    }
                }
            }

            if (ring_total == 0 || cv::countNonZero(arc_mask) == 0) {
                continue;
            }

            const double ring_ratio =
                static_cast<double>(ring_black) / static_cast<double>(ring_total);
            const double inner_ratio =
                inner_total > 0 ? static_cast<double>(inner_black) / static_cast<double>(inner_total) : 0.0;
            const double outer_ratio =
                outer_total > 0 ? static_cast<double>(outer_black) / static_cast<double>(outer_total) : 0.0;

            if (ring_ratio < min_ring_ratio || inner_ratio > max_inner_ratio ||
                outer_ratio > max_outer_ratio) {
                continue;
            }

            result.arcs.push_back({arc_mask, segment_start_deg, segment_end_deg});
        }

        result.accepted = !result.arcs.empty();
        return result;
    }

    cv::Mat buildDebugComposite(
        const cv::Mat &frame,
        const cv::Mat &black_candidate_mask,
        const cv::Mat &clean_mask,
        const cv::Mat &dot_mask,
        const cv::Mat &arc_mask,
        const cv::Mat &black_final_mask,
        const std::vector<std::vector<cv::Point>> &rejected_contours,
        const std::vector<CircleArcRecord> &accepted_arcs) const {
        cv::Mat panel_input = frame.clone();
        cv::Mat panel_candidate = createMaskOverlay(frame, black_candidate_mask, cv::Scalar(0, 0, 255), 0.55);
        cv::Mat panel_clean = createMaskOverlay(frame, clean_mask, cv::Scalar(255, 200, 0), 0.45);
        cv::Mat panel_final = frame.clone();
        panel_final = createMaskOverlay(panel_final, dot_mask, cv::Scalar(0, 220, 0), 0.65);
        panel_final = createMaskOverlay(panel_final, arc_mask, cv::Scalar(0, 220, 255), 0.65);

        if (!rejected_contours.empty()) {
            cv::drawContours(
                panel_clean,
                rejected_contours,
                -1,
                cv::Scalar(0, 0, 255),
                2,
                cv::LINE_AA);
        }

        std::vector<std::vector<cv::Point>> dot_contours;
        if (cv::countNonZero(dot_mask) > 0) {
            cv::findContours(dot_mask.clone(), dot_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::drawContours(
                panel_final,
                dot_contours,
                -1,
                cv::Scalar(0, 120, 0),
                2,
                cv::LINE_AA);
        }

        for (const auto &arc : accepted_arcs) {
            cv::circle(
                panel_final,
                cv::Point(
                    static_cast<int>(std::round(arc.center.x)),
                    static_cast<int>(std::round(arc.center.y))),
                static_cast<int>(std::round(arc.radius)),
                cv::Scalar(255, 0, 0),
                1,
                cv::LINE_AA);
        }

        panel_clean = createMaskOverlay(panel_clean, black_final_mask, cv::Scalar(0, 255, 0), 0.35);

        addPanelLabel(panel_input, "Input");
        addPanelLabel(panel_candidate, "Black Candidate");
        addPanelLabel(panel_clean, "Clean + Rejected");
        addPanelLabel(panel_final, "Final Mask + Circle Fit");

        cv::Mat top_row;
        cv::Mat bottom_row;
        cv::Mat debug_image;
        cv::hconcat(std::vector<cv::Mat>{panel_input, panel_candidate}, top_row);
        cv::hconcat(std::vector<cv::Mat>{panel_clean, panel_final}, bottom_row);
        cv::vconcat(std::vector<cv::Mat>{top_row, bottom_row}, debug_image);
        return debug_image;
    }

    void publishAndDisplay(
        const std_msgs::msg::Header &header,
        const cv::Mat &frame,
        const cv::Mat &black_final_mask,
        const cv::Mat &debug_image,
        bool publish_black_final_mask,
        bool publish_debug_image) {
        if (publish_black_final_mask) {
            black_final_mask_pub_->publish(
                *cv_bridge::CvImage(header, "mono8", black_final_mask).toImageMsg());
        }
        if (publish_debug_image && !debug_image.empty()) {
            debug_image_pub_->publish(*cv_bridge::CvImage(header, "bgr8", debug_image).toImageMsg());
        }

        bool displayed_any_window = false;
        if (input_window_created_) {
            cv::imshow(kInputWindowName, frame);
            resizeWindowToFitImage(kInputWindowName, frame, display_max_width_, display_max_height_);
            displayed_any_window = true;
        }
        if (debug_window_created_ && !debug_image.empty()) {
            cv::imshow(kDebugWindowName, debug_image);
            resizeWindowToFitImage(
                kDebugWindowName,
                debug_image,
                display_max_width_,
                display_max_height_);
            displayed_any_window = true;
        }
        if (displayed_any_window) {
            cv::waitKey(1);
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        syncImageViewState();

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "cv_bridge failed while reading black feature input: %s",
                e.what());
            return;
        }

        if (!validateRobotMaskForFrame(frame)) {
            return;
        }

        cv::Mat lab_image;
        cv::Mat hsv_image;
        cv::cvtColor(frame, lab_image, cv::COLOR_BGR2Lab);
        cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> lab_channels;
        std::vector<cv::Mat> hsv_channels;
        cv::split(lab_image, lab_channels);
        cv::split(hsv_image, hsv_channels);

        const cv::Mat processed_l = preprocessLuminance(lab_channels[0]);
        cv::Mat black_candidate_mask = buildBlackCandidateMask(processed_l, hsv_channels[2]);
        if (robot_mask_enabled_) {
            cv::bitwise_and(black_candidate_mask, robot_allowed_mask_, black_candidate_mask);
        }
        const cv::Mat clean_mask = removeBorderTouchingComponents(black_candidate_mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(clean_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat dot_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::Mat arc_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> rejected_contours;
        std::vector<CircleArcRecord> accepted_arcs;

        for (const auto &contour : contours) {
            if (contour.empty()) {
                continue;
            }

            if (isDotCandidate(contour)) {
                cv::drawContours(
                    dot_mask,
                    std::vector<std::vector<cv::Point>>{contour},
                    -1,
                    cv::Scalar(255),
                    cv::FILLED);
                continue;
            }

            cv::Mat component_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
            cv::drawContours(
                component_mask,
                std::vector<std::vector<cv::Point>>{contour},
                -1,
                cv::Scalar(255),
                cv::FILLED);

            const CircleEvaluationResult circle_eval =
                evaluateCircleContour(contour, component_mask, clean_mask);
            if (!circle_eval.accepted) {
                rejected_contours.push_back(contour);
                continue;
            }

            for (const auto &arc : circle_eval.arcs) {
                cv::bitwise_or(arc_mask, arc.mask, arc_mask);
                accepted_arcs.push_back(
                    {circle_eval.center, circle_eval.radius, arc.start_deg, arc.end_deg});
            }
        }

        cv::Mat black_final_mask;
        cv::bitwise_or(dot_mask, arc_mask, black_final_mask);
        if (robot_mask_enabled_) {
            cv::bitwise_and(black_final_mask, robot_allowed_mask_, black_final_mask);
        }

        const bool debug_outputs_enabled = enable_image_view_;
        const bool publish_black_final_mask =
            debug_outputs_enabled && hasSubscribers(black_final_mask_pub_);
        const bool publish_debug_image =
            debug_outputs_enabled && hasSubscribers(debug_image_pub_);
        const bool debug_image_needed = publish_debug_image || debug_window_created_;

        cv::Mat debug_image;
        if (debug_image_needed) {
            debug_image = buildDebugComposite(
                frame,
                black_candidate_mask,
                clean_mask,
                dot_mask,
                arc_mask,
                black_final_mask,
                rejected_contours,
                accepted_arcs);
        }

        publishAndDisplay(
            msg->header,
            frame,
            black_final_mask,
            debug_image,
            publish_black_final_mask,
            publish_debug_image);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_mask_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr black_final_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    bool robot_mask_enabled_ = false;
    bool robot_mask_received_ = false;
    bool robot_mask_validated_ = false;
    cv::Mat robot_allowed_mask_;
    bool enable_image_view_ = false;
    bool show_input_image_ = false;
    bool show_debug_image_ = true;
    bool input_window_created_ = false;
    bool debug_window_created_ = false;
    bool headless_warned_ = false;
    int display_max_width_ = 960;
    int display_max_height_ = 720;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlackFeatureDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
