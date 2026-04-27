#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found"
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace {

constexpr char kInputWindowName[] = "FastMap Remap Input";
constexpr char kOutputWindowName[] = "FastMap Remap Output";

cv::Size fitWithinBounds(const cv::Size& image_size, int max_width, int max_height)
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
    const std::string& window_name,
    const cv::Mat& image,
    int max_width,
    int max_height)
{
    const cv::Size fitted_size = fitWithinBounds(image.size(), max_width, max_height);
    cv::resizeWindow(window_name, fitted_size.width, fitted_size.height);
}

std::filesystem::path resolvePath(const std::string& raw_path)
{
    if (raw_path.empty()) {
        return {};
    }

    std::filesystem::path resolved_path;
    if (raw_path.front() == '~') {
        const char* home = std::getenv("HOME");
        if (home == nullptr || std::string(home).empty()) {
            throw std::runtime_error("HOME is not set; cannot resolve '~' in robot_mask_path");
        }

        if (raw_path.size() == 1) {
            resolved_path = std::filesystem::path(home);
        } else if (raw_path[1] == '/') {
            resolved_path = std::filesystem::path(home) / raw_path.substr(2);
        } else {
            throw std::runtime_error("Only '~' and '~/' are supported in robot_mask_path");
        }
    } else {
        resolved_path = std::filesystem::path(raw_path);
    }

    if (resolved_path.is_relative()) {
        resolved_path = std::filesystem::absolute(resolved_path);
    }

    return resolved_path.lexically_normal();
}

std::string packageConfigPath(const std::string& filename)
{
    return (std::filesystem::path(
                ament_index_cpp::get_package_share_directory("rcj_localization")) /
            "config" / filename)
        .string();
}

}  // namespace

class FastMapRemapNode : public rclcpp::Node
{
public:
    FastMapRemapNode()
        : Node("fastmap_remap_node")
    {
        const auto fastMapPath = declare_parameter<std::string>("fastmap_file", "");
        const auto inputTopic = declare_parameter<std::string>("input_topic", "/image_raw");
        const auto outputTopic = declare_parameter<std::string>("output_topic", "/image_remapped");
        const auto inputTransport = declare_parameter<std::string>("input_transport", "raw");
        const auto interpolation = declare_parameter<std::string>("interpolation", "linear");
        const auto robotMaskPath = declare_parameter<std::string>(
            "robot_mask_path", packageConfigPath("remapped_mask.png"));
        declare_parameter("enable_image_view", false);
        declare_parameter("show_input_image", true);
        declare_parameter("show_output_image", true);
        declare_parameter("publish_debug_images", false);
        declare_parameter("publish_input_image", true);
        declare_parameter("publish_output_image", true);
        declare_parameter("display_max_width", 960);
        declare_parameter("display_max_height", 720);
        declare_parameter("enable_timing_log", true);
        declare_parameter("timing_log_interval", 30);

        loadFastMaps(fastMapPath);
        interpolationMode_ = parseInterpolation(interpolation);
        loadRobotMask(robotMaskPath);
        syncImageViewState();
        enableTimingLog_ = get_parameter("enable_timing_log").as_bool();
        timingLogInterval_ =
            std::max(1, static_cast<int>(get_parameter("timing_log_interval").as_int()));

        publisher_ = image_transport::create_publisher(
            this,
            outputTopic,
            rmw_qos_profile_sensor_data);
        robotMaskPublisher_ = create_publisher<sensor_msgs::msg::Image>(
            "~/robot_mask",
            rclcpp::QoS(1).reliable().transient_local());
        debugInputImagePublisher_ =
            create_publisher<sensor_msgs::msg::Image>("~/debug/input_image", 10);
        debugOutputImagePublisher_ =
            create_publisher<sensor_msgs::msg::Image>("~/debug/output_image", 10);

        subscription_ = image_transport::create_subscription(
            this,
            inputTopic,
            std::bind(&FastMapRemapNode::imageCallback, this, std::placeholders::_1),
            inputTransport,
            rmw_qos_profile_sensor_data);

        RCLCPP_INFO(get_logger(), "Loaded fast maps from: %s", fastMapPath.c_str());
        RCLCPP_INFO(
            get_logger(),
            "Fast map dimensions: source=%dx%d output=%dx%d",
            sourceWidth_,
            sourceHeight_,
            outputWidth_,
            outputHeight_);
        RCLCPP_INFO(get_logger(), "Subscribed to: %s", inputTopic.c_str());
        RCLCPP_INFO(get_logger(), "Publishing to: %s", outputTopic.c_str());
        RCLCPP_INFO(get_logger(), "Input transport: %s", inputTransport.c_str());
        RCLCPP_INFO(get_logger(), "Interpolation: %s", interpolation.c_str());
        if (robotMaskEnabled_) {
            publishRobotMask();
            RCLCPP_INFO(
                get_logger(),
                "Robot mask enabled from '%s'; publishing on %s/robot_mask",
                robotMaskPath_.c_str(),
                get_fully_qualified_name());
        } else {
            RCLCPP_INFO(get_logger(), "Robot mask disabled.");
        }
        RCLCPP_INFO(get_logger(), "Image view enabled: %s", enableImageView_ ? "true" : "false");
        RCLCPP_INFO(
            get_logger(),
            "Timing log enabled: %s, timing_log_interval=%d",
            enableTimingLog_ ? "true" : "false",
            timingLogInterval_);
    }

    ~FastMapRemapNode() override
    {
        destroyDebugWindows();
    }

private:
    int parseInterpolation(const std::string& interpolation) const
    {
        if (interpolation == "nearest") {
            return cv::INTER_NEAREST;
        }
        if (interpolation == "linear") {
            return cv::INTER_LINEAR;
        }

        throw std::runtime_error(
            "Unsupported interpolation '" + interpolation +
            "'. Supported values: nearest, linear.");
    }

    void syncWindow(const std::string& window_name, bool should_show, bool& created)
    {
        if (should_show && !created) {
            cv::namedWindow(window_name, cv::WINDOW_NORMAL);
            created = true;
        } else if (!should_show && created) {
            cv::destroyWindow(window_name);
            created = false;
        }
    }

    void syncImageViewState()
    {
        enableImageView_ = get_parameter("enable_image_view").as_bool();
        showInputImage_ = get_parameter("show_input_image").as_bool();
        showOutputImage_ = get_parameter("show_output_image").as_bool();
        publishDebugImages_ = get_parameter("publish_debug_images").as_bool();
        publishInputImage_ = get_parameter("publish_input_image").as_bool();
        publishOutputImage_ = get_parameter("publish_output_image").as_bool();
        displayMaxWidth_ = std::max(1, static_cast<int>(get_parameter("display_max_width").as_int()));
        displayMaxHeight_ = std::max(1, static_cast<int>(get_parameter("display_max_height").as_int()));

        const bool displayAvailable =
            std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
        if (enableImageView_ && !displayAvailable) {
            if (!headlessWarned_) {
                RCLCPP_WARN(
                    get_logger(),
                    "enable_image_view=true but no DISPLAY/WAYLAND_DISPLAY is available; "
                    "disabling OpenCV image view for this process.");
                headlessWarned_ = true;
            }
            enableImageView_ = false;
        }

        syncWindow(kInputWindowName, enableImageView_ && showInputImage_, inputWindowCreated_);
        syncWindow(kOutputWindowName, enableImageView_ && showOutputImage_, outputWindowCreated_);
    }

    void showDebugImages(const cv::Mat& input_image, const cv::Mat& output_image)
    {
        if (inputWindowCreated_) {
            cv::imshow(kInputWindowName, input_image);
            resizeWindowToFitImage(kInputWindowName, input_image, displayMaxWidth_, displayMaxHeight_);
        }

        if (outputWindowCreated_) {
            cv::imshow(kOutputWindowName, output_image);
            resizeWindowToFitImage(kOutputWindowName, output_image, displayMaxWidth_, displayMaxHeight_);
        }

        if (inputWindowCreated_ || outputWindowCreated_) {
            cv::waitKey(1);
        }
    }

    template<typename PublisherT>
    bool shouldPublishDebugImage(const std::shared_ptr<PublisherT>& publisher, bool image_enabled) const
    {
        return publishDebugImages_ && image_enabled && publisher != nullptr &&
               publisher->get_subscription_count() > 0U;
    }

    void publishDebugImages(
        const std_msgs::msg::Header& header,
        const std::string& encoding,
        const cv::Mat& input_image,
        const cv::Mat& output_image)
    {
        if (shouldPublishDebugImage(debugInputImagePublisher_, publishInputImage_)) {
            debugInputImagePublisher_->publish(
                *cv_bridge::CvImage(header, encoding, input_image).toImageMsg());
        }
        if (shouldPublishDebugImage(debugOutputImagePublisher_, publishOutputImage_)) {
            debugOutputImagePublisher_->publish(
                *cv_bridge::CvImage(header, encoding, output_image).toImageMsg());
        }
    }

    void destroyDebugWindows()
    {
        syncWindow(kInputWindowName, false, inputWindowCreated_);
        syncWindow(kOutputWindowName, false, outputWindowCreated_);
    }

    void loadFastMaps(const std::string& fastMapPath)
    {
        if (fastMapPath.empty()) {
            throw std::runtime_error("Parameter 'fastmap_file' must be set.");
        }

        cv::FileStorage fs(fastMapPath, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Cannot open fast map file: " + fastMapPath);
        }

        fs["source_width"] >> sourceWidth_;
        fs["source_height"] >> sourceHeight_;
        fs["output_width"] >> outputWidth_;
        fs["output_height"] >> outputHeight_;
        fs["fast_map_1"] >> fastMap1_;
        fs["fast_map_2"] >> fastMap2_;
        fs.release();

        if (fastMap1_.empty() || fastMap2_.empty()) {
            throw std::runtime_error("Fast map file is missing fast_map_1 or fast_map_2: " + fastMapPath);
        }

        if (fastMap1_.size() != fastMap2_.size()) {
            throw std::runtime_error("Fast map matrices have different sizes: " + fastMapPath);
        }

        if (outputWidth_ <= 0 || outputHeight_ <= 0) {
            outputWidth_ = fastMap1_.cols;
            outputHeight_ = fastMap1_.rows;
        }

        if (outputWidth_ != fastMap1_.cols || outputHeight_ != fastMap1_.rows) {
            throw std::runtime_error("Fast map output_width/output_height does not match matrix size: " + fastMapPath);
        }
    }

    void loadRobotMask(const std::string& robotMaskPath)
    {
        if (robotMaskPath.empty()) {
            robotMaskEnabled_ = false;
            robotMaskPath_.clear();
            robotAllowedMask_.release();
            return;
        }

        const std::filesystem::path resolved_path = resolvePath(robotMaskPath);
        cv::Mat loaded_mask = cv::imread(resolved_path.string(), cv::IMREAD_GRAYSCALE);
        if (loaded_mask.empty()) {
            throw std::runtime_error(
                "Failed to load robot mask from '" + robotMaskPath + "' (resolved path: '" +
                resolved_path.string() + "')");
        }

        if (loaded_mask.cols != outputWidth_ || loaded_mask.rows != outputHeight_) {
            throw std::runtime_error(
                "Robot mask size mismatch: mask=" + std::to_string(loaded_mask.cols) + "x" +
                std::to_string(loaded_mask.rows) + " remap_output=" +
                std::to_string(outputWidth_) + "x" + std::to_string(outputHeight_));
        }

        cv::compare(loaded_mask, 0, robotAllowedMask_, cv::CMP_GT);
        robotMaskEnabled_ = true;
        robotMaskPath_ = resolved_path.string();
    }

    void publishRobotMask()
    {
        if (!robotMaskEnabled_ || robotMaskPublisher_ == nullptr) {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        robotMaskPublisher_->publish(
            *cv_bridge::CvImage(header, "mono8", robotAllowedMask_).toImageMsg());
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            const auto cvInput = cv_bridge::toCvShare(msg, msg->encoding);
            syncImageViewState();

            if (sourceWidth_ > 0 && sourceHeight_ > 0 &&
                (cvInput->image.cols != sourceWidth_ || cvInput->image.rows != sourceHeight_)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Input image size %dx%d does not match fast map source size %dx%d. Frame dropped.",
                    cvInput->image.cols,
                    cvInput->image.rows,
                    sourceWidth_,
                    sourceHeight_);
                return;
            }

            const auto remapStart = std::chrono::steady_clock::now();
            cv::remap(
                cvInput->image,
                remappedImage_,
                fastMap1_,
                fastMap2_,
                interpolationMode_,
                cv::BORDER_CONSTANT);
            const auto remapEnd = std::chrono::steady_clock::now();

            const auto remapDurationUs =
                std::chrono::duration_cast<std::chrono::microseconds>(remapEnd - remapStart).count();
            ++frameCount_;
            totalRemapTimeUs_ += remapDurationUs;
            const double averageRemapUs =
                static_cast<double>(totalRemapTimeUs_) / static_cast<double>(frameCount_);

            auto outputMsg = cv_bridge::CvImage(msg->header, msg->encoding, remappedImage_).toImageMsg();
            publisher_.publish(*outputMsg);
            if (robotMaskEnabled_) {
                publishRobotMask();
            }
            publishDebugImages(msg->header, msg->encoding, cvInput->image, remappedImage_);
            showDebugImages(cvInput->image, remappedImage_);

            if (enableTimingLog_ &&
                frameCount_ % static_cast<uint64_t>(timingLogInterval_) == 0U) {
                RCLCPP_INFO(
                    get_logger(),
                    "frame=%llu remap_us=%lld remap_ms=%.3f avg_remap_ms=%.3f",
                    static_cast<unsigned long long>(frameCount_),
                    static_cast<long long>(remapDurationUs),
                    static_cast<double>(remapDurationUs) / 1000.0,
                    averageRemapUs / 1000.0);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "cv_bridge error: %s",
                e.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "OpenCV remap error: %s",
                e.what());
        }
    }

    image_transport::Subscriber subscription_;
    image_transport::Publisher publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr robotMaskPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugInputImagePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugOutputImagePublisher_;
    cv::Mat fastMap1_;
    cv::Mat fastMap2_;
    cv::Mat remappedImage_;
    cv::Mat robotAllowedMask_;
    int sourceWidth_ = 0;
    int sourceHeight_ = 0;
    int outputWidth_ = 0;
    int outputHeight_ = 0;
    int interpolationMode_ = cv::INTER_LINEAR;
    bool robotMaskEnabled_ = false;
    std::string robotMaskPath_;
    bool enableImageView_ = false;
    bool showInputImage_ = true;
    bool showOutputImage_ = true;
    bool publishDebugImages_ = false;
    bool publishInputImage_ = true;
    bool publishOutputImage_ = true;
    bool headlessWarned_ = false;
    bool inputWindowCreated_ = false;
    bool outputWindowCreated_ = false;
    int displayMaxWidth_ = 960;
    int displayMaxHeight_ = 720;
    bool enableTimingLog_ = true;
    int timingLogInterval_ = 30;
    uint64_t frameCount_ = 0;
    int64_t totalRemapTimeUs_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FastMapRemapNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("fastmap_remap_node"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
