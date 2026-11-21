#include "down_cam_object_detection.hpp"

#include "node_utils.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace vision {

namespace {

constexpr int kWarnThrottleMs = 5000;
constexpr int kInfoThrottleMs = 2000;

// V4L2 control name -> OpenCV CAP_PROP. Controls without an OpenCV constant go through v4l2-ctl.
const std::vector<std::pair<std::string, int>>& opencv_controls() {
    static const std::vector<std::pair<std::string, int>> controls = {
        {"brightness", cv::CAP_PROP_BRIGHTNESS},
        {"contrast", cv::CAP_PROP_CONTRAST},
        {"saturation", cv::CAP_PROP_SATURATION},
        {"hue", cv::CAP_PROP_HUE},
        {"sharpness", cv::CAP_PROP_SHARPNESS},
        {"gamma", cv::CAP_PROP_GAMMA},
        {"auto_white_balance", cv::CAP_PROP_AUTO_WB},
        {"white_balance_temperature", cv::CAP_PROP_WB_TEMPERATURE},
        {"auto_focus", cv::CAP_PROP_AUTOFOCUS},
        {"focus_absolute", cv::CAP_PROP_FOCUS},
        {"auto_exposure", cv::CAP_PROP_AUTO_EXPOSURE},
        {"exposure_time_absolute", cv::CAP_PROP_EXPOSURE},
    };
    return controls;
}

const std::vector<std::string>& v4l2_only_controls() {
    static const std::vector<std::string> controls = {"power_line_frequency", "backlight_compensation"};
    return controls;
}

const std::vector<std::string>& bool_controls() {
    static const std::vector<std::string> controls = {"auto_white_balance", "auto_focus"};
    return controls;
}

bool is_bool_control(const std::string& name) {
    const auto& b = bool_controls();
    return std::find(b.begin(), b.end(), name) != b.end();
}

long param_as_long(const rclcpp::Parameter& p) {
    return p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? static_cast<long>(p.as_bool()) : p.as_int();
}

void set_nan_position(auv_msgs::msg::VisionDetection& det) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    det.pose_camera.pose.position.x = nan;
    det.pose_camera.pose.position.y = nan;
    det.pose_camera.pose.position.z = nan;
}

}  // namespace

template <typename T>
T DownCamObjectDetectionNode::required_param(const std::string& name) {
    // Declared without a default, like the Python node: a missing YAML entry is a startup error.
    declare_parameter<T>(name);
    return get_parameter(name).get_value<T>();
}

DownCamObjectDetectionNode::DownCamObjectDetectionNode(const rclcpp::NodeOptions& options)
    : Node("down_cam_object_detection", options) {
    load_parameters();

    const std::string rel_override = required_param<std::string>("model_relative_path_override");
    const std::string rel_path = !rel_override.empty()
        ? rel_override
        : get_parameter(sim_ ? "model_relative_path.sim" : "model_relative_path.real").as_string();
    const fs::path model_path = fs::path(ament_index_cpp::get_package_share_directory("vision")) / rel_path;
    if (enable_object_detection_ && !fs::exists(model_path)) {
        RCLCPP_FATAL(get_logger(), "Model path does not exist: %s", model_path.c_str());
        throw std::runtime_error("Model path does not exist: " + model_path.string());
    }

    // Like the Python node, a model folder with only the weights placeholder publishes the raw feed.
    const bool has_weights = fs::exists(model_path / "weights.onnx") || fs::exists(model_path / "rfdetr_cpp_fp16.engine");
    if (enable_object_detection_ && has_weights) {
        RfDetrEngineOptions opts;
        opts.model_dir = model_path;
        opts.engine_builder = fs::path(ament_index_cpp::get_package_prefix("vision")) / "lib" / "vision" / "rfdetr_build";
        opts.log_info = [this](const std::string& msg) { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); };
        engine_ = std::make_unique<RfDetrEngine>(opts);
        if (!engine_->has_masks()) {
            RCLCPP_WARN(get_logger(), "Model has no segmentation output: centroids fall back to box centres, no yaw.");
        }
    } else if (enable_object_detection_) {
        RCLCPP_WARN(get_logger(), "Model weights not found in %s (placeholder detected). Publishing raw feed only.",
                    model_path.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Object detection is disabled. Publishing raw feed only.");
    }

    create_ros_interfaces();

    if (!sim_) {
        open_camera();
        apply_camera_controls();
        grab_thread_ = std::thread(&DownCamObjectDetectionNode::grab_loop, this);
    } else {
        RCLCPP_INFO(get_logger(), "Simulation mode: Subscribing to %s", image_topic_.c_str());
        auto on_frame = [this](const cv::Mat& img, const builtin_interfaces::msg::Time& stamp) {
            try {
                process_frame(img, stamp);
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Frame processing failed: %s", e.what());
            }
        };
        if (compressed_) {
            compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_, queue_size_, [this, on_frame](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                    try {
                        on_frame(cv_bridge::toCvCopy(*msg, "bgr8")->image, msg->header.stamp);
                    } catch (const cv_bridge::Exception& e) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "CV Bridge error: %s", e.what());
                    }
                });
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, queue_size_, [this, on_frame](const sensor_msgs::msg::Image::SharedPtr msg) {
                    try {
                        on_frame(cv_bridge::toCvCopy(*msg, "bgr8")->image, msg->header.stamp);
                    } catch (const cv_bridge::Exception& e) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "CV Bridge error: %s", e.what());
                    }
                });
        }
    }

    RCLCPP_INFO(get_logger(), "%s initialized.", get_name());
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_set_parameters(params); });
}

DownCamObjectDetectionNode::~DownCamObjectDetectionNode() {
    running_ = false;
    if (grab_thread_.joinable()) grab_thread_.join();
    std::lock_guard<std::mutex> lock(cap_mutex_);
    if (cap_.isOpened()) cap_.release();
}

void DownCamObjectDetectionNode::load_parameters() {
    class_names_ = required_param<std::vector<std::string>>("model_class_names");
    std::string names;
    for (const auto& n : class_names_) names += (names.empty() ? "" : ", ") + n;
    RCLCPP_INFO(get_logger(), "Class names: [%s]", names.c_str());

    declare_parameter<std::string>("model_relative_path.sim");
    declare_parameter<std::string>("model_relative_path.real");
    detection_topic_ = required_param<std::string>("detection_topic");
    queue_size_ = static_cast<int>(required_param<int64_t>("queue_size"));
    publish_annotated_image_ = required_param<bool>("publish_annotated");
    annotated_image_enabled_ = publish_annotated_image_;
    publish_annotated_every_n_frames_ =
        std::max(1, static_cast<int>(required_param<int64_t>("publish_annotated_every_n_frames")));
    conf_threshold_ = required_param<double>("model_detection_threshold");
    compressed_ = required_param<bool>("compressed");
    enable_object_detection_ = required_param<bool>("enable_object_detection");
    sim_ = required_param<bool>("sim");
    image_topic_ = required_param<std::string>("image_topic");

    video_device_ = required_param<std::string>("video_device");
    image_width_ = static_cast<int>(required_param<int64_t>("width"));
    image_height_ = static_cast<int>(required_param<int64_t>("height"));
    camera_fps_ = static_cast<int>(required_param<int64_t>("fps"));
    camera_frame_id_ = required_param<std::string>("camera_frame_id");
    base_frame_id_ = required_param<std::string>("base_frame_id");

    intrinsics_.fx = required_param<double>("fx");
    intrinsics_.fy = required_param<double>("fy");
    intrinsics_.cx = required_param<double>("cx");
    intrinsics_.cy = required_param<double>("cy");
    camera_info_topic_ = required_param<std::string>("camera_info_topic");

    const auto xyz = required_param<std::vector<double>>("auv_to_down_cam.xyz");
    const auto rpy = required_param<std::vector<double>>("auv_to_down_cam.rpy");
    if (xyz.size() != 3 || rpy.size() != 3) {
        throw std::invalid_argument("auv_to_down_cam.xyz and auv_to_down_cam.rpy must contain exactly 3 values.");
    }
    T_auv_down_cam_ = transform_from_xyz_rpy(xyz, rpy);

    const auto labels = required_param<std::vector<std::string>>("down_cam_projection_labels");
    const auto heights = required_param<std::vector<double>>("down_cam_projection_heights");
    for (std::size_t i = 0; i < std::min(labels.size(), heights.size()); ++i) {
        projection_heights_[labels[i]] = heights[i];
    }
    pool_floor_z_ = required_param<double>("pool_floor_z");
    // depth_scale is declared for parity with the yaml but, as in the Python node, unused by the ray-plane math.
    declare_parameter<double>("depth_scale.sim");
    declare_parameter<double>("depth_scale.real");
    declare_parameter<double>("lateral_scale.sim");
    declare_parameter<double>("lateral_scale.real");
    lateral_scale_ = get_parameter(sim_ ? "lateral_scale.sim" : "lateral_scale.real").as_double();
    pose_source_ = required_param<std::string>("pose_source");
    auv_pose_topic_ = required_param<std::string>("auv_pose_topic");
    border_margin_ = static_cast<int>(required_param<int64_t>("border_margin"));

    for (const auto& [name, prop] : opencv_controls()) {
        (void)prop;
        if (is_bool_control(name)) {
            required_param<bool>(name);
        } else {
            required_param<int64_t>(name);
        }
    }
    for (const auto& name : v4l2_only_controls()) required_param<int64_t>(name);

    collection_dir_ = required_param<std::string>("collection_dir");
    collection_interval_ = required_param<double>("collection_interval_seconds");
    collection_extension_ = required_param<std::string>("collection_file_extension");
}

void DownCamObjectDetectionNode::create_ros_interfaces() {
    if (pose_source_ == "auv_pose") {
        auv_pose_sub_ = create_subscription<PoseStamped>(
            auv_pose_topic_, rclcpp::SensorDataQoS(), [this](const PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(auv_pose_mutex_);
                auv_pose_msg_ = *msg;
            });
        RCLCPP_INFO(get_logger(), "Subscribed to AUV pose on: %s with sensor data QoS", auv_pose_topic_.c_str());
    }

    using std::placeholders::_1;
    using std::placeholders::_2;
    services_.push_back(create_service<auv_msgs::srv::AutomaticCapture>(
        "/vision/down_cam/toggle_collection", std::bind(&DownCamObjectDetectionNode::toggle_collection, this, _1, _2)));
    services_.push_back(create_service<std_srvs::srv::Trigger>(
        "/vision/down_cam/toggle_annotated_image", std::bind(&DownCamObjectDetectionNode::toggle_annotated, this, _1, _2)));
    services_.push_back(create_service<std_srvs::srv::Trigger>(
        "/vision/down_cam/get_camera_settings", std::bind(&DownCamObjectDetectionNode::get_camera_settings, this, _1, _2)));
    services_.push_back(create_service<auv_msgs::srv::SetDownCamProjectionHeights>(
        "/vision/down_cam/set_projection_heights",
        std::bind(&DownCamObjectDetectionNode::set_projection_heights, this, _1, _2)));

    pub_detections_ = create_publisher<auv_msgs::msg::VisionDetectionFrame>(detection_topic_, queue_size_);
    RCLCPP_INFO(get_logger(), "Publishing detections to: %s", detection_topic_.c_str());

    pub_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, queue_size_);
    RCLCPP_INFO(get_logger(), "Publishing camera info to: %s", camera_info_topic_.c_str());
    build_camera_info_template();

    if (publish_annotated_image_) {
        const std::string topic = detection_topic_ + "/annotated" + (compressed_ ? "/compressed" : "");
        if (compressed_) {
            pub_annotated_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic, queue_size_);
        } else {
            pub_annotated_raw_ = create_publisher<sensor_msgs::msg::Image>(topic, queue_size_);
        }
        RCLCPP_INFO(get_logger(), "Publishing annotated debug image to: %s", topic.c_str());
    }
}

void DownCamObjectDetectionNode::build_camera_info_template() {
    const auto& K = intrinsics_;
    auto& info = camera_info_template_;
    info.header.frame_id = camera_frame_id_;
    info.width = image_width_;
    info.height = image_height_;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {K.fx, 0.0, K.cx, 0.0, 0.0, K.fy, K.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

void DownCamObjectDetectionNode::open_camera() {
    RCLCPP_INFO(get_logger(), "Opening camera: %s (%dx%d @ %d fps)", video_device_.c_str(), image_width_,
                image_height_, camera_fps_);
    std::lock_guard<std::mutex> lock(cap_mutex_);
    if (!cap_.open(video_device_, cv::CAP_V4L2)) {
        throw std::runtime_error("Failed to open camera: " + video_device_);
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS, camera_fps_);
    RCLCPP_INFO(get_logger(), "Camera opened: %dx%d @ %.1f fps", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
                static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), cap_.get(cv::CAP_PROP_FPS));
}

void DownCamObjectDetectionNode::apply_camera_controls() {
    std::string applied;
    const auto add_applied = [&applied](const std::string& s) { applied += (applied.empty() ? "" : ", ") + s; };

    for (const auto& [name, prop] : opencv_controls()) {
        // Manual values are ignored by the camera while the matching auto mode is on.
        if (name == "white_balance_temperature" && get_parameter("auto_white_balance").as_bool()) continue;
        if (name == "exposure_time_absolute" && get_parameter("auto_exposure").as_int() == 3) continue;
        if (name == "focus_absolute" && get_parameter("auto_focus").as_bool()) continue;

        const long value = param_as_long(get_parameter(name));
        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(cap_mutex_);
            ok = cap_.set(prop, static_cast<double>(value));
        }
        if (ok) {
            add_applied(name + "=" + std::to_string(value));
        } else {
            RCLCPP_WARN(get_logger(), "Failed to set %s=%ld via OpenCV", name.c_str(), value);
        }
    }
    for (const auto& name : v4l2_only_controls()) {
        const long value = get_parameter(name).as_int();
        if (set_v4l2_ctl(name, value)) add_applied(name + "=" + std::to_string(value));
    }
    RCLCPP_INFO(get_logger(), "Applied camera controls: %s", applied.c_str());
}

bool DownCamObjectDetectionNode::set_v4l2_ctl(const std::string& name, long value) {
    const std::string cmd = "v4l2-ctl -d '" + video_device_ + "' --set-ctrl " + name + "=" + std::to_string(value)
                            + " >/dev/null 2>&1";
    if (std::system(cmd.c_str()) != 0) {
        RCLCPP_WARN(get_logger(), "v4l2-ctl %s failed", name.c_str());
        return false;
    }
    return true;
}

std::string DownCamObjectDetectionNode::get_v4l2_ctl(const std::string& name) {
    const std::string cmd = "v4l2-ctl -d '" + video_device_ + "' --get-ctrl " + name + " 2>&1";
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return "ERROR popen failed";
    std::string output;
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe.get()) != nullptr) output += buf;
    while (!output.empty() && std::isspace(static_cast<unsigned char>(output.back()))) output.pop_back();
    // "power_line_frequency: 2" -> "2"
    const auto colon = output.rfind(':');
    if (colon == std::string::npos) return output;
    std::string value = output.substr(colon + 1);
    value.erase(0, value.find_first_not_of(' '));
    return value;
}

void DownCamObjectDetectionNode::grab_loop() {
    cv::Mat img;
    while (rclcpp::ok() && running_) {
        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(cap_mutex_);
            ok = cap_.read(img);  // blocks at the camera frame rate
        }
        if (!ok || img.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Failed to read frame from camera");
            // Don't spin a core at 100% while the camera is unplugged.
            std::this_thread::sleep_for(10ms);
            continue;
        }
        try {
            process_frame(img, now());
        } catch (const std::exception& e) {
            // An exception escaping this thread would std::terminate the whole node; drop the frame instead.
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Frame processing failed: %s", e.what());
        }
    }
}

// ---------------------------------------------------------------------------
// Frame pipeline
// ---------------------------------------------------------------------------

void DownCamObjectDetectionNode::process_frame(const cv::Mat& img, const builtin_interfaces::msg::Time& stamp) {
    ++frame_counter_;

    bool save_frame = false;
    {
        std::lock_guard<std::mutex> lock(collection_mutex_);
        const double t = wall_time_sec();
        if (collecting_ && t - last_collection_time_ >= collection_interval_) {
            last_collection_time_ = t;
            save_frame = true;
        }
    }
    if (save_frame) {
        const fs::path path = fs::path(collection_dir_) / ("down_" + collection_timestamp() + collection_extension_);
        cv::imwrite(path.string(), img);
        RCLCPP_DEBUG(get_logger(), "Saved %s", path.c_str());
    }

    std::vector<Detection2D> detections;
    if (enable_object_detection_ && engine_) {
        try {
            detections = engine_->detect(img, static_cast<float>(conf_threshold_));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Inference failed: %s", e.what());
        }
    }

    std::optional<PoseStamped> auv_pose;
    {
        std::lock_guard<std::mutex> lock(auv_pose_mutex_);
        auv_pose = auv_pose_msg_;
    }
    if (!auv_pose) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kInfoThrottleMs, "auv_pose_msg is None! Cannot compute 3D positions.");
    }
    std::optional<Eigen::Matrix4d> T_world_camera;
    if (auv_pose) T_world_camera = transform_from_pose(auv_pose->pose) * T_auv_down_cam_;

    auv_msgs::msg::VisionDetectionFrame frame;
    frame.header.stamp = stamp;
    frame.header.frame_id = base_frame_id_;
    if (auv_pose) frame.auv_pose = *auv_pose;

    std::vector<std::array<cv::Point2f, 4>> boxes;
    boxes.reserve(detections.size());
    for (const auto& d : detections) {
        const auto geom = analyze_down_cam_mask(d.mask, d.bbox, img.cols, img.rows, border_margin_);
        boxes.push_back(geom.box);
        if (d.class_id < 0 || d.class_id >= static_cast<int>(class_names_.size())) continue;
        const std::string& label = class_names_[d.class_id];

        auv_msgs::msg::VisionDetection det;
        det.label = label;
        det.confidence = d.score;
        det.bbox_center_x = geom.center_x;
        det.bbox_center_y = geom.center_y;
        det.bbox_size_x = d.bbox.x2 - d.bbox.x1;
        det.bbox_size_y = d.bbox.y2 - d.bbox.y1;

        set_nan_position(det);
        if (T_world_camera) {
            std::optional<double> height;
            {
                std::lock_guard<std::mutex> lock(heights_mutex_);
                const auto it = projection_heights_.find(label);
                if (it != projection_heights_.end()) height = it->second;
            }
            if (!height) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kInfoThrottleMs,
                                     "Label '%s' not in down_cam_projection_labels, skipping 3D projection.", label.c_str());
            } else if (const auto pos = project_down_cam_to_plane(geom.center_x, geom.center_y, intrinsics_, lateral_scale_,
                                                                  *T_world_camera, pool_floor_z_ + *height)) {
                det.pose_camera.pose.position.x = pos->x();
                det.pose_camera.pose.position.y = pos->y();
                det.pose_camera.pose.position.z = pos->z();
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kInfoThrottleMs,
                                     "Ray for %s does not hit its projection plane in front of the camera", label.c_str());
            }
        }

        if (geom.yaw) {
            const Eigen::Quaterniond q = down_cam_orientation_from_yaw(*geom.yaw);
            det.pose_camera.pose.orientation.x = q.x();
            det.pose_camera.pose.orientation.y = q.y();
            det.pose_camera.pose.orientation.z = q.z();
            det.pose_camera.pose.orientation.w = q.w();
        } else {
            // All-zero quaternion tells the tracker to ignore this orientation update.
            det.pose_camera.pose.orientation.x = 0.0;
            det.pose_camera.pose.orientation.y = 0.0;
            det.pose_camera.pose.orientation.z = 0.0;
            det.pose_camera.pose.orientation.w = 0.0;
        }
        frame.detections.push_back(det);
    }
    pub_detections_->publish(frame);

    auto info = camera_info_template_;
    info.header.stamp = stamp;
    pub_camera_info_->publish(info);

    if (publish_annotated_image_ && annotated_image_enabled_
        && frame_counter_ % static_cast<std::uint64_t>(publish_annotated_every_n_frames_) == 0) {
        publish_annotated(img, detections, boxes, stamp);
    }
}

void DownCamObjectDetectionNode::publish_annotated(const cv::Mat& img, const std::vector<Detection2D>& detections,
                                                   const std::vector<std::array<cv::Point2f, 4>>& boxes,
                                                   const builtin_interfaces::msg::Time& stamp) {
    cv::Mat output = img.clone();
    const int thickness = std::max(1, static_cast<int>(std::round(std::min(img.cols, img.rows) / 400.0)) + 1);
    const double text_scale = std::max(0.4, std::min(img.cols, img.rows) / 900.0);

    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& d = detections[i];
        const cv::Scalar color((d.class_id * 67) % 256, (d.class_id * 137 + 80) % 256, 255);

        if (!d.mask.empty()) {
            // Half-transparent fill, like supervision's MaskAnnotator.
            cv::Mat overlay = output.clone();
            overlay.setTo(color, d.mask);
            cv::addWeighted(overlay, 0.5, output, 0.5, 0.0, output);
        }

        std::vector<cv::Point> poly;
        for (const auto& p : boxes[i]) poly.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::polylines(output, poly, /*isClosed=*/true, color, thickness);

        if (d.class_id < 0 || d.class_id >= static_cast<int>(class_names_.size())) continue;
        const std::string text = class_names_[d.class_id] + " " + cv::format("%.2f", d.score);
        int baseline = 0;
        const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, thickness, &baseline);
        const cv::Point text_org(static_cast<int>(d.bbox.x1), std::max(size.height + 4, static_cast<int>(d.bbox.y1) - 4));
        cv::rectangle(output, text_org + cv::Point(-2, baseline), text_org + cv::Point(size.width + 2, -size.height - 4),
                      color, cv::FILLED);
        cv::putText(output, text, text_org, cv::FONT_HERSHEY_SIMPLEX, text_scale, cv::Scalar(0, 0, 0), thickness);
    }

    try {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = camera_frame_id_;
        cv_bridge::CvImage cv_img(header, "bgr8", output);
        if (pub_annotated_compressed_) {
            pub_annotated_compressed_->publish(*cv_img.toCompressedImageMsg());
        } else if (pub_annotated_raw_) {
            pub_annotated_raw_->publish(*cv_img.toImageMsg());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to publish annotated image: %s", e.what());
    }
}

// ---------------------------------------------------------------------------
// Parameters & services
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult DownCamObjectDetectionNode::on_set_parameters(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) return result;
    }

    for (const auto& p : params) {
        const auto& cv_controls = opencv_controls();
        const auto it = std::find_if(cv_controls.begin(), cv_controls.end(),
                                     [&p](const auto& c) { return c.first == p.get_name(); });
        if (it != cv_controls.end()) {
            const long value = param_as_long(p);
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                ok = cap_.set(it->second, static_cast<double>(value));
            }
            if (ok) {
                RCLCPP_INFO(get_logger(), "Dynamically updated %s=%ld", p.get_name().c_str(), value);
            } else {
                RCLCPP_WARN(get_logger(), "Failed to dynamically set %s=%ld", p.get_name().c_str(), value);
                result.successful = false;
            }
        } else if (std::find(v4l2_only_controls().begin(), v4l2_only_controls().end(), p.get_name())
                   != v4l2_only_controls().end()) {
            if (set_v4l2_ctl(p.get_name(), p.as_int())) {
                RCLCPP_INFO(get_logger(), "Dynamically updated v4l2 %s=%ld", p.get_name().c_str(), p.as_int());
            } else {
                result.successful = false;
            }
        }
    }
    return result;
}

void DownCamObjectDetectionNode::toggle_collection(
    const std::shared_ptr<auv_msgs::srv::AutomaticCapture::Request> request,
    std::shared_ptr<auv_msgs::srv::AutomaticCapture::Response> response) {
    if (request->time_interval <= 0.0) {
        response->success = false;
        response->message = "Collection interval must be > 0 seconds";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }
    std::lock_guard<std::mutex> lock(collection_mutex_);
    collecting_ = request->data;
    collection_interval_ = request->time_interval;
    if (collecting_) {
        fs::create_directories(collection_dir_);
        RCLCPP_INFO(get_logger(), "Image collection ENABLED every %.2fs → %s", collection_interval_, collection_dir_.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Image collection DISABLED (interval preserved at %.2fs)", collection_interval_);
    }
    response->success = true;
    response->message = std::string("Collection ") + (collecting_ ? "enabled" : "disabled") + " with interval "
                        + cv::format("%.2f", collection_interval_) + "s";
}

void DownCamObjectDetectionNode::toggle_annotated(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    annotated_image_enabled_ = !annotated_image_enabled_;
    response->success = true;
    response->message = std::string("Annotated image ") + (annotated_image_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void DownCamObjectDetectionNode::get_camera_settings(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string message;
    {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) {
            response->success = false;
            response->message = "Camera not opened.";
            return;
        }
        for (const auto& [name, prop] : opencv_controls()) {
            message += (message.empty() ? "" : " | ") + name + ": " + cv::format("%g", cap_.get(prop));
        }
    }
    for (const auto& name : v4l2_only_controls()) {
        message += " | " + name + ": " + get_v4l2_ctl(name);
    }
    response->success = true;
    response->message = message;
}

void DownCamObjectDetectionNode::set_projection_heights(
    const std::shared_ptr<auv_msgs::srv::SetDownCamProjectionHeights::Request> request,
    std::shared_ptr<auv_msgs::srv::SetDownCamProjectionHeights::Response> response) {
    std::lock_guard<std::mutex> lock(heights_mutex_);
    response->success = true;
    if (request->all_labels) {
        for (auto& [label, height] : projection_heights_) {
            (void)label;
            height = request->projection_height;
        }
    } else if (const auto it = projection_heights_.find(request->specific_label); it != projection_heights_.end()) {
        it->second = request->projection_height;
    } else {
        response->success = false;
        RCLCPP_WARN(get_logger(), "Label '%s' not found in down_cam_projection_labels.", request->specific_label.c_str());
    }
}

}  // namespace vision
