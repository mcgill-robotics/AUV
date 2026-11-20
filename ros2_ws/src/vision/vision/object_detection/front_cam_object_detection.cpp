#include "front_cam_object_detection.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace vision {

namespace {

constexpr int kWarnThrottleMs = 5000;

Eigen::Matrix4d transform_from_xyz_rpy(const std::vector<double>& xyz, const std::vector<double>& rpy) {
    // Same convention as tf_transformations.euler_matrix (static XYZ): R = Rz(yaw) * Ry(pitch) * Rx(roll).
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                              .toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    return T;
}

geometry_msgs::msg::PoseStamped pose_from_transform(const Eigen::Matrix4d& T, const std::string& frame_id,
                                                    const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = T(0, 3);
    msg.pose.position.y = T(1, 3);
    msg.pose.position.z = T(2, 3);
    const Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    return msg;
}

Eigen::Matrix4d transform_from_pose(const geometry_msgs::msg::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return T;
}

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}

double wall_time_sec() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// datetime.now().strftime('%Y%m%d_%H%M%S_%f')
std::string collection_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S_") << std::setw(6) << std::setfill('0') << us;
    return ss.str();
}

// Metres (NaN/inf allowed) -> uint16 millimetres, as in the Python node.
cv::Mat depth_to_mm(const cv::Mat& depth) {
    cv::Mat clean = depth.clone();
    cv::patchNaNs(clean, 0.0);
    clean.setTo(0.0f, (clean == std::numeric_limits<float>::infinity())
                          | (clean == -std::numeric_limits<float>::infinity()));
    cv::Mat mm;
    clean.convertTo(mm, CV_16UC1, 1000.0);  // saturates to [0, 65535]
    return mm;
}

#ifdef ZED_SDK_AVAILABLE
// Map a parameter string (e.g. "HD720", "NEURAL", "GEN_3") onto a ZED enum via sl::toString.
template <typename E>
E zed_enum_from_name(const std::string& raw_name, const std::string& param_name) {
    // sl::toString spells some values with spaces ("GEN 3") while the yaml uses the enum spelling ("GEN_3").
    const auto normalize = [](std::string s) {
        s = upper(s);
        std::replace_if(s.begin(), s.end(), [](char c) { return c == ' ' || c == '-'; }, '_');
        return s;
    };
    const std::string name = normalize(raw_name);
    std::string valid;
    for (int i = 0; i < static_cast<int>(E::LAST); ++i) {
        const auto value = static_cast<E>(i);
        const std::string candidate = normalize(sl::toString(value).c_str());
        if (candidate == name) return value;
        valid += (valid.empty() ? "" : ", ") + candidate;
    }
    throw std::invalid_argument("Unsupported " + param_name + " '" + raw_name + "'. Expected one of: " + valid);
}

sl::FLIP_MODE flip_mode_from_name(const std::string& raw_name) {
    const std::string name = upper(raw_name);
    if (name == "OFF" || name == "FALSE") return sl::FLIP_MODE::OFF;
    if (name == "ON" || name == "TRUE") return sl::FLIP_MODE::ON;
    if (name == "AUTO") return sl::FLIP_MODE::AUTO;
    throw std::invalid_argument("Unsupported zed_flip_mode '" + raw_name + "'. Expected OFF, ON or AUTO");
}

const std::vector<std::pair<std::string, sl::VIDEO_SETTINGS>>& zed_runtime_camera_settings() {
    static const std::vector<std::pair<std::string, sl::VIDEO_SETTINGS>> settings = {
        {"zed_image_brightness", sl::VIDEO_SETTINGS::BRIGHTNESS},
        {"zed_image_contrast", sl::VIDEO_SETTINGS::CONTRAST},
        {"zed_image_hue", sl::VIDEO_SETTINGS::HUE},
        {"zed_image_saturation", sl::VIDEO_SETTINGS::SATURATION},
        {"zed_image_sharpness", sl::VIDEO_SETTINGS::SHARPNESS},
        {"zed_image_gamma", sl::VIDEO_SETTINGS::GAMMA},
        {"zed_image_gain", sl::VIDEO_SETTINGS::GAIN},
        {"zed_image_auto_exposure", sl::VIDEO_SETTINGS::AEC_AGC},
        {"zed_image_auto_white_balance", sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO},
        {"zed_led_status", sl::VIDEO_SETTINGS::LED_STATUS},
    };
    return settings;
}

std::string zed_error(sl::ERROR_CODE err) { return sl::toString(err).c_str(); }
#endif

}  // namespace

template <typename T>
T FrontCamObjectDetectionNode::required_param(const std::string& name) {
    // Declared without a default, like the Python node: a missing YAML entry is a startup error.
    declare_parameter<T>(name);
    return get_parameter(name).get_value<T>();
}

FrontCamObjectDetectionNode::FrontCamObjectDetectionNode(const rclcpp::NodeOptions& options)
    : Node("front_cam_object_detection", options) {
    load_parameters();

    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/sensors/depth/z", rclcpp::SensorDataQoS(), [this](const std_msgs::msg::Float64::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            sensor_depth_ = msg->data;
        });

    const std::string rel_override = required_param<std::string>("model_relative_path_override");
    const std::string rel_path = !rel_override.empty()
        ? rel_override
        : get_parameter(sim_ ? "model_relative_path.sim" : "model_relative_path.real").as_string();
    const fs::path model_path = fs::path(ament_index_cpp::get_package_share_directory("vision")) / rel_path;
    if (enable_object_detection_ && !fs::exists(model_path)) {
        RCLCPP_FATAL(get_logger(), "Model path does not exist: %s", model_path.c_str());
        throw std::runtime_error("Model path does not exist: " + model_path.string());
    }

    if (has_zed_sdk_) {
#ifdef ZED_SDK_AVAILABLE
        open_zed();
#else
        RCLCPP_FATAL(get_logger(), "has_zed_sdk is true but this build has no ZED SDK support");
        throw std::runtime_error("vision was built without the ZED SDK");
#endif
    }

    if (enable_object_detection_) {
        RfDetrEngineOptions opts;
        opts.model_dir = model_path;
        opts.engine_builder = fs::path(ament_index_cpp::get_package_prefix("vision")) / "lib" / "vision" / "rfdetr_build";
        opts.log_info = [this](const std::string& msg) { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); };
        engine_ = std::make_unique<RfDetrEngine>(opts);
    } else {
        RCLCPP_INFO(get_logger(), "Object detection is disabled. Publishing raw feed only.");
    }

    create_ros_interfaces();
    RCLCPP_INFO(get_logger(), "%s initialized.", get_name());

    if (has_zed_sdk_) {
        grab_thread_ = std::thread(&FrontCamObjectDetectionNode::grab_loop, this);
    } else {
        RCLCPP_INFO(get_logger(), "ZED SDK disabled. Subscribing to %s", image_topic_.c_str());
        // Copy-decode to BGR, which is what the engine and the annotated publisher expect.
        if (compressed_) {
            compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_, queue_size_, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                    try {
                        image_callback(cv_bridge::toCvCopy(*msg, "bgr8")->image, msg->header.stamp);
                    } catch (const cv_bridge::Exception& e) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "CV Bridge error: %s", e.what());
                    }
                });
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, queue_size_, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    try {
                        image_callback(cv_bridge::toCvCopy(*msg, "bgr8")->image, msg->header.stamp);
                    } catch (const cv_bridge::Exception& e) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "CV Bridge error: %s", e.what());
                    }
                });
        }
    }

    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_set_parameters(params); });
}

FrontCamObjectDetectionNode::~FrontCamObjectDetectionNode() {
    running_ = false;
    if (grab_thread_.joinable()) grab_thread_.join();
#ifdef ZED_SDK_AVAILABLE
    if (zed_.isOpened()) zed_.close();
#endif
}

void FrontCamObjectDetectionNode::load_parameters() {
    class_names_ = required_param<std::vector<std::string>>("model_class_names");
    std::string names;
    for (const auto& n : class_names_) names += (names.empty() ? "" : ", ") + n;
    RCLCPP_INFO(get_logger(), "Class names: [%s]", names.c_str());

    declare_parameter<std::string>("model_relative_path.sim");
    declare_parameter<std::string>("model_relative_path.real");
    sim_ = required_param<bool>("sim");
    has_zed_sdk_ = required_param<bool>("has_zed_sdk");
    enable_object_detection_ = required_param<bool>("enable_object_detection");
    compressed_ = required_param<bool>("compressed");
    image_topic_ = required_param<std::string>("image_topic");
    detection_frame_topic_ = required_param<std::string>("detection_frame_topic");
    queue_size_ = static_cast<int>(required_param<int64_t>("queue_size"));
    conf_threshold_ = required_param<double>("model_detection_threshold");

    publish_annotated_image_ = required_param<bool>("publish_annotated");
    annotated_image_enabled_ = publish_annotated_image_;
    publish_annotated_every_n_frames_ = std::max<int>(1, required_param<int64_t>("publish_annotated_every_n_frames"));
    publish_camera_info_ = required_param<bool>("publish_camera_info");
    publish_depth_image_ = required_param<bool>("publish_depth");
    publish_depth_compressed_ = required_param<bool>("publish_depth_compressed");
    publish_depth_every_n_frames_ = std::max<int>(1, required_param<int64_t>("publish_depth_every_n_frames"));
    collect_depth_image_ = required_param<bool>("enable_depth_collection");

    collection_dir_ = required_param<std::string>("collection_image_dir");
    depth_collection_dir_ = required_param<std::string>("collection_depth_dir");
    collection_interval_ = required_param<double>("collection_interval_seconds");
    collection_extension_ = required_param<std::string>("collection_file_extension");

    // Refraction parameters are re-read every frame (they can be tuned live).
    declare_parameter<double>("depth_scale.sim");
    declare_parameter<double>("depth_scale.real");
    declare_parameter<double>("lateral_scale.sim");
    declare_parameter<double>("lateral_scale.real");
    declare_parameter<double>("depth_offset");

    enable_vio_ = required_param<bool>("enable_vio_pose");
    if (!has_zed_sdk_ && enable_vio_) {
        enable_vio_ = false;
        RCLCPP_WARN(get_logger(), "WARNING: %s disabling vio since has_zed_sdk is False.", get_name());
    }
    const std::string pose_source = required_param<std::string>("pose_source");
    if (pose_source != "zed_vio" && pose_source != "auv_pose") {
        throw std::invalid_argument("Unsupported pose source '" + pose_source + "'. Expected 'zed_vio' or 'auv_pose'.");
    }
    pose_source_auv_pose_ = pose_source == "auv_pose";
    if (!enable_vio_ && !pose_source_auv_pose_) {
        throw std::invalid_argument(
            "pose source='zed_vio' requires enable_vio=true. Either enable VIO or switch pose.source to 'auv_pose'.");
    }
    broadcast_vio_tf_ = required_param<bool>("broadcast_vio_tf");
    if (!enable_vio_ && broadcast_vio_tf_) {
        RCLCPP_WARN(get_logger(), "enable_vio is false but broadcast_vio_tf is true; VIO TF publication will be skipped.");
    }
    declare_parameter<std::string>("vio_pose_topic");
    auv_pose_topic_ = required_param<std::string>("auv_pose_topic");

    enable_gate_top_crop_ = required_param<bool>("gate_top_crop.enable");
    gate_top_crop_ratio_ = required_param<double>("gate_top_crop.ratio");
    enable_border_exclusion_ = required_param<bool>("border_exclusion.enable");
    border_exclusion_mode_ = required_param<std::string>("border_exclusion.mode");
    border_exclusion_margin_px_ = static_cast<int>(required_param<int64_t>("border_exclusion.margin_px"));
    border_exclusion_labels_ = required_param<std::vector<std::string>>("border_exclusion.labels");

    global_frame_id_ = required_param<std::string>("global_frame_id");
    declare_parameter<std::string>("auv_frame_id");
    vio_frame_id_ = required_param<std::string>("vio_frame_id");
    detection_frame_id_ = required_param<std::string>("detection_frame_id");
    image_frame_id_ = required_param<std::string>("image_frame_id");

    const auto vec3 = [this](const std::string& name) {
        const auto v = required_param<std::vector<double>>(name);
        if (v.size() != 3) throw std::invalid_argument("Parameter '" + name + "' must contain exactly 3 values.");
        return v;
    };
    const Eigen::Matrix4d T_auv_camera_center =
        transform_from_xyz_rpy(vec3("auv_to_camera_center.xyz"), vec3("auv_to_camera_center.rpy"));
    const Eigen::Matrix4d T_camera_center_detection =
        transform_from_xyz_rpy(vec3("camera_center_to_detection.xyz"), vec3("camera_center_to_detection.rpy"));
    T_auv_detection_ = T_auv_camera_center * T_camera_center_detection;
    T_detection_auv_ = T_auv_detection_.inverse();

    // Depth estimation
    for (const auto& label : class_names_) {
        const std::string mode = declare_parameter<std::string>("depth_estimation.mode_by_label." + label, "");
        if (!mode.empty()) depth_cfg_.mode_by_label[label] = mode;
        const double height = declare_parameter<double>("depth_estimation.known_heights." + label, 0.0);
        if (height > 0.0) depth_cfg_.known_heights[label] = height;
    }
    depth_cfg_.default_mode = declare_parameter<std::string>("depth_estimation.default_mode", "depth_map_median");
    depth_cfg_.known_planes["surface"] = declare_parameter<double>("depth_estimation.known_planes.surface", 0.0);
    depth_cfg_.known_planes["floor"] = declare_parameter<double>("depth_estimation.known_planes.floor", -4.0);
    depth_cfg_.max_pitch_deg = declare_parameter<double>("depth_estimation.known_height_angle_limits.max_pitch_deg", 20.0);
    depth_cfg_.max_roll_deg = declare_parameter<double>("depth_estimation.known_height_angle_limits.max_roll_deg", 20.0);
    depth_cfg_.depth_map_shrink_ratio = declare_parameter<double>("depth_estimation.depth_map_shrink_ratio", 0.8);
    depth_cfg_.depth_map_nearest_percentile = declare_parameter<double>("depth_estimation.depth_map_nearest_percentile", 15.0);
    depth_cfg_.depth_map_min_valid_fraction = declare_parameter<double>("depth_estimation.depth_map_min_valid_fraction", 0.05);
    depth_cfg_.border_margin_px = border_exclusion_margin_px_;

    // ZED parameters are read in open_zed(); declare them here so YAML overrides always validate.
    for (const char* name : {"zed_depth_confidence_threshold", "zed_depth_stabilization", "zed_sdk_verbose",
                             "zed_sdk_gpu_id", "zed_image_brightness", "zed_image_contrast", "zed_image_hue",
                             "zed_image_saturation", "zed_image_sharpness", "zed_image_gamma", "zed_image_exposure",
                             "zed_image_gain", "zed_image_white_balance_temperature", "zed_led_status",
                             "zed_fps.sim", "zed_fps.real", "stream.port"}) {
        declare_parameter<int64_t>(name);
    }
    for (const char* name : {"zed_depth_maximum_distance", "zed_depth_minimum_distance",
                             "zed_depth_positional_tracking_min_range", "zed_open_timeout_sec",
                             "zed_grab_compute_capping_fps"}) {
        declare_parameter<double>(name);
    }
    for (const char* name : {"zed_self_calib", "zed_enable_right_side_measure", "zed_enable_image_enhancement",
                             "zed_async_grab_camera_recovery", "zed_enable_image_validity_check",
                             "zed_image_auto_exposure", "zed_image_auto_white_balance"}) {
        declare_parameter<bool>(name);
    }
    for (const char* name : {"zed_depth_mode", "zed_resolution.sim", "zed_resolution.real",
                             "zed_optional_opencv_calibration_file", "stream.ip"}) {
        declare_parameter<std::string>(name);
    }
    declare_parameter<std::string>("zed_flip_mode", "OFF");
    declare_parameter<std::string>("zed_positional_tracking_mode.sim", "GEN_1");
    declare_parameter<std::string>("zed_positional_tracking_mode.real", "GEN_3");
    declare_parameter<int64_t>("zed_retrieval_width", 0);
    declare_parameter<int64_t>("zed_retrieval_height", 0);
}

void FrontCamObjectDetectionNode::create_ros_interfaces() {
    RCLCPP_INFO(get_logger(), "Setting QOL queue size to: %d", queue_size_);
    pub_detection_frame_ = create_publisher<auv_msgs::msg::VisionDetectionFrame>(detection_frame_topic_, queue_size_);

    if (publish_camera_info_) {
        const std::string topic = detection_frame_topic_ + "/camera_info";
        pub_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(topic, queue_size_);
        RCLCPP_INFO(get_logger(), "Publishing camera info to: %s", topic.c_str());
    }

    if (enable_vio_) {
        const std::string topic = get_parameter("vio_pose_topic").as_string();
        pub_vio_pose_ = create_publisher<PoseStamped>(topic, 10);
        RCLCPP_INFO(get_logger(), "Publishing VIO pose measurement to: %s", topic.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "VIO disabled: skipping ZED positional tracking and VIO pose publication.");
    }

    if (pose_source_auv_pose_) {
        auv_pose_sub_ = create_subscription<PoseStamped>(
            auv_pose_topic_, rclcpp::SensorDataQoS(), [this](const PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(auv_pose_mutex_);
                auv_pose_msg_ = *msg;
            });
        RCLCPP_INFO(get_logger(), "Using AUV pose from: %s", auv_pose_topic_.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Using ZED VIO pose to derive synchronized AUV poses");
    }

    if (has_zed_sdk_) {
        // To use the ZED IMU instead of the hardware IMU, change `topic_in` of the
        // imu_processor in sensors_frames.yaml to this topic.
        pub_zed_imu_ = create_publisher<sensor_msgs::msg::Imu>("/sensors/zed/imu/data", rclcpp::SensorDataQoS());
    }

    RCLCPP_INFO(get_logger(), "Publishing synchronized detection frames to: %s", detection_frame_topic_.c_str());

    if (publish_annotated_image_) {
        const std::string topic = detection_frame_topic_ + "/annotated" + (compressed_ ? "/compressed" : "");
        if (compressed_) {
            pub_annotated_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic, queue_size_);
        } else {
            pub_annotated_raw_ = create_publisher<sensor_msgs::msg::Image>(topic, queue_size_);
        }
        RCLCPP_INFO(get_logger(), "Annotated debug image topic ready at: %s (enabled=%s)", topic.c_str(),
                    annotated_image_enabled_ ? "true" : "false");
    }

    if (publish_depth_image_) {
        std::string topic = detection_frame_topic_ + "/depth";
        if (publish_depth_compressed_) {
            topic += "/compressed";
            pub_depth_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic, queue_size_);
        } else {
            pub_depth_raw_ = create_publisher<sensor_msgs::msg::Image>(topic, queue_size_);
        }
        RCLCPP_INFO(get_logger(), "Publishing depth image to: %s", topic.c_str());
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    using std::placeholders::_1;
    using std::placeholders::_2;
    services_.push_back(create_service<auv_msgs::srv::AutomaticCapture>(
        "/vision/front_cam/toggle_collection", std::bind(&FrontCamObjectDetectionNode::toggle_collection, this, _1, _2)));
    services_.push_back(create_service<std_srvs::srv::Trigger>(
        "/vision/front_cam/toggle_annotated_image", std::bind(&FrontCamObjectDetectionNode::toggle_annotated, this, _1, _2)));
    services_.push_back(create_service<std_srvs::srv::Trigger>(
        "/vision/front_cam/reset_vio_pose", std::bind(&FrontCamObjectDetectionNode::reset_vio_pose, this, _1, _2)));
    services_.push_back(create_service<std_srvs::srv::Trigger>(
        "/vision/front_cam/get_camera_settings", std::bind(&FrontCamObjectDetectionNode::get_camera_settings, this, _1, _2)));
}

// ---------------------------------------------------------------------------
// ZED setup
// ---------------------------------------------------------------------------
#ifdef ZED_SDK_AVAILABLE

void FrontCamObjectDetectionNode::open_zed() {
    double max_depth = get_parameter("zed_depth_maximum_distance").as_double();
    if (max_depth > 40.0) {
        RCLCPP_WARN(get_logger(), "zed_depth_maximum_distance capped from %.2f to 40.0 (ZED maximum)", max_depth);
        max_depth = 40.0;
    }
    double min_depth = get_parameter("zed_depth_minimum_distance").as_double();
    if (min_depth < 0.2) {
        RCLCPP_WARN(get_logger(), "zed_depth_minimum_distance capped from %.2f to 0.2 (ZED minimum)", min_depth);
        min_depth = 0.2;
    }
    double tracking_min_range = get_parameter("zed_depth_positional_tracking_min_range").as_double();
    if (tracking_min_range < 0.2) {
        RCLCPP_WARN(get_logger(), "zed_depth_positional_tracking_min_range capped from %.2f to 0.2 (ZED minimum)",
                    tracking_min_range);
        tracking_min_range = 0.2;
    }
    const int depth_stabilization = static_cast<int>(get_parameter("zed_depth_stabilization").as_int());
    if (!enable_vio_ && depth_stabilization != 0) {
        RCLCPP_WARN(get_logger(), "enable_vio is false, so zed_depth_stabilization will be forced to 0 "
                                  "to avoid implicit positional tracking enablement.");
    }
    const int stream_port = static_cast<int>(get_parameter("stream.port").as_int());

    sl::InitParameters init;
    init.sdk_verbose = static_cast<int>(get_parameter("zed_sdk_verbose").as_int());
    init.sdk_gpu_id = static_cast<int>(get_parameter("zed_sdk_gpu_id").as_int());
    init.camera_resolution = zed_enum_from_name<sl::RESOLUTION>(
        get_parameter(sim_ ? "zed_resolution.sim" : "zed_resolution.real").as_string(), "zed_camera_resolution");
    init.camera_fps = static_cast<int>(get_parameter(sim_ ? "zed_fps.sim" : "zed_fps.real").as_int());
    init.camera_disable_self_calib = !get_parameter("zed_self_calib").as_bool();
    init.camera_image_flip = flip_mode_from_name(get_parameter("zed_flip_mode").as_string());
    init.enable_right_side_measure = get_parameter("zed_enable_right_side_measure").as_bool();
    init.coordinate_units = sl::UNIT::METER;
    init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init.depth_mode = zed_enum_from_name<sl::DEPTH_MODE>(get_parameter("zed_depth_mode").as_string(), "zed_depth_mode");
    init.depth_stabilization = enable_vio_ ? depth_stabilization : 0;
    init.depth_maximum_distance = static_cast<float>(max_depth);
    init.depth_minimum_distance = static_cast<float>(min_depth);
    init.enable_image_enhancement = get_parameter("zed_enable_image_enhancement").as_bool();
    init.open_timeout_sec = static_cast<float>(get_parameter("zed_open_timeout_sec").as_double());
    init.async_grab_camera_recovery = get_parameter("zed_async_grab_camera_recovery").as_bool();
    init.grab_compute_capping_fps = static_cast<float>(get_parameter("zed_grab_compute_capping_fps").as_double());
    init.enable_image_validity_check = get_parameter("zed_enable_image_validity_check").as_bool();
    const std::string calib_file = get_parameter("zed_optional_opencv_calibration_file").as_string();
    if (!calib_file.empty()) init.optional_opencv_calibration_file = calib_file.c_str();
    if (sim_) {
        init.input.setFromStream(get_parameter("stream.ip").as_string().c_str(), stream_port);
    }

    sl::ERROR_CODE result = sl::ERROR_CODE::FAILURE;
    for (int i = 0; i < 5; ++i) {
        result = zed_.open(init);
        if (result == sl::ERROR_CODE::SUCCESS) break;
        RCLCPP_ERROR(get_logger(), "Failed to open zed camera:, retrying %d more times: %s", 5 - i, zed_error(result).c_str());
        zed_.close();
        std::this_thread::sleep_for(2s);
    }
    if (result != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_FATAL(get_logger(), "Failed to open zed camera after 5 attempts");
        throw std::runtime_error("Failed to open ZED camera after 5 attempts.");
    }

    apply_zed_camera_controls();

    // Open at high resolution for better depth, retrieve smaller for inference & publishing.
    // Only applies to the real camera — sim streams at a fixed resolution.
    const int retrieval_w = static_cast<int>(get_parameter("zed_retrieval_width").as_int());
    const int retrieval_h = static_cast<int>(get_parameter("zed_retrieval_height").as_int());
    if (!sim_ && retrieval_w > 0 && retrieval_h > 0) {
        const auto native = zed_.getCameraInformation().camera_configuration.resolution;
        retrieval_resolution_ = sl::Resolution(retrieval_w, retrieval_h);
        native_width_ = static_cast<int>(native.width);
        native_height_ = static_cast<int>(native.height);
        bbox_scale_x_ = static_cast<double>(native_width_) / retrieval_w;
        bbox_scale_y_ = static_cast<double>(native_height_) / retrieval_h;
        RCLCPP_INFO(get_logger(), "Retrieval scaling enabled: grab %dx%d -> retrieve %dx%d (bbox scale %.3fx, %.3fx)",
                    native_width_, native_height_, retrieval_w, retrieval_h, bbox_scale_x_, bbox_scale_y_);
    }

    if (enable_vio_) {
        sl::PositionalTrackingParameters pos;
        pos.mode = zed_enum_from_name<sl::POSITIONAL_TRACKING_MODE>(
            get_parameter(sim_ ? "zed_positional_tracking_mode.sim" : "zed_positional_tracking_mode.real").as_string(),
            "zed_positional_tracking_mode");
        pos.enable_imu_fusion = true;
        pos.set_floor_as_origin = false;
        pos.enable_area_memory = false;
        pos.depth_min_range = static_cast<float>(tracking_min_range);
        // Offset the camera from the AUV origin so VIO starts at the AUV pose.
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                zed_initial_transform_(r, c) = static_cast<float>(T_auv_detection_(r, c));
            }
        }
        pos.initial_world_transform = zed_initial_transform_;
        const auto err = zed_.enablePositionalTracking(pos);
        if (err != sl::ERROR_CODE::SUCCESS) {
            zed_.close();
            throw std::runtime_error("Failed to enable ZED positional tracking: " + zed_error(err));
        }
    }

    // Custom-box object detection is only used to get ZED 3D points for zed_3d labels.
    sl::ObjectDetectionParameters obj_params;
    obj_params.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    obj_params.enable_tracking = false;
    obj_params.enable_segmentation = false;
    obj_params.filtering_mode = sl::OBJECT_FILTERING_MODE::NONE;
    const auto od_err = zed_.enableObjectDetection(obj_params);
    if (od_err != sl::ERROR_CODE::SUCCESS) {
        zed_.close();
        throw std::runtime_error("Failed to enable ZED object detection: " + zed_error(od_err));
    }
    obj_runtime_params_.object_detection_properties.detection_confidence_threshold = 0;
    obj_runtime_params_.object_detection_properties.is_grounded = false;
    obj_runtime_params_.object_detection_properties.is_static = true;

    runtime_params_.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    runtime_params_.confidence_threshold = static_cast<int>(get_parameter("zed_depth_confidence_threshold").as_int());
    runtime_params_.texture_confidence_threshold = 100;

    if (!sim_) {
        sl::StreamingParameters stream;
        stream.codec = sl::STREAMING_CODEC::H264;
        stream.port = static_cast<unsigned short>(stream_port);
        stream.bitrate = 8000;  // kbps
        const auto err = zed_.enableStreaming(stream);
        if (err != sl::ERROR_CODE::SUCCESS) {
            zed_.close();
            throw std::runtime_error("Failed to enable ZED streaming: " + zed_error(err));
        }
    }

    build_camera_info_template();
}

void FrontCamObjectDetectionNode::apply_zed_camera_controls() {
    const auto set = [this](const std::string& name, sl::VIDEO_SETTINGS setting, int value) {
        const auto err = zed_.setCameraSettings(setting, value);
        if (err != sl::ERROR_CODE::SUCCESS) {
            zed_.close();
            throw std::runtime_error("Failed to set " + name + "=" + std::to_string(value) + " on the ZED camera: "
                                     + zed_error(err));
        }
        return name + "=" + std::to_string(value);
    };

    std::string applied;
    for (const auto& [name, setting] : zed_runtime_camera_settings()) {
        const auto param = get_parameter(name);
        const int value = param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? static_cast<int>(param.as_bool())
                                                                                    : static_cast<int>(param.as_int());
        applied += (applied.empty() ? "" : ", ") + set(name, setting, value);
    }
    if (!get_parameter("zed_image_auto_exposure").as_bool()) {
        applied += ", " + set("zed_image_gain", sl::VIDEO_SETTINGS::GAIN,
                              static_cast<int>(get_parameter("zed_image_gain").as_int()));
        applied += ", " + set("zed_image_exposure", sl::VIDEO_SETTINGS::EXPOSURE,
                              static_cast<int>(get_parameter("zed_image_exposure").as_int()));
    }
    if (!get_parameter("zed_image_auto_white_balance").as_bool()) {
        applied += ", " + set("zed_image_white_balance_temperature", sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE,
                              static_cast<int>(get_parameter("zed_image_white_balance_temperature").as_int()));
    }
    RCLCPP_INFO(get_logger(), "Configured ZED camera: resolution=%s fps=%ld flip=%s | controls: %s",
                get_parameter(sim_ ? "zed_resolution.sim" : "zed_resolution.real").as_string().c_str(),
                get_parameter(sim_ ? "zed_fps.sim" : "zed_fps.real").as_int(),
                get_parameter("zed_flip_mode").as_string().c_str(), applied.c_str());
}

void FrontCamObjectDetectionNode::build_camera_info_template() {
    const auto config = zed_.getCameraInformation().camera_configuration;
    const auto& calib = config.calibration_parameters.left_cam;

    double fx = calib.fx, fy = calib.fy, cx = calib.cx, cy = calib.cy;
    int width = static_cast<int>(config.resolution.width);
    int height = static_cast<int>(config.resolution.height);
    // With retrieval scaling, scale intrinsics so CameraInfo, images and projection math agree.
    if (retrieval_resolution_.width > 0) {
        const double sx = 1.0 / bbox_scale_x_;
        const double sy = 1.0 / bbox_scale_y_;
        fx *= sx;
        fy *= sy;
        cx *= sx;
        cy *= sy;
        width = static_cast<int>(retrieval_resolution_.width);
        height = static_cast<int>(retrieval_resolution_.height);
    }
    intrinsics_ = {fx, fy, cx, cy};

    auto& info = camera_info_template_;
    info.header.frame_id = image_frame_id_;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d.assign(std::begin(calib.disto), std::end(calib.disto));
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
}

void FrontCamObjectDetectionNode::publish_zed_imu(const builtin_interfaces::msg::Time& stamp) {
    sl::SensorsData sensors;
    if (zed_.getSensorsData(sensors, sl::TIME_REFERENCE::IMAGE) != sl::ERROR_CODE::SUCCESS) return;

    const auto& imu = sensors.imu;
    const auto orientation = imu.pose.getOrientation();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = image_frame_id_;
    msg.orientation.x = orientation.x;
    msg.orientation.y = orientation.y;
    msg.orientation.z = orientation.z;
    msg.orientation.w = orientation.w;
    // ZED reports deg/s; Z is flipped to match the ROS frame (same as the Python node).
    constexpr double kDegToRad = M_PI / 180.0;
    msg.angular_velocity.x = imu.angular_velocity.x * kDegToRad;
    msg.angular_velocity.y = imu.angular_velocity.y * kDegToRad;
    msg.angular_velocity.z = -imu.angular_velocity.z * kDegToRad;
    msg.linear_acceleration.x = imu.linear_acceleration.x;
    msg.linear_acceleration.y = imu.linear_acceleration.y;
    msg.linear_acceleration.z = imu.linear_acceleration.z;
    pub_zed_imu_->publish(msg);
}

#endif  // ZED_SDK_AVAILABLE

// ---------------------------------------------------------------------------
// Frame sources
// ---------------------------------------------------------------------------

void FrontCamObjectDetectionNode::grab_loop() {
#ifdef ZED_SDK_AVAILABLE
    while (rclcpp::ok() && running_) {
        const auto t_start = std::chrono::steady_clock::now();

        const auto result = zed_.grab(runtime_params_);
        if (result == sl::ERROR_CODE::CORRUPTED_FRAME) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Corrupted frame");
            continue;
        }
        if (result != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Zed.grab() failed: %s",
                                 zed_error(result).c_str());
            // Don't spin a core at 100% while the camera is unplugged or restarting.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        const auto stamp = now();
        const auto vio_pose = enable_vio_ ? vio_world_pose_snapshot(stamp) : std::nullopt;

        if (pub_zed_imu_) publish_zed_imu(stamp);

        const auto health = zed_.getHealthStatus();
        if (health.low_image_quality) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Low image quality");
        }
        if (health.low_lighting) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Low lighting conditions");
        }

        zed_.retrieveImage(image_buffer_, sl::VIEW::LEFT, sl::MEM::CPU, retrieval_resolution_);
        const cv::Mat bgra(static_cast<int>(image_buffer_.getHeight()), static_cast<int>(image_buffer_.getWidth()),
                           CV_8UC4, image_buffer_.getPtr<sl::uchar1>(sl::MEM::CPU),
                           image_buffer_.getStepBytes(sl::MEM::CPU));
        cv::Mat img;
        cv::cvtColor(bgra, img, cv::COLOR_BGRA2BGR);
        ++frame_counter_;

        // Depth is always retrieved: the depth_map_* modes need it.
        zed_.retrieveMeasure(depth_buffer_, sl::MEASURE::DEPTH, sl::MEM::CPU, retrieval_resolution_);
        const cv::Mat depth(static_cast<int>(depth_buffer_.getHeight()), static_cast<int>(depth_buffer_.getWidth()),
                            CV_32FC1, depth_buffer_.getPtr<sl::float1>(sl::MEM::CPU),
                            depth_buffer_.getStepBytes(sl::MEM::CPU));

        bool is_collection_frame = false;
        {
            std::lock_guard<std::mutex> lock(collection_mutex_);
            is_collection_frame = collecting_ && (wall_time_sec() - last_collection_time_) >= collection_interval_;
        }
        try {
            process_frame(img, stamp, &depth, vio_pose, is_collection_frame, t_start);
        } catch (const std::exception& e) {
            // An exception escaping this thread would std::terminate the whole node; drop the frame instead.
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "Frame processing failed: %s", e.what());
        }
    }
#endif
}

void FrontCamObjectDetectionNode::image_callback(const cv::Mat& bgr, const builtin_interfaces::msg::Time& stamp) {
    const auto t_start = std::chrono::steady_clock::now();
    ++frame_counter_;
    bool is_collection_frame = false;
    {
        std::lock_guard<std::mutex> lock(collection_mutex_);
        is_collection_frame = collecting_ && (wall_time_sec() - last_collection_time_) >= collection_interval_;
    }
    process_frame(bgr, stamp, nullptr, std::nullopt, is_collection_frame, t_start);
}

// ---------------------------------------------------------------------------
// Poses
// ---------------------------------------------------------------------------

std::optional<geometry_msgs::msg::PoseStamped> FrontCamObjectDetectionNode::vio_world_pose_snapshot(
    const builtin_interfaces::msg::Time& stamp) {
#ifdef ZED_SDK_AVAILABLE
    if (!enable_vio_) return std::nullopt;

    sl::Pose cam_pose;
    const auto state = zed_.getPosition(cam_pose, sl::REFERENCE_FRAME::WORLD);
    if (state != sl::POSITIONAL_TRACKING_STATE::OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "VIO tracking not OK: %s", sl::toString(state).c_str());
        return std::nullopt;
    }

    double sensor_z = 0.0;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        sensor_z = sensor_depth_;
    }

    // The pressure sensor overrides VIO Z.
    const auto translation = cam_pose.getTranslation();
    auto rotation = cam_pose.getRotationMatrix();  // sl::Matrix3f::operator() is non-const
    Eigen::Matrix4d T_world_detection = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            T_world_detection(r, c) = rotation(r, c);
        }
    }
    T_world_detection.block<3, 1>(0, 3) = Eigen::Vector3d(translation.x, translation.y, -sensor_z);
    return pose_from_transform(T_world_detection * T_detection_auv_, global_frame_id_, stamp);
#else
    (void)stamp;
    return std::nullopt;
#endif
}

std::optional<geometry_msgs::msg::PoseStamped> FrontCamObjectDetectionNode::detection_frame_auv_pose(
    const builtin_interfaces::msg::Time& stamp, const std::optional<PoseStamped>& vio_pose) {
    if (pose_source_auv_pose_) {
        std::lock_guard<std::mutex> lock(auv_pose_mutex_);
        if (!auv_pose_msg_) return std::nullopt;
        PoseStamped pose = *auv_pose_msg_;
        pose.header.stamp = stamp;
        if (pose.header.frame_id.empty()) pose.header.frame_id = global_frame_id_;
        return pose;
    }
    return vio_pose ? vio_pose : vio_world_pose_snapshot(stamp);
}

// ---------------------------------------------------------------------------
// Per-frame pipeline
// ---------------------------------------------------------------------------

void FrontCamObjectDetectionNode::process_frame(const cv::Mat& img, const builtin_interfaces::msg::Time& stamp,
                                                const cv::Mat* depth, const std::optional<PoseStamped>& vio_pose,
                                                bool is_collection_frame,
                                                std::chrono::steady_clock::time_point t_start) {
    if (vio_pose && pub_vio_pose_) {
        // Publish VIO continuously, even without detections.
        pub_vio_pose_->publish(*vio_pose);
        if (broadcast_vio_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = global_frame_id_;
            t.child_frame_id = vio_frame_id_;
            t.transform.translation.x = vio_pose->pose.position.x;
            t.transform.translation.y = vio_pose->pose.position.y;
            t.transform.translation.z = vio_pose->pose.position.z;
            t.transform.rotation = vio_pose->pose.orientation;
            tf_broadcaster_->sendTransform(t);
        }
    }

    const auto auv_pose = detection_frame_auv_pose(stamp, vio_pose);
    if (!auv_pose) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for AUV pose on %s", auv_pose_topic_.c_str());
        return;
    }

    if (is_collection_frame) save_collection_frame(img, depth);

    if (publish_depth_image_ && depth != nullptr && frame_counter_ % publish_depth_every_n_frames_ == 0) {
        publish_depth(*depth, stamp);
    }

    if (has_zed_sdk_ && pub_camera_info_) {
        auto info = camera_info_template_;
        info.header.stamp = stamp;
        pub_camera_info_->publish(info);
    }

    std::vector<Detection2D> detections;
    if (engine_) {
        try {
            detections = engine_->detect(img, static_cast<float>(conf_threshold_));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Inference failed: %s", e.what());
        }
        // Drop ids the class list doesn't know about (the Python node skipped them everywhere).
        detections.erase(std::remove_if(detections.begin(), detections.end(),
                                        [this](const Detection2D& d) {
                                            return d.class_id < 0 || d.class_id >= static_cast<int>(class_names_.size());
                                        }),
                         detections.end());
        if (enable_border_exclusion_) detections = filter_border_detections(std::move(detections), img.cols, img.rows);
    }

    auv_msgs::msg::VisionDetectionFrame frame;
    frame.header.stamp = stamp;
    frame.header.frame_id = detection_frame_id_;
    frame.auv_pose = *auv_pose;
    frame.detections = localize_detections(detections, img, depth, *auv_pose);
    pub_detection_frame_->publish(frame);

    // Always publish the (possibly annotated) image, even with no detections.
    if (publish_annotated_image_ && frame_counter_ % publish_annotated_every_n_frames_ == 0) {
        publish_annotated(img, detections, stamp);
    }

    const double latency_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    RCLCPP_DEBUG(get_logger(), "Detection latency: %.1f ms | Active 3D detections: %zu", latency_ms,
                 frame.detections.size());
}

std::vector<Detection2D> FrontCamObjectDetectionNode::filter_border_detections(std::vector<Detection2D> detections,
                                                                               int img_w, int img_h) const {
    const int margin = border_exclusion_margin_px_;
    const bool horizontal = border_exclusion_mode_ == "both" || border_exclusion_mode_ == "horizontal";
    const bool vertical = border_exclusion_mode_ == "both" || border_exclusion_mode_ == "vertical";

    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [&](const Detection2D& d) {
                           const std::string& label = class_names_[d.class_id];
                           if (std::find(border_exclusion_labels_.begin(), border_exclusion_labels_.end(), label)
                               == border_exclusion_labels_.end()) {
                               return false;
                           }
                           const auto& b = d.bbox;
                           return (horizontal && (b.x1 <= margin || b.x2 >= img_w - margin))
                                  || (vertical && (b.y1 <= margin || b.y2 >= img_h - margin));
                       }),
        detections.end());
    return detections;
}

std::vector<auv_msgs::msg::VisionDetection> FrontCamObjectDetectionNode::localize_detections(
    const std::vector<Detection2D>& detections, const cv::Mat& img, const cv::Mat* depth,
    const PoseStamped& auv_pose) {
    std::vector<auv_msgs::msg::VisionDetection> out;
    if (detections.empty()) return out;

    const auto fill_2d = [&](auv_msgs::msg::VisionDetection& msg, const Detection2D& d, const BBoxXYXY& b) {
        msg.label = class_names_[d.class_id];
        msg.confidence = d.score;
        const double size_x = std::max(0.0, b.x2 - b.x1);
        const double size_y = std::max(0.0, b.y2 - b.y1);
        msg.bbox_center_x = b.x1 + size_x / 2.0;
        msg.bbox_center_y = b.y1 + size_y / 2.0;
        msg.bbox_size_x = size_x;
        msg.bbox_size_y = size_y;
    };

    if (!has_zed_sdk_) {
        // No depth or intrinsics without the ZED: publish 2D detections only.
        for (const auto& d : detections) {
            auv_msgs::msg::VisionDetection msg;
            fill_2d(msg, d, d.bbox);
            out.push_back(msg);
        }
        return out;
    }

#ifdef ZED_SDK_AVAILABLE
    // 1. Crop gate boxes and collect ZED custom boxes for zed_3d labels.
    std::vector<BBoxXYXY> boxes;
    std::vector<sl::CustomBoxObjectData> custom_boxes;
    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& d = detections[i];
        const std::string& label = class_names_[d.class_id];
        BBoxXYXY b = d.bbox;

        // The gate's legs reach into noisy stereo territory: keep only the top bar for depth.
        if (enable_gate_top_crop_ && label == "gate") {
            b.y2 = b.y1 + (b.y2 - b.y1) * gate_top_crop_ratio_;
        }
        boxes.push_back(b);

        if (depth_cfg_.mode_for(label) == "zed_3d") {
            sl::CustomBoxObjectData box;
            box.probability = d.score;
            box.label = d.class_id;
            box.is_grounded = false;
            box.unique_object_id = sl::String(("front_cam_det_" + std::to_string(i)).c_str());

            // Upscale from retrieval resolution to the native grab resolution and clamp.
            const int clamp_w = native_width_ > 0 ? native_width_ : img.cols;
            const int clamp_h = native_height_ > 0 ? native_height_ : img.rows;
            const auto cx = [&](double x) { return static_cast<unsigned>(std::clamp(static_cast<int>(x * bbox_scale_x_), 0, clamp_w - 1)); };
            const auto cy = [&](double y) { return static_cast<unsigned>(std::clamp(static_cast<int>(y * bbox_scale_y_), 0, clamp_h - 1)); };
            // ZED order: top-left, top-right, bottom-right, bottom-left.
            box.bounding_box_2d = {sl::uint2(cx(b.x1), cy(b.y1)), sl::uint2(cx(b.x2), cy(b.y1)),
                                   sl::uint2(cx(b.x2), cy(b.y2)), sl::uint2(cx(b.x1), cy(b.y2))};
            custom_boxes.push_back(box);
        }
    }

    // 2. Retrieve ZED 3D points for zed_3d labels.
    std::unordered_map<std::string, ZedObject3D> zed_objects;
    if (!custom_boxes.empty()) {
        zed_.ingestCustomBoxObjects(custom_boxes);
        sl::Objects objects;
        zed_.retrieveCustomObjects(objects, obj_runtime_params_);
        for (const auto& obj : objects.object_list) {
            ZedObject3D z;
            z.position = Eigen::Vector3d(obj.position.x, obj.position.y, obj.position.z);
            for (int k = 0; k < 6; ++k) z.covariance[k] = obj.position_covariance[k];
            zed_objects[obj.unique_object_id.c_str()] = z;
        }
    }

    // 3. Camera-frame 3D positions.
    RefractionParams refraction;
    refraction.depth_scale = get_parameter(sim_ ? "depth_scale.sim" : "depth_scale.real").as_double();
    refraction.lateral_scale = get_parameter(sim_ ? "lateral_scale.sim" : "lateral_scale.real").as_double();
    refraction.depth_offset = get_parameter("depth_offset").as_double();

    const Eigen::Matrix4d T_world_auv = transform_from_pose(auv_pose.pose);
    const Eigen::Matrix4d T_world_camera = T_world_auv * T_auv_detection_;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tf2::Quaternion(auv_pose.pose.orientation.x, auv_pose.pose.orientation.y,
                                   auv_pose.pose.orientation.z, auv_pose.pose.orientation.w))
        .getRPY(roll, pitch, yaw);

    for (std::size_t i = 0; i < detections.size(); ++i) {
        Compute3DInput in;
        in.label = class_names_[detections[i].class_id];
        in.bbox = boxes[i];
        in.img_h = img.rows;
        in.T_world_camera = T_world_camera;
        in.auv_roll = roll;
        in.auv_pitch = pitch;
        in.depth = depth;
        const auto it = zed_objects.find("front_cam_det_" + std::to_string(i));
        if (it != zed_objects.end()) in.zed_object = it->second;

        const auto pos = compute_3d_position(in, depth_cfg_, intrinsics_, refraction);
        if (!pos) continue;

        auv_msgs::msg::VisionDetection msg;
        fill_2d(msg, detections[i], boxes[i]);
        msg.pose_camera.pose.position.x = pos->position.x();
        msg.pose_camera.pose.position.y = pos->position.y();
        msg.pose_camera.pose.position.z = pos->position.z();
        msg.pose_camera.pose.orientation.w = 1.0;
        msg.pose_camera.covariance.fill(0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                msg.pose_camera.covariance[r * 6 + c] = pos->covariance(r, c);
            }
        }
        out.push_back(msg);
    }
#else
    (void)img;
    (void)depth;
    (void)auv_pose;
#endif
    return out;
}

void FrontCamObjectDetectionNode::publish_annotated(const cv::Mat& img, const std::vector<Detection2D>& detections,
                                                    const builtin_interfaces::msg::Time& stamp) {
    cv::Mat output = img.clone();
    if (annotated_image_enabled_ && !detections.empty()) {
        // Scale line/text with resolution, like supervision's optimal scale helpers.
        const int thickness = std::max(1, static_cast<int>(std::round(std::min(img.cols, img.rows) / 400.0)) + 1);
        const double text_scale = std::max(0.4, std::min(img.cols, img.rows) / 900.0);
        for (const auto& d : detections) {
            const cv::Point tl(static_cast<int>(d.bbox.x1), static_cast<int>(d.bbox.y1));
            const cv::Point br(static_cast<int>(d.bbox.x2), static_cast<int>(d.bbox.y2));
            const cv::Scalar color = cv::Scalar((d.class_id * 67) % 256, (d.class_id * 137 + 80) % 256, 255);
            cv::rectangle(output, tl, br, color, thickness);

            const std::string text = class_names_[d.class_id] + " " + cv::format("%.2f", d.score);
            int baseline = 0;
            const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, thickness, &baseline);
            const cv::Point text_org(tl.x, std::max(size.height + 4, tl.y - 4));
            cv::rectangle(output, text_org + cv::Point(-2, baseline), text_org + cv::Point(size.width + 2, -size.height - 4),
                          color, cv::FILLED);
            cv::putText(output, text, text_org, cv::FONT_HERSHEY_SIMPLEX, text_scale, cv::Scalar(0, 0, 0), thickness);
        }
    }

    try {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = image_frame_id_;
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

void FrontCamObjectDetectionNode::publish_depth(const cv::Mat& depth, const builtin_interfaces::msg::Time& stamp) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = image_frame_id_;
    if (pub_depth_compressed_) {
        // Encoded by hand: cv_bridge's C++ toCompressedImageMsg rejects mono16 input. Same output as Python's
        // cv2_to_compressed_imgmsg(depth_mm, dst_format='png').
        sensor_msgs::msg::CompressedImage msg;
        msg.header = header;
        msg.format = "png";
        cv::imencode(".png", depth_to_mm(depth), msg.data);
        pub_depth_compressed_->publish(msg);
    } else if (pub_depth_raw_) {
        cv_bridge::CvImage cv_img(header, "32FC1", depth);
        pub_depth_raw_->publish(*cv_img.toImageMsg());
    }
}

void FrontCamObjectDetectionNode::save_collection_frame(const cv::Mat& img, const cv::Mat* depth) {
    {
        std::lock_guard<std::mutex> lock(collection_mutex_);
        last_collection_time_ = wall_time_sec();
    }
    const std::string timestamp = collection_timestamp();
    cv::imwrite((fs::path(collection_dir_) / ("front_" + timestamp + collection_extension_)).string(), img);
    if (collect_depth_image_ && depth != nullptr) {
        cv::imwrite((fs::path(depth_collection_dir_) / ("depth_" + timestamp + ".png")).string(), depth_to_mm(*depth));
    }
    RCLCPP_INFO(get_logger(), "Collected dataset step: %s", timestamp.c_str());
}

// ---------------------------------------------------------------------------
// Parameters & services
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult FrontCamObjectDetectionNode::on_set_parameters(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
#ifdef ZED_SDK_AVAILABLE
    if (!has_zed_sdk_ || !zed_.isOpened()) return result;

    auto settings = zed_runtime_camera_settings();
    settings.emplace_back("zed_image_white_balance_temperature", sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
    settings.emplace_back("zed_image_exposure", sl::VIDEO_SETTINGS::EXPOSURE);

    for (const auto& param : params) {
        const auto it = std::find_if(settings.begin(), settings.end(),
                                     [&](const auto& s) { return s.first == param.get_name(); });
        if (it == settings.end()) continue;
        const int value = param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? static_cast<int>(param.as_bool())
                                                                                    : static_cast<int>(param.as_int());
        const auto err = zed_.setCameraSettings(it->second, value);
        if (err != sl::ERROR_CODE::SUCCESS) {
            result.successful = false;
            RCLCPP_ERROR(get_logger(), "Failed to dynamically set %s=%d: %s", param.get_name().c_str(), value,
                         zed_error(err).c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Dynamically updated ZED setting %s=%d", param.get_name().c_str(), value);
        }
    }
#else
    (void)params;
#endif
    return result;
}

void FrontCamObjectDetectionNode::toggle_collection(
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
        fs::create_directories(depth_collection_dir_);
        RCLCPP_INFO(get_logger(), "Image collection ENABLED every %.2fs → %s and %s", collection_interval_,
                    collection_dir_.c_str(), depth_collection_dir_.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Image collection DISABLED (interval preserved at %.2fs)", collection_interval_);
    }
    response->success = true;
    response->message = std::string("Collection ") + (collecting_ ? "enabled" : "disabled") + " with interval "
                        + cv::format("%.2f", collection_interval_) + "s";
}

void FrontCamObjectDetectionNode::toggle_annotated(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    annotated_image_enabled_ = !annotated_image_enabled_;
    response->success = true;
    response->message = std::string("Annotated image ") + (annotated_image_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void FrontCamObjectDetectionNode::reset_vio_pose(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "VIO is disabled or ZED SDK is not available.";
#ifdef ZED_SDK_AVAILABLE
    if (has_zed_sdk_ && enable_vio_) {
        const auto err = zed_.resetPositionalTracking(zed_initial_transform_);
        response->success = err == sl::ERROR_CODE::SUCCESS;
        response->message = response->success ? "VIO positional tracking reset successfully"
                                              : "Failed to reset VIO tracking: " + zed_error(err);
    }
#endif
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void FrontCamObjectDetectionNode::get_camera_settings(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "ZED camera not available.";
#ifdef ZED_SDK_AVAILABLE
    if (!has_zed_sdk_ || !zed_.isOpened()) return;

    auto settings = zed_runtime_camera_settings();
    settings.emplace_back("zed_image_white_balance_temperature", sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE);
    settings.emplace_back("zed_image_exposure", sl::VIDEO_SETTINGS::EXPOSURE);

    std::string message;
    for (const auto& [name, setting] : settings) {
        int value = 0;
        const auto err = zed_.getCameraSettings(setting, value);
        message += (message.empty() ? "" : " | ") + name + ": "
                   + (err == sl::ERROR_CODE::SUCCESS ? std::to_string(value) : "ERROR " + zed_error(err));
    }
    response->success = true;
    response->message = message;
#endif
}

}  // namespace vision
