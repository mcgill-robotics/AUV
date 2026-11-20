#pragma once

// C++ port of the former Python node vision/object_detection/front_cam_object_detection_node.py.
// Hardware mode (has_zed_sdk): grabs image + depth + VIO from the ZED SDK on a
// dedicated thread (real camera, or the simulator's ZED stream when sim:=true).
// Topic mode: subscribes to image_topic and publishes 2D-only detections.

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <auv_msgs/msg/vision_detection_frame.hpp>
#include <auv_msgs/srv/automatic_capture.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#ifdef ZED_SDK_AVAILABLE
#include <sl/Camera.hpp>
#endif

#include "detection_geometry.hpp"
#include "rfdetr_engine.hpp"

namespace vision {

class FrontCamObjectDetectionNode : public rclcpp::Node {
public:
    explicit FrontCamObjectDetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FrontCamObjectDetectionNode() override;

private:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    // --- setup ---
    void load_parameters();
    void create_ros_interfaces();
#ifdef ZED_SDK_AVAILABLE
    void open_zed();
    void apply_zed_camera_controls();
    void build_camera_info_template();
#endif

    // --- frame pipeline ---
    void grab_loop();
    void image_callback(const cv::Mat& bgr, const builtin_interfaces::msg::Time& stamp);
    void process_frame(const cv::Mat& img, const builtin_interfaces::msg::Time& stamp, const cv::Mat* depth,
                       const std::optional<PoseStamped>& vio_pose, bool is_collection_frame,
                       std::chrono::steady_clock::time_point t_start);
    std::vector<Detection2D> filter_border_detections(std::vector<Detection2D> detections, int img_w,
                                                      int img_h) const;
    std::vector<auv_msgs::msg::VisionDetection> localize_detections(
        const std::vector<Detection2D>& detections, const cv::Mat& img, const cv::Mat* depth,
        const PoseStamped& auv_pose);
    void publish_annotated(const cv::Mat& img, const std::vector<Detection2D>& detections,
                           const builtin_interfaces::msg::Time& stamp);
    void publish_depth(const cv::Mat& depth, const builtin_interfaces::msg::Time& stamp);
    void save_collection_frame(const cv::Mat& img, const cv::Mat* depth);

    // --- poses ---
    std::optional<PoseStamped> vio_world_pose_snapshot(const builtin_interfaces::msg::Time& stamp);
    std::optional<PoseStamped> detection_frame_auv_pose(const builtin_interfaces::msg::Time& stamp,
                                                        const std::optional<PoseStamped>& vio_pose);
#ifdef ZED_SDK_AVAILABLE
    void publish_zed_imu(const builtin_interfaces::msg::Time& stamp);
#endif

    // --- callbacks ---
    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter>& params);
    void toggle_collection(const std::shared_ptr<auv_msgs::srv::AutomaticCapture::Request> request,
                           std::shared_ptr<auv_msgs::srv::AutomaticCapture::Response> response);
    void toggle_annotated(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reset_vio_pose(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void get_camera_settings(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    template <typename T>
    T required_param(const std::string& name);

    // --- parameters ---
    std::vector<std::string> class_names_;
    bool sim_ = false;
    bool has_zed_sdk_ = false;
    bool enable_object_detection_ = true;
    bool compressed_ = false;
    std::string image_topic_;
    std::string detection_frame_topic_;
    int queue_size_ = 10;
    double conf_threshold_ = 0.5;

    bool publish_annotated_image_ = false;
    std::atomic<bool> annotated_image_enabled_{false};
    int publish_annotated_every_n_frames_ = 1;
    bool publish_camera_info_ = false;
    bool publish_depth_image_ = false;
    bool publish_depth_compressed_ = false;
    int publish_depth_every_n_frames_ = 1;
    bool collect_depth_image_ = false;

    bool enable_vio_ = false;
    bool broadcast_vio_tf_ = false;
    bool pose_source_auv_pose_ = false;
    std::string auv_pose_topic_;
    std::string global_frame_id_;
    std::string vio_frame_id_;
    std::string detection_frame_id_;
    std::string image_frame_id_;

    bool enable_gate_top_crop_ = false;
    double gate_top_crop_ratio_ = 1.0;
    bool enable_border_exclusion_ = false;
    std::string border_exclusion_mode_;
    int border_exclusion_margin_px_ = 0;
    std::vector<std::string> border_exclusion_labels_;

    DepthEstimationConfig depth_cfg_;
    CameraIntrinsics intrinsics_;
    Eigen::Matrix4d T_auv_detection_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_detection_auv_ = Eigen::Matrix4d::Identity();

    // Image collection (toggled via service)
    std::atomic<bool> collecting_{false};
    std::mutex collection_mutex_;
    double collection_interval_ = 2.0;
    double last_collection_time_ = 0.0;
    std::string collection_dir_;
    std::string depth_collection_dir_;
    std::string collection_extension_;
    std::uint64_t frame_counter_ = 0;

    // --- state shared with subscription callbacks ---
    std::mutex depth_mutex_;
    double sensor_depth_ = 0.0;
    std::mutex auv_pose_mutex_;
    std::optional<PoseStamped> auv_pose_msg_;

    std::unique_ptr<RfDetrEngine> engine_;

    // --- ROS interfaces ---
    rclcpp::Publisher<auv_msgs::msg::VisionDetectionFrame>::SharedPtr pub_detection_frame_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pub_vio_pose_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_zed_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_annotated_compressed_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_depth_compressed_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr auv_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    sensor_msgs::msg::CameraInfo camera_info_template_;

    std::thread grab_thread_;
    std::atomic<bool> running_{true};

#ifdef ZED_SDK_AVAILABLE
    sl::Camera zed_;
    sl::RuntimeParameters runtime_params_;
    sl::CustomObjectDetectionRuntimeParameters obj_runtime_params_;
    sl::Transform zed_initial_transform_;
    sl::Resolution retrieval_resolution_{0, 0};  // (0, 0) = native
    double bbox_scale_x_ = 1.0;
    double bbox_scale_y_ = 1.0;
    int native_width_ = 0;
    int native_height_ = 0;
    sl::Mat image_buffer_;
    sl::Mat depth_buffer_;
#endif
};

}  // namespace vision
