#pragma once

// C++ port of vision/object_detection/down_cam_object_detection_node.py.
// Real camera: grabs the USB down camera with cv::VideoCapture (V4L2, MJPG) on a dedicated thread.
// Sim: subscribes to image_topic.
// RF-DETR segmentation masks give each object's centroid and in-plane yaw; the 3D position comes from
// intersecting the pixel ray with the pool floor (or the configured height above it).

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <auv_msgs/msg/vision_detection_frame.hpp>
#include <auv_msgs/srv/automatic_capture.hpp>
#include <auv_msgs/srv/set_down_cam_projection_heights.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "detection_geometry.hpp"
#include "rfdetr_engine.hpp"

namespace vision {

class DownCamObjectDetectionNode : public rclcpp::Node {
public:
    explicit DownCamObjectDetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DownCamObjectDetectionNode() override;

private:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    // --- setup ---
    void load_parameters();
    void create_ros_interfaces();
    void open_camera();
    void apply_camera_controls();
    void build_camera_info_template();

    // --- frame pipeline ---
    void grab_loop();
    void process_frame(const cv::Mat& img, const builtin_interfaces::msg::Time& stamp);
    void publish_annotated(const cv::Mat& img, const std::vector<Detection2D>& detections,
                           const std::vector<std::array<cv::Point2f, 4>>& boxes,
                           const builtin_interfaces::msg::Time& stamp);

    // --- V4L2 helpers ---
    bool set_v4l2_ctl(const std::string& name, long value);
    std::string get_v4l2_ctl(const std::string& name);

    // --- callbacks ---
    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter>& params);
    void toggle_collection(const std::shared_ptr<auv_msgs::srv::AutomaticCapture::Request> request,
                           std::shared_ptr<auv_msgs::srv::AutomaticCapture::Response> response);
    void toggle_annotated(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void get_camera_settings(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void set_projection_heights(const std::shared_ptr<auv_msgs::srv::SetDownCamProjectionHeights::Request> request,
                                std::shared_ptr<auv_msgs::srv::SetDownCamProjectionHeights::Response> response);

    template <typename T>
    T required_param(const std::string& name);

    // --- parameters ---
    std::vector<std::string> class_names_;
    bool sim_ = false;
    bool enable_object_detection_ = true;
    bool compressed_ = false;
    std::string image_topic_;
    std::string detection_topic_;
    std::string camera_info_topic_;
    int queue_size_ = 10;
    double conf_threshold_ = 0.5;
    bool publish_annotated_image_ = false;
    std::atomic<bool> annotated_image_enabled_{false};
    int publish_annotated_every_n_frames_ = 1;
    int border_margin_ = 0;
    double lateral_scale_ = 1.333;

    std::string video_device_;
    int image_width_ = 640;
    int image_height_ = 480;
    int camera_fps_ = 30;
    std::string camera_frame_id_;
    std::string base_frame_id_;
    CameraIntrinsics intrinsics_;
    Eigen::Matrix4d T_auv_down_cam_ = Eigen::Matrix4d::Identity();

    std::mutex heights_mutex_;
    std::unordered_map<std::string, double> projection_heights_;  // label -> height above pool floor
    double pool_floor_z_ = 0.0;
    std::string pose_source_;
    std::string auv_pose_topic_;

    // Image collection (toggled via service)
    std::mutex collection_mutex_;
    bool collecting_ = false;
    double collection_interval_ = 2.0;
    double last_collection_time_ = 0.0;
    std::string collection_dir_;
    std::string collection_extension_;
    std::uint64_t frame_counter_ = 0;

    std::mutex auv_pose_mutex_;
    std::optional<PoseStamped> auv_pose_msg_;

    std::unique_ptr<RfDetrEngine> engine_;

    // VideoCapture is shared by the grab thread and the parameter / service callbacks.
    std::mutex cap_mutex_;
    cv::VideoCapture cap_;

    // --- ROS interfaces ---
    rclcpp::Publisher<auv_msgs::msg::VisionDetectionFrame>::SharedPtr pub_detections_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_raw_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_annotated_compressed_;
    rclcpp::Subscription<PoseStamped>::SharedPtr auv_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    std::vector<rclcpp::ServiceBase::SharedPtr> services_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    sensor_msgs::msg::CameraInfo camera_info_template_;

    std::thread grab_thread_;
    std::atomic<bool> running_{true};
};

}  // namespace vision
