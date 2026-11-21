#include <rclcpp/rclcpp.hpp>

#include "vision/down_cam_object_detection.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<vision::DownCamObjectDetectionNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("down_cam_object_detection"),
                     "down_cam_object_detection failed during startup: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
