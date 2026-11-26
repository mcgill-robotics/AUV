#include <rclcpp/rclcpp.hpp>

#include "vision/front_cam_object_detection.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<vision::FrontCamObjectDetectionNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("front_cam_object_detection"),
                     "front_cam_object_detection failed during startup: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
