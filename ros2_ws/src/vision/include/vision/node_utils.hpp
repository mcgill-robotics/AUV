#pragma once

// Small helpers shared by the C++ camera detection nodes.

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace vision {

inline Eigen::Matrix4d transform_from_xyz_rpy(const std::vector<double>& xyz, const std::vector<double>& rpy) {
    // Same convention as tf_transformations.euler_matrix (static XYZ): R = Rz(yaw) * Ry(pitch) * Rx(roll).
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                              .toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    return T;
}

inline Eigen::Matrix4d transform_from_pose(const geometry_msgs::msg::Pose& pose) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return T;
}

inline double wall_time_sec() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// datetime.now().strftime('%Y%m%d_%H%M%S_%f')
inline std::string collection_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S_") << std::setw(6) << std::setfill('0') << us;
    return ss.str();
}

}  // namespace vision
