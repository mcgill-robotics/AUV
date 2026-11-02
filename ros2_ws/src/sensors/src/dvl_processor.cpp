#include "sensors/dvl_processor.hpp"
#include "sensors/utils.hpp"

namespace sensors {

/// Creates the DVL processor, loads frame and mounting parameters, and sets up
/// subscriptions and publishers for DVL, IMU, position, velocity, and odometry.
DvlProcessor::DvlProcessor() : Node("dvl_processor") {
    q_iv_.setIdentity();

    this->declare_parameter<std::vector<double>>("r_dv_v", {0.0, 0.0, 0.0});
    std::vector<double> r_dv_v_vec;
    this->get_parameter("r_dv_v", r_dv_v_vec);
    r_dv_v_ = Vec3(r_dv_v_vec[0], r_dv_v_vec[1], r_dv_v_vec[2]);

    //quarternion rotation from dvl inertial frame to pool inertial frame
    //DVL posts data with +X forward, +Y right, and +Z downward. 
    //Pool inertial uses +X forward, +Y left, and +Z up.
    this->declare_parameter<std::vector<double>>("q_pi2", {1.0, 0.0, 0.0, 0.0});
    std::vector<double> q_pi2_vec;
    this->get_parameter("q_pi2", q_pi2_vec);
    q_pi2_ = sensors::math::quatFromParamWxyz(q_pi2_vec);

    //quaternion rotation from dvl frame to vehicle frame
    this->declare_parameter<std::vector<double>>("q_vd", {1.0, 0.0, 0.0, 0.0});
    std::vector<double> q_vd_vec;
    this->get_parameter("q_vd", q_vd_vec);
    q_vd_ = sensors::math::quatFromParamWxyz(q_vd_vec);


    this->declare_parameter<std::string>("frame_id_global", "pool_link");
    this->get_parameter("frame_id_global", frame_id_global_);
    this->declare_parameter<std::string>("frame_id_auv", "auv_link");
    this->get_parameter("frame_id_auv", frame_id_auv_);

    // subscribes to the imu processor's output
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "auv_frame/imu", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::imu_callback, this, std::placeholders::_1));

    dvl_pos_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/dead_reckoning", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_position_callback, this, std::placeholders::_1));
    dvl_vel_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
        "dvl/velocity", rclcpp::SensorDataQoS().keep_last(1), std::bind(&DvlProcessor::dvl_velocity_callback, this, std::placeholders::_1));
    

    position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("auv_frame/dvl/position", rclcpp::SensorDataQoS().keep_last(1)); 
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("auv_frame/dvl/velocity", rclcpp::SensorDataQoS().keep_last(1)); 
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("state/odometry", rclcpp::SensorDataQoS().keep_last(1));
    
    //Covariances are expressed in the frame provided by the sensor
    //These are in imu frame
    orientation_covariance_ = Eigen::Matrix3d::Zero();
    angular_velocity_covariance_ = Eigen::Matrix3d::Zero();
    //
    velocity_covariance_ = Eigen::Matrix3d::Zero();
    velocity_angular_covariance_ = Eigen::Matrix3d::Zero();
    position_covariance_ = Eigen::Matrix3d::Zero();
}


//IMU data expressed in the vehicle frame, including orientation relative to the pool frame.
void DvlProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    q_iv_.w() = msg->orientation.w;
    q_iv_.x() = msg->orientation.x;
    q_iv_.y() = msg->orientation.y;
    q_iv_.z() = msg->orientation.z;

    w_v_.x() = msg->angular_velocity.x;
    w_v_.y() = msg->angular_velocity.y;
    w_v_.z() = msg->angular_velocity.z;

    const Eigen::Matrix3d orientation_covariance_vehicle =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            msg->orientation_covariance.data());
    const Eigen::Matrix3d vehicle_to_pool = q_iv_.toRotationMatrix();
    orientation_covariance_ =
        vehicle_to_pool * orientation_covariance_vehicle
        * vehicle_to_pool.transpose();
    angular_velocity_covariance_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->angular_velocity_covariance.data());
}



// msg: DVL velocity and its 3x3 covariance, expressed in DVL axes
void DvlProcessor::dvl_velocity_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {
    // Vector from DVL inertial frame to DVL, expressed in DVL frame
    Vec3 v_di2_d = Vec3(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    //Convert from row-major array to Eigen::Matrix3d
    Eigen::Matrix3d covariance = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(msg->covariance.data());

    std::pair<Vec3, Eigen::Matrix3d> result = process_dvl_velocity(v_di2_d, covariance);
    velocity_covariance_ = result.second;
    Eigen::Matrix3d lever_arm_cross_product;
    lever_arm_cross_product << 0.0, -r_dv_v_.z(), r_dv_v_.y(),
                               r_dv_v_.z(), 0.0, -r_dv_v_.x(),
                               -r_dv_v_.y(), r_dv_v_.x(), 0.0;
    velocity_angular_covariance_ =
        lever_arm_cross_product * angular_velocity_covariance_;
    current_velocity_p_ = result.first;
    velocity_received_ = true;
    publish_velocity_msg(result.first);

    if (position_received_) {
        publish_odometry_msg(current_position_p_, current_velocity_p_);
    }
}


// msg: DVL dead-reckoning position and scalar position standard deviation
// expressed in DVL axes, with the position expressed in the DVL inertial frame.
void DvlProcessor::dvl_position_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg) {
    // Vector from DVL starting origin to DVL current position, expressed in DVL starting frame
    Vec3 r_di2_i2 = Vec3(msg->position.x, msg->position.y, msg->position.z);

    // Vector from pool frame to vehicle frame, expressed in pool frame
    Vec3 r_vp_p = process_dvl_position(r_di2_i2);

    // Position covariance in dvl inertial axes
    Eigen::Matrix3d position_cov_dvl = Eigen::Matrix3d::Zero();
    position_cov_dvl(0, 0) = msg->pos_std * msg->pos_std;
    position_cov_dvl(1, 1) = msg->pos_std * msg->pos_std;
    // Z covariance is left as 0, since the depth sensor is very accurate and we're not using dvl depth
    position_cov_dvl(2, 2) = 0.0;

    const Eigen::Matrix3d dvl_rotation = q_pi2_.toRotationMatrix();
    const Eigen::Matrix3d position_cov_pool =
        dvl_rotation * position_cov_dvl * dvl_rotation.transpose();

    // Propagate vehicle-orientation uncertainty through the DVL lever arm.
    // For small orientation errors expressed in vehicle axes:
    //   delta_p = R_vehicle_to_pool * [r_dv_v]x * delta_theta
    const Eigen::Matrix3d vehicle_rotation = q_iv_.toRotationMatrix();
    const Eigen::Matrix3d orientation_covariance_vehicle =
        vehicle_rotation.transpose() * orientation_covariance_
        * vehicle_rotation;
    Eigen::Matrix3d lever_arm_cross_product;
    lever_arm_cross_product << 0.0, -r_dv_v_.z(), r_dv_v_.y(),
                               r_dv_v_.z(), 0.0, -r_dv_v_.x(),
                               -r_dv_v_.y(), r_dv_v_.x(), 0.0;
    const Eigen::Matrix3d position_jacobian =
        vehicle_rotation * lever_arm_cross_product;
    const Eigen::Matrix3d orientation_covariance =
        position_jacobian * orientation_covariance_vehicle
        * position_jacobian.transpose();

    const Eigen::Matrix3d position_cov_pool_total =
        position_cov_pool + orientation_covariance;
    // Store position covariance in pool axes for the odometry pose.
    position_covariance_ = position_cov_pool_total;
    current_position_p_ = r_vp_p;
    position_received_ = true;
    // Don't set z, since depth is better from the pressure sensor
    publish_position_msg(r_vp_p);

    if (velocity_received_) {
        publish_odometry_msg(current_position_p_, current_velocity_p_);
    }
}



// r_di2_i2: DVL displacement from its startup origin, expressed in the  dvl inertial frame
Vec3 DvlProcessor::process_dvl_position(const Vec3& r_di2_i2) const 
{
    // This is unused, not sure why its here
    // DvlData_InertialFrame dvl_inertial;
    // Vector from pool frame to dvl inertial frame, expressed in pool frame
    // these are equal because the pool frame is the initial vehicle frame
    // and the dvl inertial frame is the initial dvl frame
    Vec3 r_i2p_p = r_dv_v_;
    // Vector from dvl inertial frame to dvl current position, expressed in pool frame
    Vec3 r_di2_p = q_pi2_ * r_di2_i2;
    // Displacement from dvl frame to vehicle frame, expressed in vehicle frame
    Vec3 r_vd_v = -r_dv_v_;
    // Displacement from dvl frame to vehicle frame, expressed in pool frame
    Vec3 r_vd_p = q_iv_ * r_vd_v;
    // Displacement from pool frame to vehicle frame, expressed in pool frame
    Vec3 r_vp_p = r_i2p_p + r_di2_p + r_vd_p;
    return r_vp_p;
}

// v_di2_d: Measured DVL velocity expressed in DVL axes.
// v_cov_dvl: 3x3 DVL velocity covariance, expressed in DVL axes.
// Returns a pair containing pool-frame vehicle velocity and its vehicle-frame covariance.
std::pair<Vec3, Eigen::Matrix3d> DvlProcessor::process_dvl_velocity(const Vec3& v_di2_d, const Eigen::Matrix3d& v_cov_dvl) const {
    // Quaternion from DVL current frame to pool frame
    Quatd q_id = q_iv_ * q_vd_;
    // Measured DVL velocity expressed in pool axes
    Vec3 v_di_p = q_id * v_di2_d;
    // the cross product is velocity due to the dvl rotating about the vehicle origin
    // So this is velocity with correction for the dvl being off center. expressed in pool axes
    Vec3 v_vp_p = v_di_p - q_iv_ * (w_v_.cross(r_dv_v_));
    // Transform the DVL velocity covariance to the vehicle frame used by
    // nav_msgs/Odometry.twist.
    const Eigen::Matrix3d dvl_to_pool = q_id.toRotationMatrix();
    const Eigen::Matrix3d velocity_covariance_pool =
        dvl_to_pool * v_cov_dvl * dvl_to_pool.transpose();
    const Eigen::Matrix3d pool_to_vehicle = q_iv_.inverse().toRotationMatrix();
    const Eigen::Matrix3d velocity_covariance_vehicle =
        pool_to_vehicle * velocity_covariance_pool
        * pool_to_vehicle.transpose();

    Eigen::Matrix3d lever_arm_cross_product;
    lever_arm_cross_product << 0.0, -r_dv_v_.z(), r_dv_v_.y(),
                               r_dv_v_.z(), 0.0, -r_dv_v_.x(),
                               -r_dv_v_.y(), r_dv_v_.x(), 0.0;
    const Eigen::Matrix3d lever_arm_velocity_covariance =
        lever_arm_cross_product * angular_velocity_covariance_
        * lever_arm_cross_product.transpose();

    // The DVL and IMU angular-velocity measurements are assumed independent.
    const Eigen::Matrix3d corrected_velocity_covariance =
        velocity_covariance_vehicle + lever_arm_velocity_covariance;

    return std::make_pair(v_vp_p, corrected_velocity_covariance);
}

/// Publishes a pool-frame vehicle position as a stamped point.
///
/// Args:
///   r_vp_p: Vehicle-origin position expressed in pool axes.
void DvlProcessor::publish_position_msg(const Vec3& r_vp_p) const {
    geometry_msgs::msg::PointStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;
    msg_out.point.x = r_vp_p.x();
    msg_out.point.y = r_vp_p.y();
    msg_out.point.z = r_vp_p.z();
    
    position_pub_->publish(msg_out);
}

/// Publishes a pool-frame vehicle linear velocity as a stamped twist.
///
/// Args:
///   v_vp_p: Vehicle-origin linear velocity expressed in pool axes.
void DvlProcessor::publish_velocity_msg(const Vec3& v_vp_p) const {
    geometry_msgs::msg::TwistStamped msg_out;
    msg_out.header.frame_id = frame_id_global_;    
    msg_out.twist.linear.x = v_vp_p.x();
    msg_out.twist.linear.y = v_vp_p.y();
    msg_out.twist.linear.z = v_vp_p.z();

    velocity_pub_->publish(msg_out);
}

// Args:
///   r_vp_p: Displacement from pool frame to vehicle frame, expressed in pool frame.
///   v_vp_p: Vehicle-origin linear velocity expressed in pool axes.
void DvlProcessor::publish_odometry_msg(const Vec3& r_vp_p, const Vec3& v_vp_p) const {
    nav_msgs::msg::Odometry msg_out;
    msg_out.header.stamp = this->now();
    msg_out.header.frame_id = frame_id_global_;
    msg_out.child_frame_id = frame_id_auv_;
    msg_out.pose.pose.position.x = r_vp_p.x();
    msg_out.pose.pose.position.y = r_vp_p.y();
    msg_out.pose.pose.position.z = r_vp_p.z();
    msg_out.pose.pose.orientation.w = q_iv_.w();
    msg_out.pose.pose.orientation.x = q_iv_.x();
    msg_out.pose.pose.orientation.y = q_iv_.y();
    msg_out.pose.pose.orientation.z = q_iv_.z();

    const Vec3 v_vp_v = q_iv_.inverse() * v_vp_p;
    msg_out.twist.twist.linear.x = v_vp_v.x();
    msg_out.twist.twist.linear.y = v_vp_v.y();
    msg_out.twist.twist.linear.z = v_vp_v.z();

    // Nav2 wants the twist covariance expressed in the child frame, which is the vehicle frame
    Eigen::Matrix<double, 6, 6> twist_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    twist_covariance.block<3, 3>(0, 0) = velocity_covariance_;
    twist_covariance.block<3, 3>(0, 3) = velocity_angular_covariance_;
    twist_covariance.block<3, 3>(3, 0) = velocity_angular_covariance_.transpose();
    twist_covariance.block<3, 3>(3, 3) = angular_velocity_covariance_;

    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            msg_out.twist.covariance[row * 6 + column] = twist_covariance(row, column);
        }
    }

    // Nav2 wants the pose covariance expressed in the parent frame, which is the pool frame
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    pose_covariance.block<3, 3>(0, 0) = position_covariance_;
    pose_covariance.block<3, 3>(3, 3) = orientation_covariance_;

    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            msg_out.pose.covariance[row * 6 + column] = pose_covariance(row, column);
        }
    }

    // Publish the odometry message
    // Assuming you have an odometry publisher defined as odometry_pub_
    odometry_pub_->publish(msg_out);

} 

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::DvlProcessor>());
    rclcpp::shutdown();
    return 0;
}
