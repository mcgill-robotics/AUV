#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "controls/attitude_controller.hpp"


#define quaternion_topic "/controls/quaternion_setpoint"

// Test fixture for AttitudeController
class AttitudeControllerTest : public ::testing::Test 
{
    protected:
        static void SetUpTestSuite() 
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite() 
        {
            rclcpp::shutdown();
        }

        void SetUp() override
        {
            attitudeController = std::make_shared<controls::AttitudeController>();
        }

        void TearDown() override
        {
            attitudeController.reset();
        }

        std::shared_ptr<controls::AttitudeController> attitudeController;
};

// Verify Node exists
TEST_F(AttitudeControllerTest, NodeExists) 
{
    ASSERT_NE(attitudeController, nullptr);
    EXPECT_STREQ(attitudeController->get_name(), "attitude_controller");
    EXPECT_NE(attitudeController->pub_effort_, nullptr);
}

// Test control effort when at rest
TEST_F(AttitudeControllerTest, RestEffortMath)
{
    // At rest with default identity quaternions,
    // there should be no effort
    auto effort = attitudeController->compute_control_effort();

    // Use expect near since there might be some
    // floating point errors
    EXPECT_NEAR(effort.torque.x, 0.0, 1e-6);
    EXPECT_NEAR(effort.torque.y, 0.0, 1e-6);
    EXPECT_NEAR(effort.torque.z, 0.0, 1e-6);
}

TEST_F(AttitudeControllerTest, ProportionalYawEffort)
{
    auto target_publisher = attitudeController->create_publisher<geometry_msgs::msg::Quaternion>
        (quaternion_topic, 1);
    auto imu_pub = attitudeController->create_publisher<sensor_msgs::msg::Imu>(
        "auv_frame/imu", rclcpp::SensorDataQoS().keep_last(1));
    
    // Publish state to controller
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_pub->publish(imu_msg);


    // Enable the controller
    attitudeController->set_parameter(rclcpp::Parameter("enabled", true));

    // ZERO OUt the slew rates and I terms
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_roll_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_pitch_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_yaw_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ex", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ey", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ez", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("P_ez", 1.0));

    // Get the Pz gain
    double P_ez_;
    attitudeController->get_parameter("P_ez", P_ez_);

    // Target 60 degrees yaw 
    // Expect w = cos(60deg/2) = sqrt(3)/2
    //        x = 0 * sin(60deg/2) = 0
    //        y = 0 * sin(60deg/2) = 0
    //        z = 1 * sin(60deg/2) = 1/2
    geometry_msgs::msg::Quaternion target_msg;
    target_msg.w = std::cos(M_PI / 6.0);
    target_msg.x = 0;
    target_msg.y = 0;
    target_msg.z = 0.5;
    target_publisher->publish(target_msg);

    // Spin the node for a lil bit for the subscriber to catch the publish
    auto start_time = attitudeController->get_clock()->now();
    auto duration = rclcpp::Duration::from_seconds(5.0);

    for (int step = 0; step < 1; ++step) {
        imu_pub->publish(imu_msg);
        rclcpp::spin_some(attitudeController);
    }

    auto effort = attitudeController->compute_control_effort();

    // check expected target
    EXPECT_NEAR(effort.torque.x, 0.0, 1e-4);
    EXPECT_NEAR(effort.torque.y, 0.0, 1e-4);
    EXPECT_NEAR(effort.torque.z, 3.0, 1e-4);
}


TEST_F(AttitudeControllerTest, ClosedLoopYawConvergence)
{
    // Enable the controller
    attitudeController->set_parameter(rclcpp::Parameter("enabled", true));

    // ZERO OUt the slew rates and I terms
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_roll_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_pitch_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("max_slew_rate_yaw_deg", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ex", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ey", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("I_ez", 0.0));
    attitudeController->set_parameter(rclcpp::Parameter("P_ez", 1.0));

    // 1. Setup publishers
    auto imu_pub = attitudeController->create_publisher<sensor_msgs::msg::Imu>(
        "auv_frame/imu", rclcpp::SensorDataQoS().keep_last(1));
    auto target_pub = attitudeController->create_publisher<geometry_msgs::msg::Quaternion>(
        "/controls/quaternion_setpoint", 1);

    // Publish state to controller
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_pub->publish(imu_msg);

    // 2. Set target to 45 deg yaw
    double target_yaw = M_PI / 4.0;
    geometry_msgs::msg::Quaternion target_msg;
    target_msg.w = std::cos(target_yaw / 2.0);
    target_msg.x = 0.0;
    target_msg.y = 0.0;
    target_msg.z = std::sin(target_yaw / 2.0);
    target_pub->publish(target_msg);

    // 3. Simple AUV physical parameters (approximate)
    Eigen::Matrix3d J = Eigen::Vector3d(1.5, 2.0, 1.8).asDiagonal();       // Moment of inertia (kg*m^2)
    Eigen::Matrix3d J_inv = J.inverse();
    Eigen::Matrix3d D = Eigen::Vector3d(4.0, 4.0, 3.5).asDiagonal();       // Rotational damping (N*m*s/rad)

    // Initial vehicle state: at rest at 0 deg
    Eigen::Quaterniond q_state = Eigen::Quaterniond::Identity();
    Eigen::Vector3d w_state = Eigen::Vector3d::Zero();

    double dt = 0.04; // 25 Hz matching controller loop
    double initial_error = 0.0;
    double final_error = 0.0;

    // 4. Simulate closed loop for 3.0 seconds (75 steps)
    for (int step = 0; step < 75; ++step) {
        // Publish state to controller
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.orientation.w = q_state.w();
        imu_msg.orientation.x = q_state.x();
        imu_msg.orientation.y = q_state.y();
        imu_msg.orientation.z = q_state.z();
        imu_msg.angular_velocity.x = w_state.x();
        imu_msg.angular_velocity.y = w_state.y();
        imu_msg.angular_velocity.z = w_state.z();
        imu_pub->publish(imu_msg);

        // Process controller callbacks
        rclcpp::spin_some(attitudeController);

        // Compute controller output
        auto effort = attitudeController->compute_control_effort();
        Eigen::Vector3d tau_control(effort.torque.x, effort.torque.y, effort.torque.z);

        // Calculate current quaternion error magnitude: 2 * acos(|w|) or 2 * asin(||v||)
        Eigen::Quaterniond q_target(target_msg.w, target_msg.x, target_msg.y, target_msg.z);
        Eigen::Quaterniond q_diff = q_state.conjugate() * q_target;
        if (q_diff.w() < 0.0) q_diff.coeffs() *= -1.0;
        double current_error_rad = 2.0 * std::atan2(q_diff.vec().norm(), q_diff.w());
        if (step == 0) initial_error = current_error_rad;
        final_error = current_error_rad;

        // --- Numerical Physics Step (Euler integration) ---
        Eigen::Vector3d tau_net = tau_control - D * w_state;
        Eigen::Vector3d alpha = J_inv * tau_net;
        w_state += alpha * dt;
        
        // Quaternion kinematics: q_new = q + 0.5 * q * [0, w] * dt
        Eigen::Quaterniond w_quat(0.0, w_state.x(), w_state.y(), w_state.z());
        Eigen::Quaterniond q_dot;
        q_dot.coeffs() = 0.5 * (q_state * w_quat).coeffs();
        q_state.coeffs() += q_dot.coeffs() * dt;
        q_state.normalize();
    }
    // Assert that error converged significantly
    EXPECT_GT(initial_error, 0.7);          // ~0.785 rad (45 deg)
    EXPECT_LT(final_error, 0.05);           // Within ~3 deg of target
    EXPECT_LT(final_error, initial_error);
}