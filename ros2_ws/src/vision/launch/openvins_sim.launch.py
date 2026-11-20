"""OpenVINS on the Unity sim's front camera + IMU.

Run by path (no rebuild of the vision package needed):
    ros2 launch ros2_ws/src/vision/launch/openvins_sim.launch.py
    ros2 launch ros2_ws/src/vision/launch/openvins_sim.launch.py flip:=rotate180

Starts openvins_sim_adapter.py (decodes + flips the sim's JPEG front camera and
re-times the sim's delayed IMU) and the OpenVINS node.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
CONFIG = os.path.normpath(os.path.join(HERE, "..", "config", "openvins_sim", "estimator_config.yaml"))
ADAPTER = os.path.join(HERE, "openvins_sim_adapter.py")


def generate_launch_description():
    flip = DeclareLaunchArgument("flip", default_value="vertical", description="vertical | rotate180 | none")
    imu_delay = DeclareLaunchArgument("imu_delay", default_value="0.15", description="sim IMU latency in seconds")
    adapter = ExecuteProcess(
        cmd=[
            sys.executable, ADAPTER, "--ros-args",
            "-p", ["flip:=", LaunchConfiguration("flip")],
            "-p", ["imu_delay:=", LaunchConfiguration("imu_delay")],
        ],
        output="screen",
    )
    openvins = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": CONFIG},
        ],
        output="screen",
    )
    return LaunchDescription([flip, imu_delay, adapter, openvins])
