#!/usr/bin/env python3
"""Adapter between the Unity sim and OpenVINS (used by openvins_sim.launch.py).

The sim's sensor streams need three corrections before OpenVINS can use them:
  1. The front camera is only published as JPEG -> decode it to a raw mono8 image.
  2. The JPEG arrives upside down -> flip it (parameter `flip`: vertical | rotate180 | none).
  3. The sim delays IMU data by `imu_delay` seconds (SensorDelayBuffer, default 0.15 s) but
     stamps it at publish time -> shift the IMU stamps back so they line up with the camera.
"""
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, Imu


class OpenVinsSimAdapter(Node):
    def __init__(self):
        super().__init__("openvins_sim_adapter")
        self.flip = self.declare_parameter("flip", "vertical").value
        self.imu_delay = float(self.declare_parameter("imu_delay", 0.15).value)
        image_in = self.declare_parameter("image_in", "/zed/zed_node/rgb/color/rect/image/compressed").value
        image_out = self.declare_parameter("image_out", "/openvins/front/image_raw").value
        imu_in = self.declare_parameter("imu_in", "/imu/data").value
        imu_out = self.declare_parameter("imu_out", "/openvins/imu").value

        self.pub_image = self.create_publisher(Image, image_out, 10)
        self.pub_imu = self.create_publisher(Imu, imu_out, 200)
        self.create_subscription(CompressedImage, image_in, self.on_image, 10)
        self.create_subscription(Imu, imu_in, self.on_imu, 200)
        self.get_logger().info(f"flip={self.flip}, imu_delay={self.imu_delay:.3f}s, {image_in} -> {image_out}, {imu_in} -> {imu_out}")

    def on_image(self, msg):
        gray = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.get_logger().warn("could not decode image", throttle_duration_sec=5.0)
            return
        if self.flip == "vertical":
            gray = cv2.flip(gray, 0)
        elif self.flip == "rotate180":
            gray = cv2.flip(gray, -1)
        out = Image()
        out.header = msg.header
        out.height, out.width = gray.shape
        out.encoding = "mono8"
        out.is_bigendian = 0
        out.step = out.width
        out.data = gray.tobytes()
        self.pub_image.publish(out)

    def on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp) - Duration(seconds=self.imu_delay)
        msg.header.stamp = stamp.to_msg()
        self.pub_imu.publish(msg)


def main():
    rclpy.init()
    node = OpenVinsSimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
