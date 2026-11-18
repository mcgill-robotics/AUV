#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from auv_msgs.msg import SystemMetrics
from ament_index_python.packages import get_package_share_directory
import yaml
import psutil

class SystemMetricsNode(Node):
    def __init__(self):
        super().__init__("system_metrics_node")
        self.get_logger().info("Started the system metrics logger")

        # Get the parameter file
        default_config = os.path.join(
            get_package_share_directory('telemetry'),
            'config',
            'system_metrics.yaml'
        )
        self.declare_parameter('config_file', default_config)
        config_path = self.get_parameter('config_file').get_parameter_value().string_value

        # Load yaml config
        config = {}
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f) or {}
                self.get_logger().info(f'Loaded metrics configuration from {config_path}')
            else:
                self.get_logger().warn(f'Config file {config_path} not found. Using defaults.')
        except Exception as e:
            self.get_logger().error(f'Failed to load config from {config_path}: {e}')

        # Declare parameters with YAML values as defaults
        self.declare_parameter('collection_rate', float(config.get('collection_rate', 1.0)))
        self.declare_parameter('collect_cpu', bool(config.get('collect_cpu', True)))
        self.declare_parameter('collect_gpu', bool(config.get('collect_gpu', True)))
        self.declare_parameter('collect_memory', bool(config.get('collect_memory', True)))
        self.declare_parameter('collect_io', bool(config.get('collect_io', True)))
        self.declare_parameter('collect_network', bool(config.get('collect_network', True)))
        self.declare_parameter('on_jetson', bool(config.get('on_jetson', False)))

        # Get the parameter values
        self.collection_rate = self.get_parameter('collection_rate').get_parameter_value().double_value
        self.collect_cpu = self.get_parameter('collect_cpu').get_parameter_value().bool_value
        self.collect_gpu = self.get_parameter('collect_gpu').get_parameter_value().bool_value
        self.collect_memory = self.get_parameter('collect_memory').get_parameter_value().bool_value
        self.collect_io = self.get_parameter('collect_io').get_parameter_value().bool_value
        self.collect_network = self.get_parameter('collect_network').get_parameter_value().bool_value
        self.on_jetson = self.get_parameter('on_jetson').get_parameter_value().bool_value

        # Baseline initialization for non-blocking diffs
        psutil.cpu_percent(interval=None)
        self.last_io_time = self.get_clock().now()
        disk_counters = psutil.disk_io_counters()
        self.last_busy_time = getattr(disk_counters, 'busy_time', 0) if disk_counters else 0

        # System metrics publisher and timer
        self.metrics_pub = self.create_publisher(SystemMetrics, "~/system_metrics", 10)
        timer_period = 1.0 / self.collection_rate if self.collection_rate > 0.0 else 1.0
        self.timer = self.create_timer(timer_period, self.publish_system_metrics)

    def _get_temperature(self):
        """Retrieve highest system temperature from available hardware sensors."""
        if hasattr(psutil, "sensors_temperatures"):
            try:
                temps = psutil.sensors_temperatures()
                all_temps = [
                    entry.current
                    for entries in temps.values()
                    for entry in entries
                    if entry.current is not None and entry.current > 0
                ]
                if all_temps:
                    return float(max(all_temps))
            except Exception:
                pass

        # Fallback to sysfs thermal zone
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            pass

        return 0.0

    def _get_gpu_percent(self):
        """Get the Jetson GPU usage through its system file"""
        if not self.on_jetson:
            return 0.0
        
        GPU_LOAD_PATH = "/sys/devices/platform/gpu.0/load"
        with open(GPU_LOAD_PATH, "r") as f:
            raw_load = int(f.read().strip())

        return raw_load / 10.0  # Convert 0-1000 scale to percentage

    def _get_io_percent(self):
        """Calculate I/O busy percentage since last timer tick."""
        now = self.get_clock().now()
        dt = (now - self.last_io_time).nanoseconds * 1e-9
        self.last_io_time = now

        disk_counters = psutil.disk_io_counters()
        if not disk_counters or not hasattr(disk_counters, 'busy_time'):
            return 0.0

        current_busy = disk_counters.busy_time
        delta_busy = (current_busy - self.last_busy_time) / 1000.0  # ms to seconds
        self.last_busy_time = current_busy

        if dt > 0:
            io_percent = (delta_busy / dt) * 100.0
            return float(min(100.0, max(0.0, io_percent)))
        return 0.0

    def publish_system_metrics(self):
        # 1. CPU Usage & Temperature
        cpu_usage = float(psutil.cpu_percent(interval=None)) if self.collect_cpu else 0.0
        temperature = float(self._get_temperature())

        # 2. Memory Usage
        memory = float(psutil.virtual_memory().percent) if self.collect_memory else 0.0

        # 3. Disk Usage & I/O
        disk_usage = float(psutil.disk_usage('/').percent)
        io_usage = float(self._get_io_percent()) if self.collect_io else 0.0

        #4. Get GPU usage
        gpu_usage = self._get_gpu_percent()

        # Create and publish message
        system_metric_msg = SystemMetrics()
        system_metric_msg.cpu_usage = cpu_usage
        system_metric_msg.gpu_usage = gpu_usage
        system_metric_msg.temperature = temperature
        system_metric_msg.memory = memory
        system_metric_msg.disk_usage = disk_usage
        system_metric_msg.io = io_usage

        self.metrics_pub.publish(system_metric_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
