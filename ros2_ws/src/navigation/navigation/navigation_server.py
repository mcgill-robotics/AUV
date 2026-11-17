#!/usr/bin/env python3
"""
Navigation Action Server

Hosts the /motion/navigate action server that orchestrates the AUV's PID
controllers (depth, X, Y, attitude) for goal-driven navigation.

Subscribes:
    state/pose (geometry_msgs/PoseStamped) - current AUV pose in pool frame
    /plan (nav_msgs/Path) - Nav2's latest computed path
    /cmd_vel_nav (geometry_msgs/Twist) - Nav2 controller velocity command

Publishes setpoints to:
    /controls/depth_setpoint (std_msgs/Float64)
    /controls/x_setpoint     (std_msgs/Float64)
    /controls/y_setpoint     (std_msgs/Float64)
    /controls/quaternion_setpoint (geometry_msgs/Quaternion)
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from auv_msgs.action import AUVNavigate
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath

from controls.utils import yaw_from_quaternion
import numpy as np
from scipy.spatial.transform import Rotation


class NavigationServer(Node):
    """Action server that drives the AUV to target poses via PID controllers."""

    def __init__(self):
        super().__init__('navigation_server')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('feedback_rate_hz', 10.0)
        self.declare_parameter('state_topic', 'state/pose')
        self.declare_parameter('depth_setpoint_topic', '/controls/depth_setpoint')
        self.declare_parameter('x_setpoint_topic', '/controls/x_setpoint')
        self.declare_parameter('y_setpoint_topic', '/controls/y_setpoint')
        self.declare_parameter('quat_setpoint_topic', '/controls/quaternion_setpoint')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('controller_action', '/follow_path')
        self.declare_parameter('replanning_rate_hz', 1.0)
        self.declare_parameter('carrot_distance', 0.5)
        self.declare_parameter('planning_frame', 'pool_link')
        self.declare_parameter('planner_id', 'GridBased')

        self._planner_client = ActionClient(
            self, ComputePathToPose, self.get_parameter('planner_action').value
        )
        self._controller_client = ActionClient(
            self, FollowPath, self.get_parameter('controller_action').value
        )

        self.feedback_rate_hz = self.get_parameter('feedback_rate_hz').value
        state_topic = self.get_parameter('state_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        depth_setpoint_topic = self.get_parameter('depth_setpoint_topic').value
        x_setpoint_topic = self.get_parameter('x_setpoint_topic').value
        y_setpoint_topic = self.get_parameter('y_setpoint_topic').value
        quat_setpoint_topic = self.get_parameter('quat_setpoint_topic').value
        self._feedback_period = 1.0 / self.feedback_rate_hz
        self._replanning_period = 1.0 / float(
            self.get_parameter('replanning_rate_hz').value
        )
        self._carrot_distance = float(self.get_parameter('carrot_distance').value)
        self._planning_frame = self.get_parameter('planning_frame').value
        self._planner_id = self.get_parameter('planner_id').value

        # ── State ────────────────────────────────────────────────────────────
        self.current_pose = None  # Latest PoseStamped from state_aggregator
        self._goal_handle = None  # active navigation goal handle
        self._goal_lock = threading.Lock()
        self._planning_goal = None # Latest PoseStamped sent to Nav2 for path computation
        self._planning_goal_request = None # Latest AUVNavigate goal request
        self._controller_goal_handle = None # Latest FollowPath goal handle sent to Nav2 controller

        # Cache the last published quaternion setpoint to prevent drift
        # on uncontrolled DOFs when sending successive partial-orientation commands.
        self._last_quat_setpoint = None

        # Callback group for concurrent action + subscriptions
        self.cb_group = ReentrantCallbackGroup()

        # QoS for sensor data (latest reading only)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        #QoS for setpoints (ensure delivery)
        setpoint_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber: AUV state ────────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped,
            state_topic,
            self._pose_callback,
            sensor_qos,
            callback_group=self.cb_group,
        )

        self.plan_sub = self.create_subscription(
            Path,
            plan_topic,
            self._plan_callback,
            sensor_qos,
            callback_group=self.cb_group,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self._cmd_vel_callback,
            sensor_qos,
            callback_group=self.cb_group,
        )

        # ── Publishers: controller setpoints ─────────────────────────────────
        self.pub_depth_sp = self.create_publisher(Float64, depth_setpoint_topic, setpoint_qos)
        self.pub_x_sp = self.create_publisher(Float64, x_setpoint_topic, setpoint_qos)
        self.pub_y_sp = self.create_publisher(Float64, y_setpoint_topic, setpoint_qos)
        self.pub_quat_sp = self.create_publisher(Quaternion, quat_setpoint_topic, setpoint_qos)
        self.replan_timer = self.create_timer(
            self._replanning_period,
            self._replan_callback,
            callback_group=self.cb_group,
        )

        # ── Action server ────────────────────────────────────────────────────
        self.action_server = ActionServer(
            self,
            AUVNavigate,
            '/motion/navigate',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Navigation server ready on /motion/navigate')

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _replan_callback(self):
        """Request a fresh path for the active navigation goal."""
        if self._planning_goal is not None:
            self._request_path()

    def _request_path(self):
        """Ask Nav2 to compute a path to the active goal."""
        if not self._planner_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Planner action server not available')
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._planning_goal
        goal_msg.planner_id = self._planner_id
        future = self._planner_client.send_goal_async(goal_msg)
        future.add_done_callback(self._planner_goal_response_callback)

    def _planner_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Planner rejected the path request')

    def _plan_callback(self, msg: Path):
        """Forward incoming path plans to the controller action server."""
        if not self._controller_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available, cannot forward plan')
            return

        if self._controller_goal_handle is not None:
            self._controller_goal_handle.cancel_goal_async()

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "general_goal_checker"
        goal_msg.progress_checker_id = "progress_checker"
        future = self._controller_client.send_goal_async(goal_msg)
        future.add_done_callback(self._controller_goal_response_callback)
        self.get_logger().info('Forwarded path plan to controller action server')

    def _controller_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self._controller_goal_handle = goal_handle
        else:
            self.get_logger().warn('Controller rejected the path plan')

    def _cmd_vel_callback(self, msg: Twist):
        """Publish a position carrot in front of the vehicle for the PID loops."""
        if self.current_pose is None or self._planning_goal_request is None or not self._planning_goal_request.do_path_computation:
            return

        speed = math.hypot(msg.linear.x, msg.linear.y)
        current = self.current_pose.pose.position
        if speed < 1e-6:
            carrot_x = current.x
            carrot_y = current.y
        else:
            yaw = yaw_from_quaternion(self.current_pose.pose.orientation)
            pool_velocity_x = (
                msg.linear.x * math.cos(yaw) - msg.linear.y * math.sin(yaw)
            )
            pool_velocity_y = (
                msg.linear.x * math.sin(yaw) + msg.linear.y * math.cos(yaw)
            )
            scale = self._carrot_distance / speed
            carrot_x = current.x + pool_velocity_x * scale
            carrot_y = current.y + pool_velocity_y * scale

        if self._planning_goal_request.do_x:
            x_msg = Float64()
            x_msg.data = carrot_x
            self.pub_x_sp.publish(x_msg)
        if self._planning_goal_request.do_y:
            y_msg = Float64()
            y_msg.data = carrot_y
            self.pub_y_sp.publish(y_msg)

    def _pose_callback(self, msg: PoseStamped):
        """Cache the latest AUV pose."""
        self.current_pose = msg

    def _goal_callback(self, goal_request):
        """Accept all incoming goals (preemption handled by action server)."""
        self.get_logger().info('Received navigation goal')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # ─────────────────────────────────────────────────────────────────────────
    # Goal execution
    # ─────────────────────────────────────────────────────────────────────────

    async def _execute_callback(self, goal_handle):
        """Main execution loop for a navigation goal."""
        # Handle preemption: ensure only one goal is running at a time
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Preempting previous goal with new request')
                try:
                    self._goal_handle.abort()
                except Exception as e:
                    self.get_logger().debug(f'Ignored abort on previous goal handle: {e}')
            self._goal_handle = goal_handle

        goal = goal_handle.request
        self._planning_goal_request = goal
        result = AUVNavigate.Result()

        # ── Wait for state data ──────────────────────
        if self.current_pose is None:
            self.get_logger().warn('Waiting for state/pose...')
            for _ in range(50):  # Wait up to 5 seconds
                if self.current_pose is not None:
                    break
                await self._sleep(0.1)
            if self.current_pose is None:
                result.success = False
                result.message = 'No state/pose data received'
                goal_handle.abort()
                self._clear_navigation_state(goal_handle)
                return result

        # ── Compute absolute target ──────────────────
        target_x, target_y, target_z, target_quat = self._compute_absolute_target(goal)
        self._log_target(goal, target_x, target_y, target_z, target_quat)

        # if goal.do_path_computation is True, request a path from Nav2; otherwise, publish setpoints directly
        if goal.do_path_computation:
            self._planning_goal = PoseStamped()
            self._planning_goal.header.frame_id = self._planning_frame
            self._planning_goal.header.stamp = self.get_clock().now().to_msg()
            self._planning_goal.pose.position.x = target_x
            self._planning_goal.pose.position.y = target_y
            self._planning_goal.pose.position.z = target_z
            self._planning_goal.pose.orientation = target_quat
            self._request_path()

        else:
            self._planning_goal = None
            if self._controller_goal_handle is not None:
                self._controller_goal_handle.cancel_goal_async()
                self._controller_goal_handle = None
            self._publish_setpoints(goal, target_x, target_y, target_z, target_quat)

        # ── Feedback loop ────────────────────────────
        feedback_msg = AUVNavigate.Feedback()
        rate_period = self._feedback_period
        time_in_tolerance = 0.0
        start_time = self.get_clock().now()

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Goal canceled'
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                self._clear_navigation_state(goal_handle)
                return result

            # Check for preemption (goal_handle becomes inactive if preempted)
            if not goal_handle.is_active:
                self.get_logger().info('Goal preempted by new goal')
                result.success = False
                result.message = 'Preempted'
                return result

            # Read current state
            if self.current_pose is None:
                await self._sleep(rate_period)
                continue

            current = self.current_pose.pose

            # Compute errors (only for controlled DOFs)
            pos_error = self._compute_position_error(
                goal, current, target_x, target_y, target_z
            )
            
            angular_error = self._compute_angular_error(goal, current.orientation, target_quat)

            # Check convergence
            pos_converged = pos_error <= goal.position_tolerance
            angular_converged = angular_error <= goal.angular_tolerance

            if pos_converged and angular_converged:
                time_in_tolerance += rate_period
            else:
                time_in_tolerance = 0.0

            # Publish feedback
            feedback_msg.current_pose = current
            feedback_msg.position_error_3d = Vector3(
                x=(current.position.x - target_x) if goal.do_x else 0.0,
                y=(current.position.y - target_y) if goal.do_y else 0.0,
                z=(current.position.z - target_z) if goal.do_z else 0.0
            )
            feedback_msg.position_error = pos_error
            feedback_msg.angular_error = angular_error
            feedback_msg.time_in_tolerance = time_in_tolerance
            goal_handle.publish_feedback(feedback_msg)

            # Success condition
            if time_in_tolerance > goal.hold_time:
                result.success = True
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                result.message = (
                    f'Reached target in {elapsed:.1f}s (held for {goal.hold_time:.1f}s, '
                    f'pos_tol={goal.position_tolerance:.2f}m, '
                    f'ang_tol={math.degrees(goal.angular_tolerance):.1f}deg)'
                )
                goal_handle.succeed()
                with self._goal_lock:
                    if self._goal_handle == goal_handle:
                        self._goal_handle = None
                        self._planning_goal = None
                        self._planning_goal_request = None
                self.get_logger().info(result.message)
                return result

            # Timeout condition
            if goal.timeout > 0.0:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed >= goal.timeout:
                    result.success = False
                    result.message = (
                        f'Timed out after {goal.timeout:.1f}s '
                        f'(pos_err={pos_error:.3f}m, ang_err={math.degrees(angular_error):.1f}deg)'
                    )
                    goal_handle.abort()
                    with self._goal_lock:
                        if self._goal_handle == goal_handle:
                            self._goal_handle = None
                            self._planning_goal = None
                            self._planning_goal_request = None
                    self.get_logger().warn(result.message)
                    return result

            await self._sleep(rate_period)

        # Node shutting down
        result.success = False
        result.message = 'Node shutdown'
        goal_handle.abort()
        self._clear_navigation_state(goal_handle)
        return result

    def _clear_navigation_state(self, goal_handle):
        """Stop forwarding the completed navigation goal to Nav2."""
        with self._goal_lock:
            if self._goal_handle != goal_handle:
                return
            self._goal_handle = None
            self._planning_goal = None
            self._planning_goal_request = None
            if self._controller_goal_handle is not None:
                self._controller_goal_handle.cancel_goal_async()
                self._controller_goal_handle = None

    # ─────────────────────────────────────────────────────────────────────────
    # Target computation
    # ─────────────────────────────────────────────────────────────────────────

    def _compute_absolute_target(self, goal):
        """Resolve the goal into absolute pool-frame coordinates."""
        current = self.current_pose.pose
        r_current = Rotation.from_quat([current.orientation.x, current.orientation.y, 
                                        current.orientation.z, current.orientation.w])

        if not goal.is_relative:
            # Absolute target - use directly
            target_x = goal.target_pose.position.x
            target_y = goal.target_pose.position.y
            target_z = goal.target_pose.position.z
            r_goal = Rotation.from_quat([
                goal.target_pose.orientation.x, goal.target_pose.orientation.y,
                goal.target_pose.orientation.z, goal.target_pose.orientation.w
            ])
            r_target = r_goal
        else:
            vec_offset = np.array([
                goal.target_pose.position.x,
                goal.target_pose.position.y,
                goal.target_pose.position.z
            ])

            if goal.is_local_frame:
                # Rotate body-frame offsets by the AUV's full 3D orientation
                pool_vec = r_current.apply(vec_offset)
                target_x = current.position.x + pool_vec[0]
                target_y = current.position.y + pool_vec[1]
                target_z = current.position.z + pool_vec[2]
            else:
                # Field-centric: offsets are already in pool frame
                target_x = current.position.x + vec_offset[0]
                target_y = current.position.y + vec_offset[1]
                target_z = current.position.z + vec_offset[2]

            # Convert goal rotation to Scipy Rotation
            r_goal = Rotation.from_quat([
                goal.target_pose.orientation.x, goal.target_pose.orientation.y,
                goal.target_pose.orientation.z, goal.target_pose.orientation.w
            ])
            
            # Local rotation composition: R_target = R_current * R_goal
            r_target = r_current * r_goal

        # Merged Quaternion Strategy: only use target angles for active DOFs
        # To prevent drift on uncontrolled DOFs over successive goals, we inherit
        # from the last explicitly commanded setpoint.
        if self._last_quat_setpoint is not None:
            r_base = Rotation.from_quat([
                self._last_quat_setpoint.x, self._last_quat_setpoint.y,
                self._last_quat_setpoint.z, self._last_quat_setpoint.w
            ])
        else:
            r_base = r_current

        base_euler = r_base.as_euler('ZYX', degrees=False)
        target_euler = r_target.as_euler('ZYX', degrees=False)
        
        effective_euler = [
            target_euler[0] if goal.do_yaw else base_euler[0],
            target_euler[1] if goal.do_pitch else base_euler[1],
            target_euler[2] if goal.do_roll else base_euler[2],
        ]
        
        r_effective = Rotation.from_euler('ZYX', effective_euler, degrees=False)
        q_arr = r_effective.as_quat()
        target_quat = Quaternion(x=q_arr[0], y=q_arr[1], z=q_arr[2], w=q_arr[3])

        return target_x, target_y, target_z, target_quat

    def _compute_angular_error(self, goal, current_quat, target_quat):
        """Compute angular error considering only controlled DOFs."""
        if not (goal.do_yaw or goal.do_pitch or goal.do_roll):
            return 0.0

        r_current = Rotation.from_quat([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        r_target = Rotation.from_quat([target_quat.x, target_quat.y, target_quat.z, target_quat.w])

        current_euler = r_current.as_euler('ZYX', degrees=False)
        target_euler = r_target.as_euler('ZYX', degrees=False)

        eval_euler = [
            target_euler[0] if goal.do_yaw else current_euler[0],
            target_euler[1] if goal.do_pitch else current_euler[1],
            target_euler[2] if goal.do_roll else current_euler[2],
        ]

        r_eval = Rotation.from_euler('ZYX', eval_euler, degrees=False)
        r_error = r_eval * r_current.inv()
        return r_error.magnitude()

    def _compute_position_error(self, goal, current, target_x, target_y, target_z):
        """Compute position error considering only controlled DOFs."""
        err_sq = 0.0
        if goal.do_x:
            err_sq += (target_x - current.position.x) ** 2
        if goal.do_y:
            err_sq += (target_y - current.position.y) ** 2
        if goal.do_z:
            err_sq += (target_z - current.position.z) ** 2
        return math.sqrt(err_sq)

    # ─────────────────────────────────────────────────────────────────────────
    # Setpoint publishing
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_setpoints(self, goal, target_x, target_y, target_z, target_quat):
        """Publish setpoints to the relevant PID controllers."""
        if goal.do_z:
            msg = Float64()
            msg.data = -target_z
            self.pub_depth_sp.publish(msg)

        if goal.do_x:
            msg = Float64()
            msg.data = target_x
            self.pub_x_sp.publish(msg)

        if goal.do_y:
            msg = Float64()
            msg.data = target_y
            self.pub_y_sp.publish(msg)

        if goal.do_yaw or goal.do_roll or goal.do_pitch:
            self.pub_quat_sp.publish(target_quat)
            self._last_quat_setpoint = target_quat

    # ─────────────────────────────────────────────────────────────────────────
    # Utilities
    # ─────────────────────────────────────────────────────────────────────────

    def _log_target(self, goal, x, y, z, target_quat):
        """Log a human-readable description of the goal."""
        parts = []
        if goal.do_x:
            parts.append(f'x={x:.2f}')
        if goal.do_y:
            parts.append(f'y={y:.2f}')
        if goal.do_z:
            parts.append(f'z={z:.2f}')
        if goal.do_yaw or goal.do_roll or goal.do_pitch:
            r = Rotation.from_quat([target_quat.x, target_quat.y, target_quat.z, target_quat.w])
            euler = r.as_euler('ZYX', degrees=True)
            if goal.do_yaw:
                parts.append(f'yaw={euler[0]:.1f}deg')
            if goal.do_pitch:
                parts.append(f'pitch={euler[1]:.1f}deg')
            if goal.do_roll:
                parts.append(f'roll={euler[2]:.1f}deg')

        mode = 'absolute'
        if goal.is_relative and goal.is_local_frame:
            mode = 'local frame relative'
        elif goal.is_relative:
            mode = 'map frame relative'

        self.get_logger().info(
            f'Navigating ({mode}): {", ".join(parts)} '
            f'[tol={goal.position_tolerance:.2f}m, '
            f'ang_tol={math.degrees(goal.angular_tolerance):.1f}deg, '
            f'hold={goal.hold_time:.1f}s, '
            f'timeout={goal.timeout:.1f}s]'
        )

    async def _sleep(self, seconds: float):
        """Async sleep using a one-shot ROS timer."""
        future = rclpy.task.Future()

        def _done():
            future.set_result(True)
            timer.cancel()

        timer = self.create_timer(seconds, _done, callback_group=self.cb_group)
        await future


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()

    # MultiThreadedExecutor needed for concurrent action execution + subscriptions
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
