# I wrote this code following the template offered in your GitHub repository, specifically the preQual/retangle_with_yaw_qualification.py
# I have modified a few things to adapt it to the Google Form question.
# For instances, the AUV moves in a square orbit instead of a rectangle, and always faces inward the square instead of along its trajectory.
# Finally, I adjusted the format to respect the template presented in your question with mission choice check and mission complete behaviours.

# Python dependencies
import math

# ROS dependencies
import py_trees
import py_trees_ros

from py_trees.common import Status, Access
from py_trees.blackboard import Client

import rclpy
from rclpy.node import Node

# AUV dependencies
from controls import navigation_client
from controls.goal_helpers import move_robot_centric, rotate_relative, set_depth 

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour

class CheckDepth(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckDepth"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self, **kwargs):
        self.node = kwargs.get('node', None)
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)
    
    def update(self):
        pose = self.blackboard.sensors.pose
        if pose.position.z > -1.6:
            self.node.get_logger().info("Abort")
            return py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().info("Continue")
            return py_trees.common.Status.SUCCESS

class MySelector(py_trees.composites.Selector):
    def __init__(self, position_tolerance: float, hold_time: float, timeout: float, name="MySelector"):
        super().__init__(name, memory=True)
        check_depth_leaf = CheckDepth()
        dive_leaf = BasicActionBehaviour("Dive to 0", set_depth(z=0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))

        self.add_children([check_depth_leaf, dive_leaf])


class SquareOrbitMission(py_trees.composites.Sequence):
    """
    This PyTrees Sequence is the root of the square orbit mission
    """
    def __init__(self, angular_tolerance: float, position_tolerance: float, hold_time: float, timeout: float):
        super().__init__("SquareOrbit", memory=True)

       
        dive_leaf = BasicActionBehaviour("Dive to -1.0", set_depth(z=-1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        forward_move_leaf = BasicActionBehaviour("Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        yaw_rotate_leaf = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi / 2, tolerance=angular_tolerance, hold_time=hold_time, timeout=timeout))
        forward_move_leaf1 = BasicActionBehaviour("Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        yaw_rotate_leaf1 = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi / 2, tolerance=angular_tolerance, hold_time=hold_time, timeout=timeout))

        selector_subtree = MySelector(position_tolerance=position_tolerance, hold_time=hold_time, timeout=timeout)

        forward_move_leaf2 = BasicActionBehaviour("Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        yaw_rotate_leaf2 = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi / 2, tolerance=angular_tolerance, hold_time=hold_time, timeout=timeout))
        forward_move_leaf3 = BasicActionBehaviour("Move to forward", move_robot_centric(forward=1.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout))
        yaw_rotate_leaf3 = BasicActionBehaviour("Yaw Test", rotate_relative(dyaw_rad=math.pi / 2, tolerance=angular_tolerance, hold_time=hold_time, timeout=timeout))



        self.add_children([
            dive_leaf,
            forward_move_leaf,
            yaw_rotate_leaf,
            forward_move_leaf1,
            yaw_rotate_leaf1,

            selector_subtree,

            forward_move_leaf2, 
            yaw_rotate_leaf2,
            forward_move_leaf3,
            yaw_rotate_leaf3
            ])