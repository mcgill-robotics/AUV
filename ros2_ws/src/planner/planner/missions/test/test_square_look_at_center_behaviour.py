# Python dependencies
import math

# ROS dependencies
import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node

# AUV dependencies
from controls import navigation_client
from controls.goal_helpers import set_depth, move_to_and_look_at

# Planner dependencies
from ..mission_behaviour_components import BasicActionBehaviour


# Assumptions:
# - Mission starts near pool origin (0,0), like the Rectangle Prequal does, so the
#   square corners are hardcoded absolute coords instead of relative to current pose.
# - "Always looks at center" = move_to_and_look_at() with reference = square center,
#   so yaw is set to face the center at every corner.
# - "Depth relative to floor" = current_z - floor_z. Floor depth is hardcoded here
#   (_POOL_FLOOR_Z); on the real sub this would come from a range sensor, not a constant.
# - If that value is > 0.5m at the halfway point, abort the rest of the square and surface.

_DIVE_DEPTH = -1.5
_POOL_FLOOR_Z = -3.0
_MAX_HEIGHT_ABOVE_FLOOR = 0.5

_SQUARE_CENTER = (0.0, 0.0)
_SQUARE_HALF_SIDE = 1.0  # 2m x 2m square

_CORNERS = [
    (_SQUARE_CENTER[0] + _SQUARE_HALF_SIDE, _SQUARE_CENTER[1] + _SQUARE_HALF_SIDE),
    (_SQUARE_CENTER[0] + _SQUARE_HALF_SIDE, _SQUARE_CENTER[1] - _SQUARE_HALF_SIDE),
    (_SQUARE_CENTER[0] - _SQUARE_HALF_SIDE, _SQUARE_CENTER[1] - _SQUARE_HALF_SIDE),
    (_SQUARE_CENTER[0] - _SQUARE_HALF_SIDE, _SQUARE_CENTER[1] + _SQUARE_HALF_SIDE),
]


class CheckAltitudeAboveFloorBehaviour(py_trees.behaviour.Behaviour):
    """Condition: is Dougie too high above the pool floor? Used as the halfway safety check."""

    def __init__(self, name="Check Altitude Above Floor"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.blackboard.register_key(key="/sensors/pose", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, 'sensors') or self.blackboard.sensors.pose is None:
            self.node.get_logger().info(f"[{self.name}] Waiting for pose...", throttle_duration_sec=2.0)
            return py_trees.common.Status.RUNNING

        current_z = self.blackboard.sensors.pose.position.z
        height_above_floor = current_z - _POOL_FLOOR_Z

        if height_above_floor > _MAX_HEIGHT_ABOVE_FLOOR:
            self.node.get_logger().warn(
                f"[{self.name}] {height_above_floor:.2f}m above floor (z={current_z:.2f}), aborting square."
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS


class TestSquareLookAtCenterBehaviour(py_trees.composites.Sequence):
    """Moves Dougie around a square, always facing the center. Aborts to surface if the
    halfway depth check fails."""

    def __init__(self, position_tolerance: float, angular_tolerance: float, hold_time: float, timeout: float):
        super().__init__("TestSquareLookAtCenterBehaviour", memory=True)

        cx, cy = _SQUARE_CENTER

        # 1. Dive to working depth
        dive_leaf = BasicActionBehaviour(
            "Dive to working depth",
            set_depth(z=_DIVE_DEPTH, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout),
        )

        # 2. Corners of the square, always facing the center
        corner_leaves = []
        for i, (x, y) in enumerate(_CORNERS, start=1):
            corner_leaves.append(BasicActionBehaviour(
                f"Square: corner {i} (look at center)",
                move_to_and_look_at(
                    target_x=x, target_y=y, target_z=_DIVE_DEPTH,
                    reference_x=cx, reference_y=cy,
                    position_tolerance=position_tolerance, yaw_tolerance=angular_tolerance,
                    hold_time=hold_time, timeout=timeout,
                ),
            ))

        first_half = corner_leaves[:2]
        second_half = corner_leaves[2:]

        # 3. Halfway safety gate: continue if the altitude check passes, else abort and surface
        continue_branch = py_trees.composites.Sequence("Continue Square", memory=True)
        continue_branch.add_children([
            CheckAltitudeAboveFloorBehaviour(),
            *second_half,
            BasicActionBehaviour(
                "Ascend to Surface",
                set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout),
            ),
        ])

        abort_branch = py_trees.composites.Sequence("Abort To Surface", memory=True)
        abort_branch.add_children([
            BasicActionBehaviour(
                "Abort: Ascend to Surface",
                set_depth(z=0.0, tolerance=position_tolerance, hold_time=hold_time, timeout=timeout),
            ),
        ])

        depth_safety_gate = py_trees.composites.Selector("Depth Safety Gate", memory=True)
        depth_safety_gate.add_children([continue_branch, abort_branch])

        self.add_children([
            dive_leaf,
            *first_half,
            depth_safety_gate,
        ])

