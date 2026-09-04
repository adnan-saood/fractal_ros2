"""MoveIt action-client transport for collision-checked tool poses."""

from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from shape_msgs.msg import SolidPrimitive


class MoveItExecutor:
    """Plans and executes an Omega tool pose through MoveIt's move action."""

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, MoveGroup, '/move_action')
        self.group_name = node.declare_parameter('motion_test.group_name', 'panda_arm').value
        self.tool_link = node.declare_parameter('motion_test.tool_link', 'omega_contact_tip').value
        self.frame_id = node.declare_parameter('motion_test.frame_id', 'world').value
        self.position_tolerance = node.declare_parameter(
            'motion_test.position_tolerance', 0.005).value
        self.orientation_tolerance = node.declare_parameter(
            'motion_test.orientation_tolerance', 0.05).value

    def server_is_ready(self):
        return self.client.server_is_ready()

    def execute_pose(self, pose, on_complete):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.goal_constraints = [self.make_pose_constraint(pose)]
        goal.planning_options.plan_only = False
        self.client.send_goal_async(goal).add_done_callback(
            lambda future: self.on_goal_response(future, on_complete))

    def make_pose_constraint(self, pose):
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.frame_id
        position_constraint.link_name = self.tool_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        position_constraint.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[pose])
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.frame_id
        orientation_constraint.link_name = self.tool_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        return Constraints(
            position_constraints=[position_constraint],
            orientation_constraints=[orientation_constraint])

    def on_goal_response(self, future, on_complete):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('MoveIt rejected the motion goal.')
            on_complete(False)
            return
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self.on_goal_result(result_future, on_complete))

    def on_goal_result(self, future, on_complete):
        result = future.result().result
        on_complete(result.error_code.val == MoveItErrorCodes.SUCCESS)