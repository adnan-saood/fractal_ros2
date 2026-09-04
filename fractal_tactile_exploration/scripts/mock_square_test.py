"""Opt-in Cartesian square test for the MoveIt mock Panda hardware."""

from geometry_msgs.msg import Pose


class MockSquareTest:
    """Executes a configurable XY square one collision-checked pose at a time."""

    def __init__(self, node, executor):
        self.node = node
        self.executor = executor
        self.enabled = node.declare_parameter('motion_test.enabled', False).value
        self.xy = node.declare_parameter(
            'motion_test.square_xy', [0.30, -0.05, 0.40, -0.05, 0.40, 0.05, 0.30, 0.05]).value
        self.z = node.declare_parameter('motion_test.z', 0.50).value
        self.orientation = node.declare_parameter(
            'motion_test.orientation_xyzw', [0.0, 1.0, 0.0, 0.0]).value
        self.start_delay = node.declare_parameter('motion_test.start_delay', 2.0).value
        self.index = 0
        self.timer = None
        if self.enabled:
            self.validate_parameters()
            self.timer = node.create_timer(self.start_delay, self.start)

    def validate_parameters(self):
        if len(self.xy) < 8 or len(self.xy) % 2:
            raise ValueError('motion_test.square_xy must contain at least four XY coordinate pairs.')
        if len(self.orientation) != 4:
            raise ValueError('motion_test.orientation_xyzw must contain four values.')

    def start(self):
        if not self.executor.server_is_ready():
            self.node.get_logger().info('Waiting for MoveIt /move_action before starting square test.')
            return
        self.timer.cancel()
        self.node.get_logger().info(
            f'Starting mock XY square test with {len(self.xy) // 2} waypoints.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.index >= len(self.xy) // 2:
            self.node.get_logger().info('Mock XY square test completed.')
            return
        point_index = 2 * self.index
        x, y = self.xy[point_index:point_index + 2]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, self.z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self.orientation
        self.node.get_logger().info(
            f'Planning square waypoint {self.index + 1}: x={x:.3f}, y={y:.3f}, z={self.z:.3f}.')
        self.executor.execute_pose(pose, self.on_waypoint_complete)

    def on_waypoint_complete(self, succeeded):
        if not succeeded:
            self.node.get_logger().error(f'Mock square waypoint {self.index + 1} failed.')
            return
        self.index += 1
        self.send_next_waypoint()