#!/usr/bin/env python3
"""Publish deterministic Franka force and L5325 tactile data for simulation."""

import math

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from franka_msgs.msg import FrankaState
from paxini_hardware.msg import TactileSensor, TactileTaxel
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class SimSensorNode(Node):
    def __init__(self):
        super().__init__('tactile_sim_sensor')
        self.map_frame = self.declare_parameter('map_frame', 'tactile_map').value
        self.tip_frame = self.declare_parameter('tip_frame', 'omega_contact_tip').value
        self.surface_mode = self.declare_parameter('surface.mode', 'bump').value
        self.surface_origin_x = self.declare_parameter('surface.origin_x', 0.0).value
        self.surface_origin_y = self.declare_parameter('surface.origin_y', 0.0).value
        self.surface_size_x = self.declare_parameter('surface.size_x', 0.4).value
        self.surface_size_y = self.declare_parameter('surface.size_y', 0.4).value
        self.surface_base_z = self.declare_parameter('surface.base_z', 0.0).value
        self.bump_x = self.declare_parameter('surface.bump_x', 0.05).value
        self.bump_y = self.declare_parameter('surface.bump_y', 0.05).value
        self.bump_height = self.declare_parameter('surface.bump_height', 0.008).value
        self.bump_sigma = self.declare_parameter('surface.bump_sigma', 0.04).value
        self.stiffness = self.declare_parameter('contact.stiffness', 180.0).value
        self.taxel_count = self.declare_parameter('taxel_count', 239).value
        self.force_publisher = self.create_publisher(
            FrankaState, '/franka_robot_state_broadcaster/robot_state', 10)
        self.tactile_publisher = self.create_publisher(
            TactileSensor, '/paxini/L5325_omega/tactile_sensor', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '~/surface_markers', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.taxel_layout = self._make_taxel_layout()
        self.create_timer(0.02, self.publish_sensor_state)
        self.create_timer(1.0, self.publish_surface_marker)

    def _make_taxel_layout(self):
        layout = []
        columns = 16
        rows = math.ceil(self.taxel_count / columns)
        for index in range(self.taxel_count):
            row, column = divmod(index, columns)
            x = (column - (columns - 1) / 2.0) * 0.004
            y = (row - (rows - 1) / 2.0) * 0.004
            layout.append((x, y))
        return layout

    def surface_height(self, x, y):
        if not (self.surface_origin_x <= x <= self.surface_origin_x + self.surface_size_x
                and self.surface_origin_y <= y <= self.surface_origin_y + self.surface_size_y):
            return self.surface_base_z - 1.0
        if self.surface_mode == 'plane':
            return self.surface_base_z
        distance_squared = (x - self.bump_x) ** 2 + (y - self.bump_y) ** 2
        return self.surface_base_z + self.bump_height * math.exp(
            -distance_squared / (2.0 * self.bump_sigma ** 2))

    def tip_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.tip_frame, rclpy.time.Time())
        except TransformException:
            return None

    def publish_sensor_state(self):
        transform = self.tip_transform()
        if transform is None:
            return
        stamp = self.get_clock().now().to_msg()
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z
        penetration = max(0.0, self.surface_height(x, y) - z)
        total_force = min(4.0, self.stiffness * penetration)

        state = FrankaState()
        state.header.stamp = stamp
        for index in range(3):
            state.k_f_ext_hat_k[index] = 0.0
        state.k_f_ext_hat_k[2] = total_force
        self.force_publisher.publish(state)

        tactile = TactileSensor()
        tactile.header.stamp = stamp
        tactile.header.frame_id = self.tip_frame
        tactile.resultant_force.z = total_force
        per_taxel_force = total_force / max(1, self.taxel_count)
        for index, (taxel_x, taxel_y) in enumerate(self.taxel_layout):
            taxel = TactileTaxel()
            taxel.index = index
            taxel.position = Point(x=taxel_x, y=taxel_y, z=0.0)
            taxel.normal.z = 1.0
            taxel.force.z = per_taxel_force
            tactile.taxels.append(taxel)
        self.tactile_publisher.publish(tactile)

    def publish_surface_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'analytic_surface'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.surface_origin_x + self.surface_size_x / 2.0
        marker.pose.position.y = self.surface_origin_y + self.surface_size_y / 2.0
        marker.pose.position.z = self.surface_base_z - 0.005
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.surface_size_x
        marker.scale.y = self.surface_size_y
        marker.scale.z = 0.01
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 0.9
        marker.color.a = 0.35
        self.marker_publisher.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    rclpy.spin(SimSensorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
