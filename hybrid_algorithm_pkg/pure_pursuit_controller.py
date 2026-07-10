#!/usr/bin/env python3
"""Pure-pursuit path tracking controller for differential-drive robots."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class PurePursuitController(Node):
    """Track the planner's Path output with bounded velocity commands."""

    def __init__(self) -> None:
        super().__init__("pure_pursuit_controller")
        self.path = []
        self.current_pose = None
        self.closest_index = 0

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("lookahead_distance", 0.4)
        self.declare_parameter("target_speed", 0.2)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("control_frequency", 20.0)

        self.create_subscription(
            Path, self.get_parameter("path_topic").value, self.path_callback, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        frequency = float(self.get_parameter("control_frequency").value)
        if frequency <= 0.0:
            raise ValueError("control_frequency must be positive")
        self.timer = self.create_timer(1.0 / frequency, self.control_loop)
        self.get_logger().info("Pure pursuit controller is ready.")

    def path_callback(self, msg: Path) -> None:
        """Replace the active path with a newly planned one."""
        self.path = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self.closest_index = 0
        self.get_logger().info(f"Received a path with {len(self.path)} poses.")

    def odom_callback(self, msg: Odometry) -> None:
        """Update the current planar robot pose."""
        position = msg.pose.pose.position
        self.current_pose = (
            position.x,
            position.y,
            self._get_yaw(msg.pose.pose.orientation),
        )

    @staticmethod
    def _get_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self) -> None:
        """Publish one bounded pure-pursuit control command."""
        if not self.path or self.current_pose is None:
            self._stop()
            return

        current_x, current_y, current_yaw = self.current_pose
        goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        distance_to_goal = math.hypot(
            self.path[-1][0] - current_x, self.path[-1][1] - current_y
        )
        if distance_to_goal <= goal_tolerance:
            self.path = []
            self._stop()
            self.get_logger().info("Goal reached.")
            return

        self.closest_index = min(
            range(self.closest_index, len(self.path)),
            key=lambda index: math.hypot(
                self.path[index][0] - current_x,
                self.path[index][1] - current_y,
            ),
        )
        lookahead = float(self.get_parameter("lookahead_distance").value)
        if lookahead <= 0.0:
            self.get_logger().error("lookahead_distance must be positive.")
            self._stop()
            return

        target_index = len(self.path) - 1
        for index in range(self.closest_index, len(self.path)):
            distance = math.hypot(
                self.path[index][0] - current_x,
                self.path[index][1] - current_y,
            )
            if distance >= lookahead:
                target_index = index
                break

        target_x, target_y = self.path[target_index]
        alpha = math.atan2(
            target_y - current_y, target_x - current_x
        ) - current_yaw
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi

        target_speed = float(self.get_parameter("target_speed").value)
        max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        command = Twist()
        command.linear.x = target_speed
        angular_speed = 2.0 * target_speed * math.sin(alpha) / lookahead
        command.angular.z = max(
            -max_angular_speed, min(max_angular_speed, angular_speed)
        )
        self.cmd_pub.publish(command)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    """Run the pure-pursuit controller node."""
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
