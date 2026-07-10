#!/usr/bin/env python3
"""ROS 2 node that adapts OccupancyGrid data to the Hybrid A* planner."""

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage

from .hybrid_astar import (
    extract_trajectory, hybrid_astar, Node as SearchNode, SearchConfig)


class HybridAlgorithmPlanner(Node):
    """Plan a kinematically feasible path whenever a new goal is received."""

    def __init__(self) -> None:
        super().__init__("hybrid_algorithm_planner")
        self.map_data = None
        self.start = None
        self.goal = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("use_odom_start", True)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("motion_step", 0.20)
        self.declare_parameter("wheelbase", 0.40)
        self.declare_parameter("goal_tolerance", 0.10)
        self.declare_parameter("max_steering_angle_deg", 30.0)
        self.declare_parameter("heading_bins", 72)
        self.declare_parameter("max_iterations", 250000)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.start_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )
        self.path_publisher = self.create_publisher(
            Path, self.get_parameter("path_topic").value, 10
        )
        self.get_logger().info("Hybrid A* planner is ready; waiting for map and pose.")

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Convert, threshold and inflate a ROS occupancy grid."""
        if msg.info.resolution <= 0.0:
            self.get_logger().error("Rejected map with non-positive resolution.")
            return

        raw_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        threshold = int(self.get_parameter("occupied_threshold").value)
        binary_map = raw_data > threshold
        if bool(self.get_parameter("unknown_is_occupied").value):
            binary_map |= raw_data < 0

        robot_radius = float(self.get_parameter("robot_radius").value)
        inflation_cells = max(0, math.ceil(robot_radius / msg.info.resolution))
        if inflation_cells:
            y, x = np.ogrid[
                -inflation_cells:inflation_cells + 1,
                -inflation_cells:inflation_cells + 1,
            ]
            circular_kernel = x * x + y * y <= inflation_cells * inflation_cells
            binary_map = ndimage.binary_dilation(
                binary_map, structure=circular_kernel
            )

        self.map_data = binary_map.astype(np.uint8)
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().info(
            f"Map ready: {msg.info.width}x{msg.info.height}, "
            f"resolution={msg.info.resolution:.3f} m, "
            f"inflation={inflation_cells} cells."
        )

    def start_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Use RViz's initial pose when odometry-based starts are disabled."""
        if bool(self.get_parameter("use_odom_start").value):
            return
        self.start = self._pose_to_node(msg.pose.pose)

    def odom_callback(self, msg: Odometry) -> None:
        """Continuously update the planning start from odometry."""
        if not bool(self.get_parameter("use_odom_start").value):
            return
        self.start = self._pose_to_node(msg.pose.pose)

    def goal_callback(self, msg: PoseStamped) -> None:
        """Store a goal and start planning."""
        self.goal = self._pose_to_node(msg.pose)
        if self.goal is None:
            self.get_logger().warn("Goal ignored because no map has been received.")
            return
        self.plan()

    def _pose_to_node(self, pose):
        if self.map_resolution is None:
            return None
        x = math.floor((pose.position.x - self.map_origin_x) / self.map_resolution)
        y = math.floor((pose.position.y - self.map_origin_y) / self.map_resolution)
        return SearchNode(x, y, self._get_yaw(pose.orientation))

    @staticmethod
    def _get_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _search_config(self) -> SearchConfig:
        resolution = self.map_resolution
        return SearchConfig(
            step_size=float(self.get_parameter("motion_step").value) / resolution,
            wheelbase=float(self.get_parameter("wheelbase").value) / resolution,
            goal_tolerance=float(
                self.get_parameter("goal_tolerance").value
            ) / resolution,
            max_steering_angle=math.radians(
                float(self.get_parameter("max_steering_angle_deg").value)
            ),
            heading_bins=int(self.get_parameter("heading_bins").value),
            max_iterations=int(self.get_parameter("max_iterations").value),
        )

    def plan(self) -> None:
        """Run Hybrid A* and publish the resulting path in the map frame."""
        if self.map_data is None or self.start is None or self.goal is None:
            self.get_logger().warn("Planning requires a map, start pose and goal pose.")
            return

        started_at = time.perf_counter()
        result = hybrid_astar(
            self.start, self.goal, self.map_data, self._search_config()
        )
        elapsed = time.perf_counter() - started_at
        if result is None:
            self.get_logger().warn(f"No path found after {elapsed:.3f} s.")
            return

        trajectory = extract_trajectory(result)
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y, yaw in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = (
                (x + 0.5) * self.map_resolution + self.map_origin_x
            )
            pose.pose.position.y = (
                (y + 0.5) * self.map_resolution + self.map_origin_y
            )
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(
            f"Published {len(trajectory)} path poses in {elapsed:.3f} s."
        )


def main(args=None) -> None:
    """Run the Hybrid A* planner node."""
    rclpy.init(args=args)
    node = HybridAlgorithmPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
