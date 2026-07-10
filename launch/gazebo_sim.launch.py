"""Launch the project against a user-supplied Gazebo world and map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    robot_package = LaunchConfiguration("robot_description_package").perform(context)
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    world = LaunchConfiguration("world").perform(context)
    map_file = LaunchConfiguration("map").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_share = get_package_share_directory(robot_package)
    project_share = get_package_share_directory("hybrid_algorithm_pkg")
    urdf_path = os.path.join(robot_share, "urdf", urdf_file)

    with open(urdf_path, encoding="utf-8") as urdf_stream:
        robot_description = urdf_stream.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", world],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-g"],
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("headless")),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-name", robot_name,
                "-topic", "robot_description",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"/model/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ],
            remappings=[(f"/model/{robot_name}/tf", "/tf")],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
        ),
        Node(
            package="hybrid_algorithm_pkg",
            executable="hybrid_astar_planner",
            output="screen",
            parameters=[
                os.path.join(project_share, "config", "planner_params.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="hybrid_algorithm_pkg",
            executable="pure_pursuit_controller",
            output="screen",
            parameters=[
                os.path.join(project_share, "config", "planner_params.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(project_share, "config", "hybrid_algorithm.rviz"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description():
    """Declare portable simulation inputs and construct the launch graph."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            description="Absolute path to a Gazebo Sim world file.",
        ),
        DeclareLaunchArgument(
            "map",
            description="Absolute path to a Nav2 map YAML file.",
        ),
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="fishbot_description",
        ),
        DeclareLaunchArgument("urdf_file", default_value="fishbot_base.urdf"),
        DeclareLaunchArgument("robot_name", default_value="fishbot"),
        DeclareLaunchArgument("spawn_x", default_value="-2.0"),
        DeclareLaunchArgument("spawn_y", default_value="1.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.1"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
