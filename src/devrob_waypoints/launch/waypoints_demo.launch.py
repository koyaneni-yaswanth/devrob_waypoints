from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1) Load MoveIt configuration for the Panda robot
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda",
            package_name="moveit_resources_panda_moveit_config"
        )
        .to_moveit_configs()
    )

    # 2) Waypoints YAML argument
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("devrob_waypoints"),
            "config",
            "waypoints.yaml",
        ]),
        description="Path to YAML file containing end-effector waypoints",
    )
    waypoints_file = LaunchConfiguration("waypoints_file")

    # 3) move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # 4) robot_state_publisher (publishes TF for the robot)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # 5) RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # 6) Your waypoint node
    waypoint_node = Node(
        package="devrob_waypoints",
        executable="waypoint_node",
        name="waypoint_node",
        output="screen",
        parameters=[
            {
                "planning_group": "panda_arm",
                "waypoints_file": waypoints_file,
            },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        waypoints_file_arg,
        move_group_node,
        robot_state_publisher_node,   # <-- added
        rviz_node,
        waypoint_node,
    ])

