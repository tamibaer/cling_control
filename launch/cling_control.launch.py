from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_table").to_moveit_configs()

    rviz_config = os.path.join(
    get_package_share_directory("cling_control"),
        "config",
        "config.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            {"trajectory_execution.allowed_start_tolerance": 0.0},
        ],
    )

    cling_control = Node(
        package="cling_control",
        executable="cling_control_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_name": "D415",
            "serial_no": '"241222063543"',
        }.items(),
    )

    camera_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.018", "0.757", "1.204",
            "0.408", "0.429", "-0.567", "0.573",
            "ur5e_base_link", "D415_link",
        ],
    )

    return LaunchDescription([
        rviz_node,
        move_group_node,
        cling_control,
        realsense_launch,
        camera_static_transform,
    ])