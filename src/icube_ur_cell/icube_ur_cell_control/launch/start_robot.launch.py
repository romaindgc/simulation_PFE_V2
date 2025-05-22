from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
 
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",  # put your robot's IP address here
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("icube_ur_cell_control"),
                                "launch",
                                "start_robot_full.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("icube_ur_cell_description"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "controllers_file": PathJoinSubstitution(
                        [
                            FindPackageShare("icube_ur_cell_control"),
                            "config",
                            "full_ros2_controllers.yaml",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("icube_ur_cell_control"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                    "update_rate_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("icube_ur_cell_control"),
                            "config",
                            "update_rate.yaml",
                        ]
                    ),
                }.items(),
            ),
        ]
    )
