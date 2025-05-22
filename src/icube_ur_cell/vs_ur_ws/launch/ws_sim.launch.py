from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Lancement de robot_control avec use_fake_hardware
    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('icube_ur_cell_control'),
                'launch',
                'start_robot_control.launch.py'
            ])
        ]),
        launch_arguments={'use_fake_hardware': 'true'}.items()
    )

    # Lancement différé de move_group
    move_group_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('icube_ur_cell_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    ])
                ])
            )
        ]
    )

    # Lancement différé de RViz
    rviz_launch = TimerAction(
        period=4.0,  # après move_group
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('icube_ur_cell_moveit_config'),
                        'launch',
                        'moveit_rviz.launch.py'
                    ])
                ])
            )
        ]
    )

    # Lancement différé du node vs_ur_ws
    vs_ur_ws_node = TimerAction(
        period=7.0,  # laisse le temps à MoveIt de se lancer
        actions=[
            Node(
                package="vs_ur_ws",
                executable="vs_ur_ws",
                name="vs_ur_ws",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                ]
            )
        ]
    )

    return LaunchDescription([
        robot_control_launch,
        move_group_launch,
        rviz_launch,
        vs_ur_ws_node
    ])
