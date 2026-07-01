from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 第一步：启动 px4_fly.launch.py
    launch_px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('px4_bringup'),
                'launch',
                'include',
                'px4_fly.launch.py'
            ])
        )
    )

    launch_serial_image = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('px4_bringup'),
                        'launch',
                        'serial_and_image_2025TI.launch.py'
                    ])
                )
            )
        ]
    )

    launch_cpp_node = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'offboard_cpp', '2025_Ti_main_node'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        launch_px4,
        launch_serial_image,
        launch_cpp_node
    ])
