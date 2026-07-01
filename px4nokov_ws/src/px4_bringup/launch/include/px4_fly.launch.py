from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch 1：T265 相机（立即启动）
    t265_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_t265_launch.py'
            ])
        )
    )

    vision_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('vision_to_mavros'),
                        'launch',
                        't265_tf_to_mavros_launch.py'
                    ])
                )
            )
        ]
    )

    mavros_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('px4_bringup'),
                        'launch',
                        'include',
                        'px4.launch.py'
                    ])
                )
            )
        ]
    )

    return LaunchDescription([
        t265_launch,
        vision_launch,
        mavros_launch
    ])
