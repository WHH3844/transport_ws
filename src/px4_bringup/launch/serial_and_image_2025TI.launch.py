from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 启动 serial_driver
    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("serial_driver"),
                "launch",
                "serial_driver.launch.py"
            ])
        )
    )

    # 启动 opencv_cpp，延迟 3 秒
    opencv_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("opencv_cpp"),
                        "launch",
                        "image.launch.py"
                    ])
                )
            )
        ]
    )

    # 启动 yolo_node，延迟 5 秒（比起初始晚 5s）
    yolo_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("cv_yolo_paddle_pkg"),
                        "launch",
                        "yolo_node.launch.py"
                    ])
                )
            )
        ]
    )

    return LaunchDescription([
        serial_launch,
        opencv_launch,
        yolo_launch
    ])
