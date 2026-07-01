from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    my_package_name = 'px4_bringup'
    mavros_pkg_share = get_package_share_directory('mavros')
    mavros_launch_file = os.path.join(mavros_pkg_share, 'launch', 'px4.launch')
    my_pkg_share = get_package_share_directory(my_package_name)
    yaml_config_path = os.path.join(my_pkg_share, 'config', 'mavros_params.yaml')
    if os.path.exists(yaml_config_path):
        with open(yaml_config_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
            yaml_params = yaml_data.get('/**', {}).get('ros__parameters', {})
    else:
        yaml_params = {}

    declared_args = [
        DeclareLaunchArgument('fcu_url', default_value=yaml_params.get('fcu_url', '/dev/ttyACM0:57600')),
        DeclareLaunchArgument('gcs_url', default_value=yaml_params.get('gcs_url', '')),
        DeclareLaunchArgument('tgt_system', default_value=str(yaml_params.get('tgt_system', '1'))),
        DeclareLaunchArgument('tgt_component', default_value=str(yaml_params.get('tgt_component', '1'))),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value=yaml_params.get('fcu_protocol', 'v2.0')),
        DeclareLaunchArgument('respawn_mavros', default_value=str(yaml_params.get('respawn_mavros', 'false'))),
        DeclareLaunchArgument('namespace', default_value=yaml_params.get('namespace', 'mavros')),
    ]

    def launch_setup(context, *args, **kwargs):
        return [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(mavros_launch_file),
                launch_arguments={
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'tgt_system': LaunchConfiguration('tgt_system'),
                    'tgt_component': LaunchConfiguration('tgt_component'),
                    'log_output': LaunchConfiguration('log_output'),
                    'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                    'respawn_mavros': LaunchConfiguration('respawn_mavros'),
                    'namespace': LaunchConfiguration('namespace'),
                }.items()
            )
        ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])