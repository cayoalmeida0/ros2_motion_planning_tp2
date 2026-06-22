import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ld90_gz = get_package_share_directory('ld90_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_description = get_package_share_directory('amr_description')

    world = os.path.join(pkg_ld90_gz, 'worlds', 'empty.sdf')
    model_file = os.path.join(pkg_ld90_gz, 'models', 'ld90_gz.sdf')
    bridge_file = os.path.join(pkg_ld90_gz, 'config', 'bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_amr_description)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world}'
        }.items()
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ld90',
            '-file', model_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_file},
            {'qos_overrides./tf.publisher.durability': 'volatile'},
            {'qos_overrides./tf.publisher.reliability': 'reliable'},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        set_gz_resource_path,
        gz_sim,
        spawn,
        bridge
    ])