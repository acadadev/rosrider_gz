import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('rosrider_gz_bringup')
    pkg_project_gazebo = get_package_share_directory('rosrider_gz_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # setup to launch the simulator and gazebo world, autostarted with the -r flag
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r {os.path.join(pkg_project_gazebo, "worlds", "simple_tracked_world.sdf")}'
        }.items(),
    )

    # bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'simple_tracked_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])
