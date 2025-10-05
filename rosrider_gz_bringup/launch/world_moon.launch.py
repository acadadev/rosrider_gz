import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('rosrider_gz_bringup')
    pkg_project_gazebo = get_package_share_directory('rosrider_gz_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_ukf = LaunchConfiguration('use_ukf', default='false')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r {os.path.join(pkg_project_gazebo, "worlds", "world_moon.sdf")}'
        }.items(),
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'simulation_singular.rviz')],
       condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    bridge_explorer_r2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'bridge', 'explorer_r2_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[
                    {os.path.join(pkg_project_bringup, 'config', 'ukf_gazebo.yaml')},
                    {'use_sim_time': True }
                   ],
        condition=IfCondition(use_ukf)
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
                    {os.path.join(pkg_project_bringup, 'config', 'ekf_gazebo.yaml')},
                    {'use_sim_time': True }
                   ],
        condition=UnlessCondition(use_ukf)
    )

    front_laser_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_laser_static_transform',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0.35',
            '--y', '0',
            '--z', '0.55',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_laser'
        ]
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Open RVIZ'),
        DeclareLaunchArgument('use_ukf', default_value='false', description='Use UKF instead of EKF'),
        bridge_explorer_r2,
        ukf_node,
        ekf_node,
        rviz,
        front_laser_static_transform
    ])