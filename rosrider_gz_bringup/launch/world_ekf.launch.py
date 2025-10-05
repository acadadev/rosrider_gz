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
    pkg_project_description = get_package_share_directory('rosrider_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_ukf = LaunchConfiguration('use_ukf', default='false')

    urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'next.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r {os.path.join(pkg_project_gazebo, "worlds", "world_ekf.sdf")}'
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True },
            {'robot_description': robot_description},
            {'frame_prefix': ''},
            {'publish_frequency': 20.0}
        ]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'simulation_singular.rviz')],
       condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'bridge', 'next_gz_singular_ekf_bridge.yaml'),
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

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Open RVIZ'),
        DeclareLaunchArgument('use_ukf', default_value='false', description='Use UKF instead of EKF'),
        bridge,
        ukf_node,
        ekf_node,
        robot_state_publisher,
        rviz
    ])
