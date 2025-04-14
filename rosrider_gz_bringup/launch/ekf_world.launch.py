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
    pkg_project_description = get_package_share_directory('rosrider_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # load the URDF file from description package
    urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'next.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # setup to launch the simulator and gazebo world, autostarted with the -r flag
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-r {os.path.join(pkg_project_gazebo, "worlds", "ekf_world.sdf")}'
        }.items(),
    )

    # takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True },
            {'robot_description': robot_description},
            {'frame_prefix': 'next/'}
        ]
    )

    # visualize in RVIZ
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'simulation.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'rosrider_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'next/imu_link', 'next/imu_link/imu_sensor']
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RVIZ'),
        bridge,
        robot_state_publisher,
        transform,
        rviz
    ])
