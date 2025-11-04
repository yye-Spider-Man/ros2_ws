#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # 配置文件路径
    slam_params_file = os.path.join(slam_pkg_dir, 'config', 'slam_config.yaml')
    nav2_params_file = os.path.join(slam_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(slam_pkg_dir, 'config', 'rviz_config.rviz')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 启动SLAM工具箱
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 启动AMCL和Nav2导航栈
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )
    
    # 启动TF变换 - laser到base_link
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0', 
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 启动RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        slam_toolbox_launch,
        nav2_bringup,
        base_to_laser_tf,
        rviz,
    ])