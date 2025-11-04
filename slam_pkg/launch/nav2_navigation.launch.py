import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default='/media/l-pider/6A84B6FF84B6CCB7/slam_map.yaml')
    
    # 1) 传感器融合
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg_dir, 'launch', 'lidar_and_imu.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2) Nav2完整导航系统（Localization + Navigation）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': os.path.join(slam_pkg_dir, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
            'use_remappings': 'true'
        }.items()
    )

    # 3) RViz 可视化
    rviz_config = os.path.join(slam_pkg_dir, 'rviz', 'nav2_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4) Launch 描述
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('map', default_value='/media/l-pider/6A84B6FF84B6CCB7/slam_map.yaml', description='Full path to map yaml'))
    ld.add_action(sensor_fusion)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)

    return ld