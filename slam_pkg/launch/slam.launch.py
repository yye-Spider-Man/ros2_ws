# slam.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取包路径
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    
    # 配置文件路径
    config_file = os.path.join(slam_pkg_dir, 'config', 'slam_config.yaml')
    
    # 声明参数
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # 雷达启动文件路径
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )
    
    # 声明launch参数
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
    
    # 启动雷达驱动
    rplidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file)
    )
    
    # 里程计模拟器 - 提供50Hz的odom到base_link变换
    odom_simulator = Node(
        package='slam_pkg',
        executable='odom_simulator.py',
        name='odom_simulator',
        output='screen'
    )
    
    # TF变换 - laser到base_link
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0', 
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )
    
    # SLAM节点
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # RViz可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_use_rviz_cmd)
    
    # 添加节点
    ld.add_action(rplidar_driver)
    ld.add_action(odom_simulator)
    ld.add_action(base_to_laser_tf)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    
    return ld