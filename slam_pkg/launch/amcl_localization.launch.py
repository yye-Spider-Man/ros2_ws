import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')  # 无默认，必须指定

    # 1. 您的传感器融合 (lidar_and_imu.py)
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg_dir, 'launch', 'lidar_and_imu.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # 调整参数如果有
    )

    # 2. Map Server (加载指定的地图)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
    )

    # 3. AMCL (定位)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(slam_pkg_dir, 'config', 'amcl_params.yaml'), {'use_sim_time': use_sim_time}]
    )

    # 4. RViz (可视化)
    rviz_config = os.path.join(slam_pkg_dir, 'rviz', 'amcl_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. 新增：Lifecycle Manager (自动激活 map_server 和 amcl)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,  # 自动 configure -> activate
            'node_names': ['map_server', 'amcl']  # 指定节点，按序激活 (map_server 先)
        }]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('map', description='Full path to map yaml'))  # 无默认，必须指定
    ld.add_action(sensor_fusion)
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(rviz_node)
    ld.add_action(lifecycle_manager)  # 添加此行，自动管理

    return ld
