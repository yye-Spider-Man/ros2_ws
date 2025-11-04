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

    # 1. 您的传感器融合 (lidar_and_imu.py)
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg_dir, 'launch', 'lidar_and_imu.py')),  # 修改这里
        launch_arguments={'use_sim_time': use_sim_time}.items()  # 如果有参数，调整
    )

    # 2. SLAM Toolbox (实时建图)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch/online_async_launch.py']),
        launch_arguments={
            'slam_params_file': os.path.join(slam_pkg_dir, 'config', 'slam_params.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # 3. RViz (可视化)
    rviz_config = os.path.join(slam_pkg_dir, 'rviz', 'slam_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time'))
    ld.add_action(sensor_fusion)
    ld.add_action(slam_launch)
    ld.add_action(rviz_node)

    return ld
