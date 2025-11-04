# sensor_fusion.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    slam_pkg_dir = get_package_share_directory('slam_pkg')
    
    # 配置文件路径
    ekf_config = os.path.join(slam_pkg_dir, 'config', 'ekf.yaml')
    imu_config = os.path.join(slam_pkg_dir, 'config', 'imu.yaml')  # 明确 IMU YAML 路径
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # 1. 启动RPLidar C1驱动
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()  # 同步 sim_time
    )
    
    # 2. 启动IMU驱动（合并 parameters：YAML + use_sim_time）
    imu_driver = Node(
        package='dm_imu_ros2',
        executable='dm_imu_node',
        name='dm_imu_node',
        output='screen',
        parameters=[imu_config, {'use_sim_time': use_sim_time}],  # 合并，避免重复
        remappings=[('/imu/data_raw', '/imu/data')]  # 重映射到标准话题，EKF 订阅 /imu/data
    )
    
    # 3. 启动RF2O激光雷达里程计（已合并参数）
    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',  # 确认订阅 RPLIDAR /scan（非 /laser_scan）
            'odom_topic': '/odom_rf2o',  # 输出话题
            'publish_tf': True,  # 发布 base_link -> odom TF
            'base_frame_id': 'base_link',  # 统一到 AMCL/Nav2 标准
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',  # 无初始位话题
            'freq': 10.0,  # 匹配 /scan hz，避丢帧警告
            'min_ray_number': 200,  # RPLIDAR 最小射线数，避 "degenerate scan"
            'queue_size': 10,  # QoS 缓冲，减 USB 延迟
            'use_sim_time': use_sim_time  # 同步
        }]
    )
    
    # 4. 启动EKF节点进行传感器融合（合并 parameters：YAML + use_sim_time）
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],  # 合并，避免重复
        remappings=[('odometry/filtered', '/odometry/local')]
    )
    
    # 5. 静态TF变换: base_footprint到base_link（加 parameters）
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0',  # z=0，无高（调整您的底盘）
                   'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]  # 添加 sim_time
    )
    
    # 6. 静态TF变换: base_link到laser（加 parameters）
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0',
                   'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 7. 静态TF变换: base_link到imu_link（加 parameters）
    base_link_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0',
                   'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_use_sim_time_cmd)
    
    # 添加节点和launch包含
    ld.add_action(rplidar_launch)
    ld.add_action(imu_driver)
    ld.add_action(rf2o_laser_odometry)
    ld.add_action(ekf_localization)
    ld.add_action(base_footprint_to_base_link)
    ld.add_action(base_link_to_laser)
    ld.add_action(base_link_to_imu)
    
    return ld
