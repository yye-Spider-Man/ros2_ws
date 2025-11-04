#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomSimulator(Node):
    def __init__(self):
        super().__init__('odom_simulator')
        
        # 创建里程计发布器
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅激光雷达里程计
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',  # rf2o激光雷达里程计的话题
            self.odom_callback,
            10)
        
        self.get_logger().info('Odom Simulator has been started.')
        
    def odom_callback(self, msg):
        # 转发里程计消息，将frame_id改为odom
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # 复制位置和方向信息
        odom.pose = msg.pose
        odom.twist = msg.twist
        
        # 发布里程计消息
        self.odom_publisher.publish(odom)
        
        # 创建并发布TF变换
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_simulator = OdomSimulator()
    try:
        rclpy.spin(odom_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        odom_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()