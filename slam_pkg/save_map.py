#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        self.req = SaveMap.Request()

    def send_request(self, map_filename):
        self.req.map_url = map_filename
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    
    if len(sys.argv) != 2:
        print("用法: save_map.py <地图文件名>")
        return
    
    map_filename = sys.argv[1]
    
    # 确保路径存在
    map_dir = os.path.dirname(map_filename)
    if map_dir and not os.path.exists(map_dir):
        os.makedirs(map_dir)
    
    map_saver = MapSaver()
    response = map_saver.send_request(map_filename)
    
    if response:
        map_saver.get_logger().info(f'地图已保存至: {map_filename}')
    else:
        map_saver.get_logger().error('保存地图失败')
    
    map_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()