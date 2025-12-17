#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class FrenetTestPublisher(Node):
    def __init__(self):
        super().__init__('frenet_test_publisher')
        
        # 发布参考路径
        self.path_pub = self.create_publisher(Path, '/local_path_plan', 10)
        
        # 发布参考速度
        self.vel_pub = self.create_publisher(Float64MultiArray, '/local_vel_cmd', 10)
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 发布costmap
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.publish_data)
        
        # 创建测试costmap（只在初始化时创建一次）
        self.costmap_msg = self.create_test_costmap()
        
        self.get_logger().info('Frenet Test Publisher Started')
    
    def publish_data(self):
        # 发布简单的直线参考路径
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # 生成50米直线路径
        for i in range(100):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(i) * 0.1
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        # 发布参考速度（每个点都是1.8m/s）
        vel_msg = Float64MultiArray()
        vel_msg.data = [2.0] * 100
        self.vel_pub.publish(vel_msg)
        
        # 发布里程计（车辆在起点）
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear.x = 1.8
        odom_msg.twist.twist.linear.y = 0.0
        
        self.odom_pub.publish(odom_msg)
        
        # 发布costmap
        self.costmap_msg.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap_msg)
    
    def create_test_costmap(self):
        """创建一个带有测试障碍物的局部代价地图"""
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'map'
        
        # 地图参数：20m x 10m，分辨率0.1m
        width = 200   # 20m / 0.1m
        height = 100  # 10m / 0.1m
        resolution = 0.1
        
        costmap.info.resolution = resolution
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin.position.x = 0.0
        costmap.info.origin.position.y = -5.0  # 中心对齐
        costmap.info.origin.position.z = 0.0
        costmap.info.origin.orientation.w = 1.0
        
        # 初始化为自由空间（代价值0）
        data = np.zeros((height, width), dtype=np.int8)
        
        # 添加一些测试障碍物
        # 障碍物1：在x=8m, y=0.5m处的静态障碍
        obs1_x = int(8.0 / resolution)
        obs1_y = int((0.5 + 5.0) / resolution)
        for i in range(-5, 6):
            for j in range(-5, 6):
                if 0 <= obs1_y + i < height and 0 <= obs1_x + j < width:
                    data[obs1_y + i, obs1_x + j] = 100
        
        # 障碍物2：在x=12m, y=-0.8m处的静态障碍
        obs2_x = int(12.0 / resolution)
        obs2_y = int((-0.8 + 5.0) / resolution)
        for i in range(-5, 6):
            for j in range(-5, 6):
                if 0 <= obs2_y + i < height and 0 <= obs2_x + j < width:
                    data[obs2_y + i, obs2_x + j] = 100
        
        # 障碍物3：路边界（左右边界稍微加一些代价）
        for x in range(width):
            # 左边界 (y = -1.8m)
            y_left = int((-1.8 + 5.0) / resolution)
            if 0 <= y_left < height:
                data[y_left, x] = 80
            # 右边界 (y = 1.8m)
            y_right = int((1.8 + 5.0) / resolution)
            if 0 <= y_right < height:
                data[y_right, x] = 80
        
        # 转换为一维数组
        costmap.data = data.flatten().tolist()
        
        return costmap

def main(args=None):
    rclpy.init(args=args)
    node = FrenetTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
