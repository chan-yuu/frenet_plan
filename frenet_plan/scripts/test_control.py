#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray

class AdaptivePurePursuit(Node):
    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        #Params
        self.declare_parameter('lookahead_distance', 2.0)  # 预瞄距离 (米)
        self.declare_parameter('wheelbase', 1.5)           # 轴距 (米): 用于计算物理转向角
        self.declare_parameter('max_lat_accel', 1.5)       # 最大允许侧向加速度 (m/s^2), 决定舒适度
        self.declare_parameter('min_speed', 0.5)           # 最小过弯速度
        self.declare_parameter('goal_tolerance', 0.5)      # 到达终点的容差 (米)
        self.declare_parameter('control_rate', 20.0)       # 控制频率 (Hz)

        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.L = self.get_parameter('wheelbase').value
        self.max_lat_accel = self.get_parameter('max_lat_accel').value
        self.min_speed = self.get_parameter('min_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_rate = self.get_parameter('control_rate').value

        # Subscribers
        self.sub_path = self.create_subscription(Path, '/frenet_path', self.path_callback, 10)
        self.sub_vel = self.create_subscription(Float64MultiArray, '/frenet_vel', self.vel_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables
        self.path_msg = None
        self.vel_list = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.is_odom_received = False

        # Timer
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.get_logger().info("Adaptive Pure Pursuit Initialized")

    def path_callback(self, msg):
        self.path_msg = msg

    def vel_callback(self, msg):
        self.vel_list = msg.data

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Quaternion to Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.is_odom_received = True

    def find_target_index(self):
        """查找满足预瞄距离的目标点索引"""
        if self.path_msg is None or len(self.path_msg.poses) == 0:
            return None, None

        # 1. 找到离机器人最近的点
        min_dist = float('inf')
        nearest_idx = -1
        
        # 简单全局搜索 (路径点不多时性能可接受)
        for i, pose in enumerate(self.path_msg.poses):
            dx = pose.pose.position.x - self.robot_x
            dy = pose.pose.position.y - self.robot_y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # 2. 从最近点向后搜，找到第一个大于预瞄距离的点
        target_idx = nearest_idx
        for i in range(nearest_idx, len(self.path_msg.poses)):
            dx = self.path_msg.poses[i].pose.position.x - self.robot_x
            dy = self.path_msg.poses[i].pose.position.y - self.robot_y
            dist = math.hypot(dx, dy)
            
            if dist > self.lookahead_dist:
                target_idx = i
                break
        
        # 如果都在预瞄距离内，取最后一个点
        if target_idx == nearest_idx and nearest_idx == len(self.path_msg.poses) - 1:
            target_idx = len(self.path_msg.poses) - 1
            
        return target_idx, min_dist

    def calculate_curvature(self, target_x, target_y):
        """Pure Pursuit 核心：计算曲率"""
        # 1. 将目标点转换到机器人局部坐标系
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        
        # 旋转矩阵 (Global -> Body)
        # local_x 向前, local_y 向左
        local_x = dx * math.cos(self.robot_yaw) + dy * math.sin(self.robot_yaw)
        local_y = -dx * math.sin(self.robot_yaw) + dy * math.cos(self.robot_yaw)

        # 2. 计算预瞄距离 L
        L = math.hypot(local_x, local_y)
        
        if L < 0.01:
            return 0.0, 0.0

        # 3. 计算曲率 Gamma = 2 * y / L^2
        curvature = 2.0 * local_y / (L * L)
        
        return curvature, L

    def adaptive_velocity_control(self, target_v, curvature):
        """根据曲率限制速度 (舒适性控制)"""
        # 1. 估算需要的转向角 (Bicycle Model: tan(delta) = L * curvature)
        # 注意: 这里计算的是前轮转向角
        try:
            steering_angle = math.atan(self.L * curvature)
        except ValueError:
            return target_v

        if abs(steering_angle) < 0.01:
            return target_v

        # 2. 限制最大转向角用于计算 (防止 tan 90度爆炸)
        steer_abs = min(abs(steering_angle), 0.7) # 约 40度

        # 3. 计算物理极限速度
        # 曲率半径 R = 1 / curvature
        # 或者直接用公式: v < sqrt( a_max * R )
        # R = L / tan(delta)
        try:
            radius = self.L / math.tan(steer_abs)
            v_limit = math.sqrt(self.max_lat_accel * radius)
        except ZeroDivisionError:
            v_limit = target_v

        # 4. 取最小值
        final_v = min(target_v, v_limit)
        
        # 5. 保证最小速度 (防止因计算导致停车)
        final_v = max(final_v, self.min_speed)
        
        return final_v

    def control_loop(self):
        if not self.is_odom_received or self.path_msg is None:
            return

        cmd = Twist()

        # 1. 寻找目标点
        target_idx, _ = self.find_target_index()
        if target_idx is None:
            return

        target_pose = self.path_msg.poses[target_idx].pose.position
        
        # 2. 检查是否到达终点
        dist_to_goal = math.hypot(target_pose.x - self.robot_x, target_pose.y - self.robot_y)
        if target_idx == len(self.path_msg.poses) - 1 and dist_to_goal < self.goal_tolerance:
            self.get_logger().info("Goal Reached!", throttle_duration_sec=2.0)
            self.pub_cmd.publish(cmd) # Stop
            return

        # 3. 获取规划器给出的原始速度
        plan_v = 0.5
        if len(self.vel_list) > 0:
            v_idx = min(target_idx, len(self.vel_list) - 1)
            plan_v = self.vel_list[v_idx]

        # 4. 计算 Pure Pursuit 曲率
        curvature, lookahead_L = self.calculate_curvature(target_pose.x, target_pose.y)

        # 5. 【关键】应用大转向降速逻辑
        final_v = self.adaptive_velocity_control(plan_v, curvature)

        # 6. 计算最终角速度
        # omega = v * curvature
        omega = final_v * curvature

        # 7. 发布指令
        cmd.linear.x = float(final_v)
        cmd.angular.z = float(omega)

        self.pub_cmd.publish(cmd)
        
        # Debug Logging
        # self.get_logger().info(f"PlanV: {plan_v:.2f}, FinalV: {final_v:.2f}, Curv: {curvature:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist()) # 安全停车
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()