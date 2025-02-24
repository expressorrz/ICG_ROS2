#!/usr/bin/env python3
# filepath: /home/ipu/Documents/robot_learning/ICG_ROS2/src/pose_subscriber.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_poses_eyeinhand',  # 话题名称，根据实际发布话题修改
            self.pose_callback,
            10  # QoS设置
        )
        self.subscription  # 防止未使用警告
        
        self.get_logger().info('姿态订阅节点已启动')

    def pose_callback(self, msg):
        # 获取位置信息
        position = msg.pose.position
        x = position.x
        y = position.y
        z = position.z
        
        # 获取方向信息（四元数）
        orientation = msg.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        # 打印接收到的信息
        self.get_logger().info(
            f'收到物体位姿:\n'
            f'位置: x={x:.3f}, y={y:.3f}, z={z:.3f}\n'
            f'方向: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    
    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        print('用户中断运行')
    finally:
        pose_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()