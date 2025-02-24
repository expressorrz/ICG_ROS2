#!/usr/bin/env python3
# filepath: /home/ipu/Documents/robot_learning/ICG_ROS2/src/pose_subscriber.py

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        self.poses = []
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/pose',        # topic name
            self.pose_callback,
            10  # QoS
        )
        self.subscription
        
        self.get_logger().info('Pose Subscriber has been started')

    def pose_callback(self, msg):
        position = msg.pose.position
        x = position.x
        y = position.y
        z = position.z
        
        orientation = msg.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        self.get_logger().info(
            f'received object pose:\n'
            f'position: x={x:.3f}, y={y:.3f}, z={z:.3f}\n'
            f'orientation: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}'
        )

    

    def pose_callback(self, msg):
        position = msg.pose.position
        x = position.x
        y = position.y
        z = position.z
        
        orientation = msg.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        self.get_logger().info(
            f'received object pose:\n'
            f'position: x={x:.3f}, y={y:.3f}, z={z:.3f}\n'
            f'orientation: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}'
        )
        
        # Store the quaternion in the list
        self.poses.append([x, y, z, qx, qy, qz, qw])
        
        # Save the list to an npz file
        np.savez('/home/ipu/Documents/robot_learning/ICG_ROS2/src/pose_subscriber/pose_subscriber/poses.npz', poses=self.poses)

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    
    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        print('User interrupted with Ctrl-C')
    finally:
        pose_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()