#!/usr/bin/env python3
# filepath: /home/ipu/Documents/robot_learning/ICG_ROS2/src/pose_subscriber.py

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseArray, Pose

class Msg_Subscriber(Node):
    def __init__(self, task_name='pick_aruco'):
        super().__init__('twist_subscriber')

        self.task_name = task_name

        if self.task_name == 'pick_aruco' or self.task_name == 'pick_icg':
            self.marker_list = [0]
            topic_name = '/aruco_markers_eyeinhand'
        elif self.task_name == 'place_aruco' or self.task_name == 'place_icg':
            self.marker_list = [2, 3]
            topic_name = '/aruco_markers_eyetohand'

        self.data_dict = {
            'twists': [],
            'poses': {}
        }
        
        for marker_id in self.marker_list:
            self.data_dict['poses'][f'pose_{marker_id}'] = []
        
        self.twist_subscription = self.create_subscription(
            TwistStamped,
            '/ips_abbirb1600_onrobot2fg7/robot/servo_node_abbirb1600/delta_twist_cmds',        # topic name
            self.twist_callback,
            10  # QoS
        )
        
        self.pose_subscription = self.create_subscription(
            ArucoMarkers,
            topic_name,
            self.pose_callback,
            10  # QoS
        )
    

    def twist_callback(self, msg):
        # Get linear and angular velocities from the twist message
        linear = msg.twist.linear
        angular = msg.twist.angular
        
        # Extract the components
        timestamp_sec = msg.header.stamp.sec
        timestamp_nanosec = msg.header.stamp.nanosec

        x = linear.x
        y = linear.y
        z = linear.z
        
        qx = angular.x
        qy = angular.y
        qz = angular.z
        
        self.get_logger().info(
            f'received twist:\n'
            f'linear: x={x:.3f}, y={y:.3f}, z={z:.3f}\n'
            f'angular: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}'
        )
        
        # Store the twist in the list
        self.data_dict['twists'].append([timestamp_sec, timestamp_nanosec, x, y, z, qx, qy, qz])
        

    def pose_callback(self, msg):
        

        for marker_id in self.marker_list:
            if marker_id not in msg.marker_ids:
                continue
            else:
                # Get the index from marker_list and the pose from the message
                idx = msg.marker_ids.index(marker_id)
                self.get_logger().info(f'idx: {idx}, marker_ids: {msg.marker_ids}, marker_id: {marker_id}')

                if msg.poses[idx].position.x != 0.0:
                    

                    timestamp_sec = msg.header.stamp.sec
                    timestamp_nanosec = msg.header.stamp.nanosec
                    position = msg.poses[idx].position
                    orientation = msg.poses[idx].orientation
            
                    x = position.x
                    y = position.y
                    z = position.z
                    qx = orientation.x
                    qy = orientation.y
                    qz = orientation.z
                    qw = orientation.w
            
                    self.get_logger().info(
                        f'received object pose:\n'
                        f'position: x={x:.3f}, y={y:.3f}, z={z:.3f}\n'
                        f'orientation: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}'
                    )

                    # Store the pose in the list
                    self.data_dict['poses'][f'pose_{marker_id}'].append([timestamp_sec, timestamp_nanosec, x, y, z, qx, qy, qz, qw])
                else:
                    self.data_dict['poses'][f'pose_{marker_id}'].append([0, 0, 0, 0, 0, 0, 0, 0, 0])
            
  
        
        # Save the list to an npz file
        np.savez(f'/home/qiangubuntu/research/ips_dtir/src/temp/pick_and_place/icg_ros2/pose_subscriber/pose_subscriber/data_{self.task_name}.npz', **self.data_dict)

def main(args=None):
    # pick_aruco / place_aruco / pick_icg / place_icg
    task_name = 'place_aruco'
    
    rclpy.init(args=args)
    msg_subscriber = Msg_Subscriber(task_name)

    
    try:
        rclpy.spin(msg_subscriber)
    except KeyboardInterrupt:
        print('User interrupted with Ctrl-C')
    finally:
        msg_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()