#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
from std_msgs.msg import Header

class UltrasonicToPointCloud(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_pointcloud')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ultrasonic_data',
            self.ultrasonic_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, 'ultrasonic_pointcloud', 10)
        self.sensor_positions = [
            [0.15, 0.0, 0.05],  # Front
            [0.0, 0.1, 0.05],   # Left
            [0.0, -0.1, 0.05],  # Right
            [-0.15, 0.0, 0.05]  # Back
        ]
        self.sensor_orientations = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.5708],
            [0.0, 0.0, -1.5708],
            [0.0, 0.0, 3.1416]
        ]
        self.get_logger().info('Ultrasonic to PointCloud node started')

    def ultrasonic_callback(self, msg):
        self.get_logger().info(f'Received ultrasonic data: {msg.data}')
        if len(msg.data) != 4:
            self.get_logger().warn('Invalid ultrasonic data length')
            return
        points = []
        for i, distance_cm in enumerate(msg.data):
            # Convert cm to meters
            distance_m = distance_cm / 100.0
            if 0.0 <= distance_m <= 5.0:  # Range in meters (0-500 cm)
                yaw = self.sensor_orientations[i][2]
                x_base, y_base, z_base = self.sensor_positions[i]
                x = distance_m
                y = 0.0
                z = 0.0
                x_base_adj = x * np.cos(yaw) - y * np.sin(yaw) + x_base
                y_base_adj = x * np.sin(yaw) + y * np.cos(yaw) + y_base
                points.append([x_base_adj, y_base_adj, z_base])
            else:
                self.get_logger().warn(f'Invalid distance from sensor {i}: {distance_cm} cm (out of 0-500 cm range)')
        if points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            pointcloud = point_cloud2.create_cloud(header, fields, points)
            self.publisher.publish(pointcloud)
            self.get_logger().info(f'Published {len(points)} points to /ultrasonic_pointcloud')
        else:
            self.get_logger().warn('No valid points to publish')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
