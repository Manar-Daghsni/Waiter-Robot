#!/usr/bin/env python3
# save_pose.py
import rclpy
from rclpy.node import Node
import yaml, os, sys
from geometry_msgs.msg import PoseWithCovarianceStamped

CONFIG = '/home/adminrobot/ros2_ws_lidar/src/config/saved_poses.yaml'

class Saver(Node):
    def __init__(self, map_key):
        super().__init__('save_pose_helper')
        self.map_key = map_key
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb, 1)
        self.get_logger().info('Waiting for /amcl_pose, publish a pose (move robot or set in RViz) then wait...')

    def cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # compute yaw from quaternion
        q = msg.pose.pose.orientation
        import math
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.get_logger().info(f'Saving pose for "{self.map_key}": x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
        data = {}
        if os.path.exists(CONFIG):
            with open(CONFIG, 'r') as f:
                data = yaml.safe_load(f) or {}
        data[self.map_key] = [float(x), float(y), float(yaw)]
        with open(CONFIG, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info('Saved. Exiting.')
        rclpy.shutdown()

def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: save_pose.py <map_key>')
        return
    map_key = sys.argv[1]
    rclpy.init()
    node = Saver(map_key)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
