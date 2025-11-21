#!/usr/bin/env python3
# amcl_initializer.py (fixed: cancel timer instead of oneshot kwarg)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import yaml
import os
import math
import time
import sys

# Adjust this path if your saved_poses.yaml lives elsewhere
CONFIG = '/home/adminrobot/ros2_ws_lidar/src/config/saved_poses.yaml'

class AmclInitializer(Node):
    def __init__(self, map_key):
        super().__init__('amcl_initializer')
        self.map_key = map_key
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info(f'AmclInitializer started (map_key="{self.map_key}")')

        # Create a timer that fires once after ~2s by cancelling it inside the callback.
        self.timer = self.create_timer(2.0, self._start)

    def _start(self):
        """Run once after a short delay: publish saved pose or call global localization."""
        # cancel timer immediately so _start won't run again
        try:
            self.timer.cancel()
        except Exception:
            pass

        try:
            pose = self._load_saved_pose()
            if pose:
                self.get_logger().info('Found saved pose — publishing /initialpose 3x')
                # publish a few times so AMCL has a good chance to receive it
                for i in range(3):
                    self._publish_pose(pose)
                    time.sleep(1.0)
                self.get_logger().info('Published saved initial pose; exiting initializer.')
            else:
                self.get_logger().info('No saved pose found; calling /reinitialize_global_localization')
                client = self.create_client(Empty, '/reinitialize_global_localization')
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warning('/reinitialize_global_localization service not available')
                else:
                    client.call_async(Empty.Request())
                    self.get_logger().info('Called global reinitialize service.')
        except Exception as e:
            self.get_logger().error(f'Initializer error: {e}')
        finally:
            # stop rclpy event loop so main() can finish and the process exits
            rclpy.shutdown()

    def _load_saved_pose(self):
        """Return dict {'x','y','yaw'} or None."""
        if not os.path.exists(CONFIG):
            return None
        with open(CONFIG, 'r') as f:
            data = yaml.safe_load(f) or {}
        if self.map_key in data:
            v = data[self.map_key]
            if isinstance(v, dict):
                return {'x': float(v.get('x', 0.0)), 'y': float(v.get('y', 0.0)), 'yaw': float(v.get('yaw', 0.0))}
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                return {'x': float(v[0]), 'y': float(v[1]), 'yaw': float(v[2])}
        return None

    def _publish_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = p['x']
        msg.pose.pose.position.y = p['y']
        yaw = p['yaw']
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.05   # var x
        cov[7] = 0.05   # var y
        cov[35] = 0.02  # var yaw
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.get_logger().info(f'Published initialpose: x={p["x"]:.3f} y={p["y"]:.3f} yaw={p["yaw"]:.3f}')

def main():
    map_key = ''
    if len(sys.argv) >= 2:
        map_key = sys.argv[1]
    else:
        map_key = os.getenv('MAP_KEY', '')
    if not map_key:
        print('Usage: amcl_initializer.py <map_key>  or set MAP_KEY env')
        return

    rclpy.init()
    node = AmclInitializer(map_key)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # try to clean up node if not already destroyed
    try:
        node.destroy_node()
    except Exception:
        pass

if __name__ == '__main__':
    main()
