
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
 
class StampFixer(Node):
    def __init__(self):
        super().__init__('scan_stamp_fixer')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
    def cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = StampFixer()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
