#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math


class SimpleOdom(Node):
    def __init__(self):
        super().__init__('simple_odom')

        # Publisher Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Broadcaster TF
        self.br = TransformBroadcaster(self)

        # Subscriber to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Timer (20 Hz → période = 0.05s)
        self.timer = self.create_timer(0.05, self.update)

        # Position initiale
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Vitesse courante (mise à jour par /cmd_vel)
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Dernier temps
        self.last_time = self.get_clock().now()

    def cmd_callback(self, msg: Twist):
        """Mise à jour des vitesses depuis /cmd_vel"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def update(self):
        """Mise à jour de l’odométrie et publication TF + Odometry"""
        # Calcul du delta temps
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Calcul des incréments
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        # Mise à jour de la position
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Conversion en quaternion
        q = quaternion_from_yaw(self.th)

        # Création du message Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # ✅ Covariance basique (non nulle)
        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.1
        ]

        odom.twist.covariance = [
            0.1,  0,    0,    0,    0,    0,
            0,    0.1,  0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.2
        ]

        # Publication
        self.odom_pub.publish(odom)

        # Création et envoi du TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.br.sendTransform(t)

        # Mise à jour du temps précédent
        self.last_time = current_time


def quaternion_from_yaw(yaw):
    """Convertit un angle (yaw) en quaternion ROS2"""
    from tf_transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)

    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdom()
    rclpy.spin(node)

    # Nettoyage
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
