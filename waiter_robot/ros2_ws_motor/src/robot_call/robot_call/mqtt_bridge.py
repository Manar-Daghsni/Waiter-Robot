#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

MQTT_BROKER = "192.168.91.40"
MQTT_PORT = 1883
MQTT_TOPIC = 'robot/call'
ROS_TOPIC = 'robot/call'

class MQTTBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        self.pub = self.create_publisher(String, ROS_TOPIC, 10)
        self.get_logger().info('Starting MQTT -> ROS2 bridge')

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'Connected to MQTT broker (rc={rc}), subscribing to {MQTT_TOPIC}')
        client.subscribe(MQTT_TOPIC)

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8').strip()
        self.get_logger().info(f'MQTT received: {msg.topic} -> {payload}')
        ros_msg = String()
        ros_msg.data = payload
        self.pub.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
