#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time
from collections import deque

class CallManagerPID(Node):
    def __init__(self):
        super().__init__('call_manager')

        # Subscriptions
        self.create_subscription(String, 'robot/call', self.on_call, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose_with_cov, 10)

        # BLE tracking
        self.latest_ble = {}          # { table_id: smoothed distance }
        self.ble_subscriptions = {}   # cached BLE subscriptions
        self.ble_buffers = {}         # { table_id: deque for smoothing }
        self.SMOOTH_SAMPLES = 3       # moving average window

        # Nav2 client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot pose
        self.current_pose = None  # unified PoseStamped-like

        # PID gains
        self.kp_linear = 0.6
        self.ki_linear = 0.0
        self.kd_linear = 0.08

        # BLE/PID safety
        self.pid_threshold = 0.18  # start PID if farther than this after Nav2

        self.get_logger().info('call_manager ready')

    # --- Pose callback ---
    def on_amcl_pose_with_cov(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.current_pose = ps
        self.get_logger().debug('AMCL pose received (converted)')

    # --- BLE subscription management ---
    def ensure_ble_subscription(self, table_id: str):
        topic = f'/ble_distance/{table_id}'
        if topic in self.ble_subscriptions:
            return
        self.get_logger().info(f'Creating BLE subscription for {topic}')
        sub = self.create_subscription(
            Float32,
            topic,
            lambda m, tid=table_id: self._on_ble(m, tid),
            10
        )
        self.ble_subscriptions[topic] = sub

    # --- BLE callback with smoothing ---
    def _on_ble(self, msg: Float32, table_id: str):
        dist = float(msg.data)
        if table_id not in self.ble_buffers:
            self.ble_buffers[table_id] = deque(maxlen=self.SMOOTH_SAMPLES)
        buf = self.ble_buffers[table_id]
        buf.append(dist)
        smoothed = sum(buf) / len(buf)
        self.latest_ble[table_id] = smoothed
        self.get_logger().debug(f'BLE update: {table_id} raw={dist:.3f}, smoothed={smoothed:.3f} m')

    # --- Handle a call ---
    def on_call(self, msg: String):
        table_id = msg.data.strip()
        self.get_logger().info(f'Received call for {table_id}')

        self.ensure_ble_subscription(table_id)

        if not self.wait_for_data_verbose(table_id, timeout=15.0):
            self.get_logger().warn('Required data not available, aborting call')
            return

        dist = self.latest_ble.get(table_id)
        pose = self.current_pose

        q = pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        rx = pose.pose.position.x
        ry = pose.pose.position.y

        # --- Adjust Nav2 goal (BLE calibrated: 0 = stop distance) ---
        d = min(5.0, dist)  # distance from BLE (already zero at stop)
        goal_x = rx + d * math.cos(yaw)
        goal_y = ry + d * math.sin(yaw)

        self.get_logger().info(f'Computed goal for {table_id}: x={goal_x:.3f}, y={goal_y:.3f} (d={d:.3f} m)')
        sent = self.send_nav_goal(goal_x, goal_y, yaw)
        if not sent:
            self.get_logger().warn('Nav2 goal failed/rejected')
            return

        rclpy.spin_once(self, timeout_sec=0.5)
        post_nav_dist = self.latest_ble.get(table_id, None)
        self.get_logger().info(f'BLE distance after Nav2: {post_nav_dist}')
        if post_nav_dist is None or post_nav_dist > self.pid_threshold:
            self.get_logger().info('Nav2 did not get close enough — starting PID fine approach.')
            self.pid_fine_stop(table_id)
        else:
            self.get_logger().info('Nav2 reached close enough to BLE target; skipping PID.')

    # --- Wait for pose + BLE data ---
    def wait_for_data_verbose(self, table_id: str, timeout: float = 15.0) -> bool:
        waited = 0.0
        interval = 0.5
        while rclpy.ok() and waited < timeout:
            pose_ready = self.current_pose is not None
            ble_ready = (table_id in self.latest_ble)
            self.get_logger().info(f'wait_for_data: pose_ready={pose_ready}, ble_ready={ble_ready}, waited={waited:.1f}s')
            if pose_ready and ble_ready:
                return True
            rclpy.spin_once(self, timeout_sec=interval)
            waited += interval
        return (self.current_pose is not None) and (table_id in self.latest_ble)

    # --- Send goal to Nav2 ---
    def send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('Nav2 action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw/2.0)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        self.get_logger().info('Nav2 action completed')
        return True

    # --- PID fine approach using BLE ---
    def pid_fine_stop(self, table_id: str):
        self.get_logger().info('Starting PID fine positioning (BLE-guided)...')
        dt = 0.1
        integral = 0.0
        prev_err = None
        start_time = time.time()
        max_time = 25.0

        while rclpy.ok():
            if (time.time() - start_time) > max_time:
                self.get_logger().warn('PID timeout, aborting')
                break

            if table_id not in self.latest_ble:
                rclpy.spin_once(self, timeout_sec=dt)
                continue

            distance = self.latest_ble[table_id]

            # BLE-calibrated stop: 0 = stop distance
            if distance <= 0.0:
                self.cmd_pub.publish(Twist())  # stop
                self.get_logger().info(f'Arrived at {table_id} (BLE dist {distance:.2f} m)')
                break

            # PID computation
            setpoint = 0.0  # calibrated BLE zero
            err = distance - setpoint
            if prev_err is None:
                prev_err = err
            integral += err * dt
            derivative = (err - prev_err) / dt
            prev_err = err

            lin = self.kp_linear * err + self.ki_linear * integral + self.kd_linear * derivative
            lin_cmd = max(min(lin, 0.25), -0.08)  # cap speed

            cmd = Twist()
            cmd.linear.x = lin_cmd
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=dt)

        self.cmd_pub.publish(Twist())
        self.get_logger().info('PID finished (stopped)')

def main(args=None):
    rclpy.init(args=args)
    node = CallManagerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
