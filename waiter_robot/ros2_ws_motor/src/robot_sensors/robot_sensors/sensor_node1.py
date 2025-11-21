#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Int32MultiArray

import serial, math, time, threading, traceback
import tf_transformations
import tf2_ros

# Optional: very simple 1D Kalman filter for theta (keeps noise low on yaw)
class SimpleKalman1D:
    def __init__(self, q=0.001, r=0.01, x0=0.0, p0=1.0):
        self.q = q  # process variance
        self.r = r  # measurement variance
        self.x = x0
        self.p = p0

    def update(self, measurement):
        # Prediction
        self.p += self.q
        # Update
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node1')

        # ==========================
        # Parameters (use .value for clarity)
        # ==========================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('wheel_radius', 0.0248)     # m
        self.declare_parameter('wheel_base', 0.35)         # m (distance L)
        self.declare_parameter('ticks_per_rev', 4)         # encoder pulses per wheel rev
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('ignore_O_for_raw_odom', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ignore_O_for_raw_odom = bool(self.get_parameter('ignore_O_for_raw_odom').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # ==========================
        # Publishers
        # ==========================
        qos_imu = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_odom = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_enc = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)

        self.imu_pub = self.create_publisher(Imu, '/imu', qos_imu)
        self.ultra_pub = self.create_publisher(Range, '/ultrasonic', qos_enc)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', qos_enc)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_odom)
        self.arduino_odom_pub = self.create_publisher(Odometry, '/arduino_odom', qos_odom)

        # ==========================
        # Subscribers
        # ==========================
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ==========================
        # Odom fusion state
        # ==========================
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.fused_theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_left = 0.0
        self.last_right = 0.0

        # Kalman filter for theta (optional smoothing)
        self.kalman_theta = SimpleKalman1D(q=0.001, r=0.05, x0=0.0)

        # Store last IMU yaw and gz for fusion
        self.last_imu_yaw = 0.0
        self.last_imu_gz = 0.0

        # TF broadcaster
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        # complementary fusion weight for encoder vs imu heading (0..1)
        self.ENCODER_WEIGHT = 0.7

        # ==========================
        # Serial management
        # ==========================
        self.ser = None
        self._serial_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader_thread = None

        self._open_serial()
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()

        # Health log
        self.create_timer(5.0, self._health_log)

        self.get_logger().info(
            f"SensorNode1 initialized (port={self.port} baud={self.baud} tpr={self.ticks_per_rev})"
        )

    # ==================================================
    # Serial helpers
    # ==================================================
    def _try_int(self, s):
        try:
            return int(float(s))
        except Exception:
            return None

    def _try_float(self, s):
        try:
            return float(s)
        except Exception:
            return None

    def _open_serial(self):
        try:
            with self._serial_lock:
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(1.2)
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
            self.get_logger().info(f"Serial {self.port} @ {self.baud} opened")
        except Exception as e:
            self.get_logger().error(f"Failed opening serial {self.port}: {e}")
            self.ser = None

    def _reopen_serial(self):
        try:
            with self._serial_lock:
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
            time.sleep(1.0)
            self._open_serial()
        except Exception as e:
            self.get_logger().error(f"_reopen_serial error: {e}")

    # ==================================================
    # Serial reader loop
    # ==================================================
    def _serial_reader_loop(self):
        while not self._stop_reader.is_set():
            try:
                if self.ser is None:
                    time.sleep(1.0)
                    self._open_serial()
                    continue

                try:
                    raw = self.ser.readline()
                except Exception as e:
                    self.get_logger().warning(f"Serial read exception: {e}")
                    self._reopen_serial()
                    continue

                if not raw:
                    continue

                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = str(raw)

                if not line:
                    continue

                try:
                    self._handle_line(line)
                except Exception as e:
                    self.get_logger().warning(
                        f"Error processing serial line: {e}\n{traceback.format_exc()}"
                    )

            except Exception as e:
                self.get_logger().error(
                    f"Reader loop unexpected error: {e}\n{traceback.format_exc()}"
                )
                time.sleep(1.0)

    def shutdown(self):
        self._stop_reader.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        with self._serial_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    # ==================================================
    # Serial line handler
    # ==================================================
    def _handle_line(self, line: str):
        parts = [p.strip() for p in line.split(',') if p is not None]
        if not parts:
            return

        tag = parts[0]
        now = self.get_clock().now()

        if tag == 'O':
            self._parse_O(parts, now)
        elif tag == 'E':
            self._parse_E(parts, now)
        elif tag == 'I':
            self._parse_I(parts, now)
        elif tag == 'U':
            self._parse_U(parts, now)
        elif tag == 'CMD':
            if len(parts) > 1:
                self.get_logger().info(f"Arduino executed command: {parts[1]}")
            else:
                self.get_logger().info("Arduino executed command (no detail)")
        else:
            self.get_logger().debug(f"Unknown serial line (ignored): {line}")

    # ==================================================
    # Cmd_vel → Arduino
    # ==================================================
    def cmd_vel_callback(self, msg: Twist):
        lin, ang = msg.linear.x, msg.angular.z
        if abs(lin) < 0.05 and abs(ang) < 0.05:
            cmd = "s\n"
        elif lin > 0.05:
            cmd = "f\n"
        elif lin < -0.05:
            cmd = "b\n"
        elif ang > 0.05:
            cmd = "l\n"
        elif ang < -0.05:
            cmd = "r\n"
        else:
            cmd = "s\n"

        try:
            with self._serial_lock:
                if self.ser is None:
                    self.get_logger().warn("Serial not open, cannot send cmd")
                    return
                self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"Failed to send cmd: {e}")

    # ==================================================
    # Encoder parser → compute odometry + twist
    # ==================================================
    def _parse_E(self, parts, now):
        # Expect: E,fl,fr,rl,rr
        if len(parts) < 5:
            self.get_logger().warning("E message too short")
            return

        fl = self._try_int(parts[1]); fr = self._try_int(parts[2])
        rl = self._try_int(parts[3]); rr = self._try_int(parts[4])
        if None in (fl, fr, rl, rr):
            self.get_logger().warning(f"Invalid encoder ints: {parts[1:5]}")
            return

        # Publish encoder ticks (raw)
        enc = Int32MultiArray(); enc.data = [fl, fr, rl, rr]
        self.encoder_pub.publish(enc)

        # Time delta
        now_time = now
        dt = (now_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.001

        # Average left/right ticks (cumulative counts from Arduino)
        left_ticks = (fl + rl) / 2.0
        right_ticks = (fr + rr) / 2.0

        # Convert ticks to meters (delta)
        d_left = (2.0 * math.pi * self.wheel_radius) * (left_ticks - self.last_left) / float(self.ticks_per_rev)
        d_right = (2.0 * math.pi * self.wheel_radius) * (right_ticks - self.last_right) / float(self.ticks_per_rev)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / float(self.wheel_base)

        # ---------------------------
        # Fusion: integrate encoder displacement into fused pose
        # ---------------------------
        # Use the small-angle mid-point integration for x,y
        self.fused_x += d_center * math.cos(self.fused_theta + d_theta / 2.0)
        self.fused_y += d_center * math.sin(self.fused_theta + d_theta / 2.0)

        # Fuse heading: use complementary fusion of encoder delta and IMU yaw if available
        # encoder estimate:
        encoder_theta = self.fused_theta + d_theta
        # if we have an IMU yaw measured, do complementary fusion between encoder estimate and IMU yaw
        if hasattr(self, 'last_imu_yaw'):
            fused_theta = (self.ENCODER_WEIGHT * encoder_theta +
                           (1.0 - self.ENCODER_WEIGHT) * self.last_imu_yaw)
        else:
            fused_theta = encoder_theta

        # Optional smoothing / filtering on theta
        fused_theta = self.kalman_theta.update(fused_theta)

        self.fused_theta = fused_theta

        # Compute twist (linear + angular): linear velocity forward and angular velocity
        vx = d_center / dt
        vth_encoder = d_theta / dt

        # Fuse angular rate with IMU gz if available (simple average)
        if hasattr(self, 'last_imu_gz'):
            vth = 0.5 * (vth_encoder + self.last_imu_gz)
        else:
            vth = vth_encoder

        # Build Odometry message (fused)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.fused_x
        odom.pose.pose.position.y = self.fused_y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.fused_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set sensible pose covariance (6x6 flattened). Tweak these values for your system.
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02   # var(x)  (m^2)
        pose_cov[7] = 0.02   # var(y)
        pose_cov[14] = 1e6   # var(z) (unused / unknown)
        pose_cov[21] = 1e6   # var(rot_x)
        pose_cov[28] = 1e6   # var(rot_y)
        pose_cov[35] = (0.05 * 0.05)  # var(yaw) ~ (0.05 rad)^2
        odom.pose.covariance = pose_cov

        # twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vth

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.1    # var vx
        twist_cov[7] = 0.1    # var vy
        twist_cov[35] = 0.02  # var vtheta
        odom.twist.covariance = twist_cov

        # Publish fused odom for SLAM and navigation
        self.odom_pub.publish(odom)

        # Also publish raw Arduino odom if user requests it (useful for debugging)
        raw_odom = Odometry()
        raw_odom.header.stamp = now.to_msg()
        raw_odom.header.frame_id = self.odom_frame
        raw_odom.child_frame_id = self.base_frame
        # Build raw pose from encoder-only estimate (no IMU fusion)
        # Use the encoder-only incremental pose (we computed fused_x,y earlier from enc)
        raw_odom.pose = odom.pose
        raw_odom.twist = odom.twist
        # You could mark covariances higher for raw_odom if desired
        self.arduino_odom_pub.publish(raw_odom)

        # Broadcast TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.fused_x
        t.transform.translation.y = self.fused_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(t)

        # Update last ticks and time
        self.last_left = left_ticks
        self.last_right = right_ticks
        self.last_time = now_time

    # ==================================================
    # IMU parser
    # ==================================================
    def _parse_I(self, parts, now):
        # Expect: I,ax,ay,az,gx,gy,gz,qx,qy,qz,qw
        if len(parts) < 11:
            self.get_logger().warning("I message too short")
            return

        ax, ay, az = self._try_float(parts[1]), self._try_float(parts[2]), self._try_float(parts[3])
        gx, gy, gz = self._try_float(parts[4]), self._try_float(parts[5]), self._try_float(parts[6])
        qx, qy, qz, qw = (self._try_float(parts[7]), self._try_float(parts[8]),
                          self._try_float(parts[9]), self._try_float(parts[10]))
        if None in (ax, ay, az, gx, gy, gz, qx, qy, qz, qw):
            self.get_logger().warning(f"Invalid IMU floats: {parts[1:11]}")
            return

        # Store angular z for odom fusion (gz is angular velocity about z in rad/s)
        self.last_imu_gz = gz

        # compute yaw from quaternion
        try:
            yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])[2]
            self.last_imu_yaw = yaw
        except Exception:
            # fallback if euler extraction fails
            self.last_imu_yaw = self.fused_theta

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_pub.publish(imu)

    # ==================================================
    # Ultrasonic parser
    # ==================================================
    def _parse_U(self, parts, now):
        if len(parts) < 2:
            return
        try:
            r = float(parts[1])
            msg = Range()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'ultrasonic_link'
            msg.range = r
            self.ultra_pub.publish(msg)
        except Exception:
            pass

    # ==================================================
    # Arduino O message parser (optional)
    # O,x,y,theta,vx,vy,vtheta
    # If ignore_O_for_raw_odom==True we skip processing O (we rely on E + I fusion).
    # ==================================================
    def _parse_O(self, parts, now):
        # If user asked to ignore Arduino O lines then return
        # Note: previously logic was inverted; fixed here.
        if self.ignore_O_for_raw_odom:
            return

        if len(parts) < 7:
            self.get_logger().warning("O message too short")
            return

        try:
            sx, sy, stheta, svx, svy, svtheta = parts[1:7]
            x = float(sx); y = float(sy); theta = float(stheta)
            vx = float(svx); vy = float(svy); vth = float(svtheta)
        except Exception:
            self.get_logger().warning(f"Malformed O message: {parts}")
            return

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # sensible covariance for Arduino-provided odom (tweak as needed)
        oc = [0.0] * 36
        oc[0] = 0.05; oc[7] = 0.05; oc[35] = 0.05*0.05
        odom.pose.covariance = oc
        odom.twist.covariance = oc

        self.arduino_odom_pub.publish(odom)

    # ==================================================
    # Health log
    # ==================================================
    def _health_log(self):
        self.get_logger().debug(
            f"health: fused_x={self.fused_x:.3f} fused_y={self.fused_y:.3f} fused_theta={self.fused_theta:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    # MultiThreadedExecutor is OK, but ensure it's used properly:
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

