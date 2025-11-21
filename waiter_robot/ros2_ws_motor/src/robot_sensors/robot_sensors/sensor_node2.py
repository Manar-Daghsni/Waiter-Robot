#!/usr/bin/env python3
"""
SensorNode with Time-Synchronized Transforms:
- Subscribes to /scan to get the most recent laser scan timestamp
- Uses this timestamp for publishing odometry and transforms
- EKF (x,y,theta) fusing mecanum wheel odometry + IMU yaw
- publishes fused /odom (RELIABLE QoS) and raw /arduino_odom
- publishes /wheel_vels (Float32MultiArray) = [vFL, vFR, vRL, vRR] (m/s)
- fills sensible odom pose & twist covariances
- broadcasts TF (odom -> base_frame) with same timestamp as scan header
- provides /pure_pursuit/enabled publisher and subscribes to /pure_pursuit/path (Path)
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, Range, LaserScan
from std_msgs.msg import Int32MultiArray, Bool, Float32MultiArray

import serial, math, time, threading, traceback
import tf_transformations
import tf2_ros

# ----------------- small EKF helpers (3-state) -----------------
def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

class EKFOdometry:
    def __init__(self, Q_diag=(0.02, 0.02, 0.01), R_theta=0.05*0.05):
        # state x = [x, y, theta]
        self.x = [0.0, 0.0, 0.0]
        # covariance P 3x3
        self.P = [[1e-3 if i==j else 0.0 for j in range(3)] for i in range(3)]
        self.Q_diag = Q_diag
        self.R_theta = R_theta

    def predict(self, Vx, Vy, omega, dt):
        theta = self.x[2]
        ct = math.cos(theta); st = math.sin(theta)
        dx = Vx * dt; dy = Vy * dt; dth = omega * dt
        # integrate (robot-frame velocities -> world-frame)
        self.x[0] += dx * ct - dy * st
        self.x[1] += dx * st + dy * ct
        self.x[2] = wrap_angle(self.x[2] + dth)

        # Jacobian F
        F = [[1.0, 0.0, -dt * (Vx * math.sin(theta) + Vy * math.cos(theta))],
             [0.0, 1.0,  dt * (Vx * math.cos(theta) - Vy * math.sin(theta))],
             [0.0, 0.0, 1.0]]
        # Q
        Q = [[self.Q_diag[0]*dt, 0.0, 0.0],
             [0.0, self.Q_diag[1]*dt, 0.0],
             [0.0, 0.0, self.Q_diag[2]*dt]]
        # P = F P F^T + Q (compute manually)
        # temp = F * P
        temp = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    temp[i][j] += F[i][k] * self.P[k][j]
        # Pnew = temp * F^T + Q
        Pnew = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    Pnew[i][j] += temp[i][k] * F[j][k]  # F^T index swap
                Pnew[i][j] += Q[i][j]
        self.P = Pnew

    def update_theta(self, theta_meas):
        # Full Kalman update for measurement of yaw (H = [0,0,1])
        # R: scalar measurement variance for yaw
        R = float(self.R_theta)
        # S = H P H^T + R  -> scalar
        S = self.P[2][2] + R
        if S <= 1e-12:
            return
        # innovation (angle diff)
        y = wrap_angle(theta_meas - self.x[2])

        # K = P H^T S^{-1}  -> 3x1 vector
        K = [self.P[0][2] / S, self.P[1][2] / S, self.P[2][2] / S]

        # State update
        for i in range(3):
            self.x[i] += K[i] * y
        self.x[2] = wrap_angle(self.x[2])

        # Compute (I - K H) as a 3x3 matrix
        I_minus_KH = [[1.0, 0.0, -K[0]],
                      [0.0, 1.0, -K[1]],
                      [0.0, 0.0,  1.0 - K[2]]]

        # First compute A = (I-KH) * P
        A = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    A[i][j] += I_minus_KH[i][k] * self.P[k][j]

        # Then Pnew = A * (I-KH)^T + K * R * K^T
        Pnew = [[0.0]*3 for _ in range(3)]
        # A * (I-KH)^T
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    Pnew[i][j] += A[i][k] * I_minus_KH[j][k]
        # + K R K^T (outer product scaled by R)
        for i in range(3):
            for j in range(3):
                Pnew[i][j] += K[i] * R * K[j]

        self.P = Pnew


# ----------------- SensorNode -----------------
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node_pp')

        # parameters (frames included)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('wheel_radius', 0.0248)  # m
        self.declare_parameter('wheel_track', 0.35)     # width between left/right (lx)
        self.declare_parameter('wheel_length', 0.28)    # length between front/back (ly)
        self.declare_parameter('ticks_per_rev', 4)
        self.declare_parameter('ignore_O_for_raw_odom', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.wheel_length = float(self.get_parameter('wheel_length').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.ignore_O_for_raw_odom = bool(self.get_parameter('ignore_O_for_raw_odom').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # QoS
        qos_imu = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_odom = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_odom.reliability = ReliabilityPolicy.RELIABLE
        qos_enc = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)

        # publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_imu)
        self.ultra_pub = self.create_publisher(Range, '/ultrasonic', qos_enc)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', qos_enc)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_odom)  # fused EKF odom
        self.arduino_odom_pub = self.create_publisher(Odometry, '/arduino_odom', qos_odom)  # raw
        self.wheel_vels_pub = self.create_publisher(Float32MultiArray, '/wheel_vels', qos_enc)
        
        # pure pursuit control interface
        self.pp_enabled_pub = self.create_publisher(Bool, '/pure_pursuit/enabled', 10)
        self.create_subscription(Path, '/pure_pursuit/path', self._path_callback, 10)

        # subscriber for cmd_vel (still forward to arduino)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # subscriber for scan to get timestamps for time synchronization
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # EKF and state
        self.ekf = EKFOdometry(Q_diag=(0.02,0.02,0.01), R_theta=(0.05*0.05))
        self.last_fl = 0; self.last_fr = 0; self.last_rl = 0; self.last_rr = 0
        self.last_time = self.get_clock().now()
        self.last_imu_gz = 0.0
        self.last_imu_yaw = 0.0
        
        # For time synchronization - store the latest scan timestamp
        self.latest_scan_stamp = None
        self.scan_stamp_lock = threading.Lock()

        # serial management
        self.ser = None
        self._serial_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._reader_thread = None

        # TF broadcaster and path holder
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.current_path = None

        self._open_serial()
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()
        self.create_timer(5.0, self._health_log)
        self.get_logger().info(f"SensorNode (pure pursuit ready) on {self.port}@{self.baud}")

    # ---------- scan callback for time synchronization ----------
    def scan_callback(self, msg):
        # Store the latest scan timestamp for time synchronization
        with self.scan_stamp_lock:
            self.latest_scan_stamp = msg.header.stamp
        self.get_logger().debug(f"Received scan with stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    # ---------- serial helpers ----------
    def _try_int(self, s):
        try: return int(float(s))
        except Exception: return None
    def _try_float(self, s):
        try: return float(s)
        except Exception: return None

    def _open_serial(self):
        try:
            with self._serial_lock:
                if self.ser and self.ser.is_open:
                    try: self.ser.close()
                    except Exception: pass
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(1.0)
                try: self.ser.reset_input_buffer()
                except Exception: pass
            self.get_logger().info(f"Serial opened {self.port}@{self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed opening serial: {e}")
            self.ser = None

    def _reopen_serial(self):
        try:
            with self._serial_lock:
                if self.ser:
                    try: self.ser.close()
                    except Exception: pass
                self.ser = None
            time.sleep(1.0)
            self._open_serial()
        except Exception as e:
            self.get_logger().error(f"_reopen_serial: {e}")

    # ---------- serial reader ----------
    def _serial_reader_loop(self):
        while not self._stop_reader.is_set():
            try:
                if self.ser is None:
                    time.sleep(1.0); self._open_serial(); continue
                try:
                    raw = self.ser.readline()
                except Exception as e:
                    self.get_logger().warning(f"Serial read exception: {e}")
                    self._reopen_serial(); continue
                if not raw: continue
                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = str(raw)
                if not line: continue
                try:
                    self._handle_line(line)
                except Exception as e:
                    self.get_logger().warning(f"Error processing line: {e}\n{traceback.format_exc()}")
            except Exception as e:
                self.get_logger().error(f"Reader loop unexpected: {e}\n{traceback.format_exc()}")
                time.sleep(1.0)

    def shutdown(self):
        self._stop_reader.set()
        if self._reader_thread: self._reader_thread.join(timeout=1.0)
        with self._serial_lock:
            if self.ser:
                try: self.ser.close()
                except Exception: pass
            self.ser = None

    # ---------- serial handlers ----------
    def _handle_line(self, line: str):
        parts = [p.strip() for p in line.split(',') if p is not None]
        if not parts: return
        tag = parts[0]
        now = self.get_clock().now()
        if tag == 'E':   # encoder: E,fl,fr,rl,rr
            self._parse_E(parts, now)
        elif tag == 'I': # imu
            self._parse_I(parts, now)
        elif tag == 'O': # optional arduino fused odom
            if not self.ignore_O_for_raw_odom:
                self._parse_O(parts, now)
        elif tag == 'U':
            self._parse_U(parts, now)
        elif tag == 'CMD':
            self.get_logger().info(f"Arduino CMD: {parts[1] if len(parts)>1 else ''}")
        else:
            self.get_logger().debug(f"Unknown serial: {line}")

    # ---------- cmd_vel -> arduino ----------
    def cmd_vel_callback(self, msg: Twist):
        lin, ang = msg.linear.x, msg.angular.z
        if abs(lin) < 0.05 and abs(ang) < 0.05: cmd = "s\n"
        elif lin > 0.05: cmd = "f\n"
        elif lin < -0.05: cmd = "b\n"
        elif ang > 0.05: cmd = "l\n"
        elif ang < -0.05: cmd = "r\n"
        else: cmd = "s\n"
        try:
            with self._serial_lock:
                if self.ser is None: self.get_logger().warn("Serial not open"); return
                self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"Failed send cmd: {e}")

    # ---------- encoder parsing (E) ----------
    def _parse_E(self, parts, now):
        if len(parts) < 5:
            self.get_logger().warning("E message too short"); return
        fl = self._try_int(parts[1]); fr = self._try_int(parts[2])
        rl = self._try_int(parts[3]); rr = self._try_int(parts[4])
        if None in (fl,fr,rl,rr):
            self.get_logger().warning("Invalid encoder ints"); return

        # publish raw ticks
        enc = Int32MultiArray(); enc.data = [fl,fr,rl,rr]; self.encoder_pub.publish(enc)

        # dt
        now_time = now
        dt = (now_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 0.001

        # compute deltas (Arduino sends cumulative ticks)
        dfl = fl - self.last_fl; dfr = fr - self.last_fr; drl = rl - self.last_rl; drr = rr - self.last_rr
        self.last_fl, self.last_fr, self.last_rl, self.last_rr = fl, fr, rl, rr

        # per-wheel linear speeds (m/s)
        circumference = 2.0 * math.pi * self.wheel_radius
        vFL = (dfl / float(self.ticks_per_rev)) * circumference / dt
        vFR = (dfr / float(self.ticks_per_rev)) * circumference / dt
        vRL = (drl / float(self.ticks_per_rev)) * circumference / dt
        vRR = (drr / float(self.ticks_per_rev)) * circumference / dt

        # publish wheel_vels topic (Float32MultiArray)
        fv = Float32MultiArray(); fv.data = [vFL, vFR, vRL, vRR]; self.wheel_vels_pub.publish(fv)

        # inverse kinematics => robot-frame Vx,Vy,omega
        Vx = (vFL + vFR + vRL + vRR) * 0.25
        Vy = (-vFL + vFR + vRL - vRR) * 0.25
        r_turn = 0.5 * (self.wheel_track + self.wheel_length)
        if r_turn <= 0.0: r_turn = max(0.001, self.wheel_track)
        omega = (-vFL + vFR - vRL + vRR) / (4.0 * r_turn)

        # EKF predict step
        self.ekf.predict(Vx, Vy, omega, dt)

        # Use the latest scan timestamp for time synchronization
        with self.scan_stamp_lock:
            if self.latest_scan_stamp is not None:
                stamp = self.latest_scan_stamp
                self.get_logger().debug(f"Using scan timestamp for odom: {stamp.sec}.{stamp.nanosec}")
            else:
                stamp = now.to_msg()
                self.get_logger().debug("No scan timestamp available, using current time")

        # build /odom (fused)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.ekf.x[0]
        odom.pose.pose.position.y = self.ekf.x[1]
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.ekf.x[2])
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]

        # pose covariance: use EKF P but ensure minimum sensible diag values
        pose_cov = [0.0]*36
        pose_cov[0] = max(self.ekf.P[0][0], 0.02)      # var x
        pose_cov[7] = max(self.ekf.P[1][1], 0.02)      # var y
        pose_cov[14] = 1e6                             # z unknown/unused
        pose_cov[21] = 1e6; pose_cov[28] = 1e6
        pose_cov[35] = max(self.ekf.P[2][2], (0.05*0.05))  # var yaw
        odom.pose.covariance = pose_cov

        # twist fill (robot-frame)
        odom.twist.twist.linear.x = Vx
        odom.twist.twist.linear.y = Vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        twist_cov = [0.0]*36
        twist_cov[0] = 0.1; twist_cov[7] = 0.1; twist_cov[35] = 0.02
        odom.twist.covariance = twist_cov

        # publish fused odom
        self.odom_pub.publish(odom)

        # publish raw Arduino odom for debugging (reuse fused pose but mark larger cov)
        raw = Odometry()
        raw.header.stamp = stamp
        raw.header.frame_id = self.odom_frame
        raw.child_frame_id = self.base_frame
        raw.pose = odom.pose; raw.twist = odom.twist
        raw_cov = [0.0]*36; raw_cov[0]=0.2; raw_cov[7]=0.2; raw_cov[35]=0.5
        raw.pose.covariance = raw_cov; raw.twist.covariance = raw_cov
        self.arduino_odom_pub.publish(raw)

        # broadcast TF (use same stamp)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.ekf.x[0]
        t.transform.translation.y = self.ekf.x[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(t)

        # update last_time
        self.last_time = now

    # ---------- IMU parse ----------
    def _parse_I(self, parts, now):
        if len(parts) < 11:
            self.get_logger().warning("I message too short"); return
        ax, ay, az = self._try_float(parts[1]), self._try_float(parts[2]), self._try_float(parts[3])
        gx, gy, gz = self._try_float(parts[4]), self._try_float(parts[5]), self._try_float(parts[6])
        qx, qy, qz, qw = (self._try_float(parts[7]), self._try_float(parts[8]),
                          self._try_float(parts[9]), self._try_float(parts[10]))
        if None in (ax,ay,az,gx,gy,gz,qx,qy,qz,qw):
            self.get_logger().warning("Invalid IMU floats"); return

        self.last_imu_gz = gz
        # yaw from quaternion
        try:
            yaw = tf_transformations.euler_from_quaternion([qx,qy,qz,qw])[2]
        except Exception:
            yaw = self.ekf.x[2]
        self.last_imu_yaw = yaw

        # Use the latest scan timestamp for time synchronization
        with self.scan_stamp_lock:
            if self.latest_scan_stamp is not None:
                stamp = self.latest_scan_stamp
                self.get_logger().debug(f"Using scan timestamp for IMU: {stamp.sec}.{stamp.nanosec}")
            else:
                stamp = now.to_msg()
                self.get_logger().debug("No scan timestamp available for IMU, using current time")

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = ax; imu.linear_acceleration.y = ay; imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx; imu.angular_velocity.y = gy; imu.angular_velocity.z = gz
        imu.orientation.x = qx; imu.orientation.y = qy; imu.orientation.z = qz; imu.orientation.w = qw
        self.imu_pub.publish(imu)

        # EKF theta update (measurement)
        self.ekf.update_theta(yaw)

        # after update, publish immediate updated odom (same stamp)
        odom = Odometry()
        odom.header.stamp = stamp; odom.header.frame_id = self.odom_frame; odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.ekf.x[0]; odom.pose.pose.position.y = self.ekf.x[1]
        q = tf_transformations.quaternion_from_euler(0.0,0.0,self.ekf.x[2])
        odom.pose.pose.orientation.x=q[0]; odom.pose.pose.orientation.y=q[1]; odom.pose.pose.orientation.z=q[2]; odom.pose.pose.orientation.w=q[3]
        pose_cov = [0.0]*36
        pose_cov[0] = max(self.ekf.P[0][0], 0.02); pose_cov[7] = max(self.ekf.P[1][1], 0.02); pose_cov[35]=max(self.ekf.P[2][2], (0.05*0.05))
        odom.pose.covariance = pose_cov
        odom.twist.covariance = [0.0]*36
        self.odom_pub.publish(odom)
        
        # broadcast tf too
        t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = self.odom_frame; t.child_frame_id = self.base_frame
        t.transform.translation.x = self.ekf.x[0]; t.transform.translation.y = self.ekf.x[1]; t.transform.translation.z = 0.0
        t.transform.rotation.x=q[0]; t.transform.rotation.y=q[1]; t.transform.rotation.z=q[2]; t.transform.rotation.w=q[3]
        self.odom_broadcaster.sendTransform(t)

    # ---------- ultrasonic ----------
    def _parse_U(self, parts, now):
        if len(parts) < 2: return
        try:
            r = float(parts[1])
            
            # Use the latest scan timestamp for time synchronization
            with self.scan_stamp_lock:
                if self.latest_scan_stamp is not None:
                    stamp = self.latest_scan_stamp
                else:
                    stamp = now.to_msg()
                    
            msg = Range(); msg.header.stamp = stamp; msg.header.frame_id='ultrasonic_link'; msg.range=r
            self.ultra_pub.publish(msg)
        except Exception:
            pass

    # ---------- Arduino-provided O (optional) ----------
    def _parse_O(self, parts, now):
        if len(parts) < 7:
            self.get_logger().warning("O message too short"); return
        try:
            sx, sy, stheta, svx, svy, svtheta = parts[1:7]
            x = float(sx); y = float(sy); theta = float(stheta)
            vx = float(svx); vy = float(svy); vth = float(svtheta)
        except Exception:
            self.get_logger().warning("Malformed O message"); return
            
        # Use the latest scan timestamp for time synchronization
        with self.scan_stamp_lock:
            if self.latest_scan_stamp is not None:
                stamp = self.latest_scan_stamp
            else:
                stamp = now.to_msg()
                
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id=self.odom_frame; odom.child_frame_id=self.base_frame
        odom.pose.pose.position.x = x; odom.pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0,0,theta)
        odom.pose.pose.orientation.x=q[0]; odom.pose.pose.orientation.y=q[1]; odom.pose.pose.orientation.z=q[2]; odom.pose.pose.orientation.w=q[3]
        odom.twist.twist.linear.x = vx; odom.twist.twist.linear.y = vy; odom.twist.twist.angular.z = vth
        oc = [0.0]*36; oc[0]=0.05; oc[7]=0.05; oc[35]=0.05*0.05
        odom.pose.covariance = oc; odom.twist.covariance = oc
        self.arduino_odom_pub.publish(odom)

    # ---------- pure pursuit path subscriber ----------
    def _path_callback(self, msg: Path):
        # expect Path in "map" frame (or transform later). Store for controller node.
        self.current_path = msg
        # publish that pure pursuit is enabled (user can change to false externally)
        b = Bool(); b.data = True; self.pp_enabled_pub.publish(b)

    def _health_log(self):
        self.get_logger().debug(f"EKF state: {self.ekf.x}, P diag: [{self.ekf.P[0][0]:.4f},{self.ekf.P[1][1]:.4f},{self.ekf.P[2][2]:.4f}]")

# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
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