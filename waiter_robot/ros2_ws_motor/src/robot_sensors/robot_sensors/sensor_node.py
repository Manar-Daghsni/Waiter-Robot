#!/usr/bin/env python3
"""
sensor_node.py (No IMU version)
- EKF odometry based on encoders + Arduino theta
- Publishes /odom, /arduino_odom, /encoder_ticks, /ultrasonic
- Handles TFs synchronized with odometry
- Robust serial parsing
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import threading, time, math, traceback, serial
import numpy as np

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Int32MultiArray, Bool, Float32MultiArray

import tf_transformations
import tf2_ros

# ----------------- EKF -----------------
class EnhancedEKFOdometry:
    def __init__(self, Q_diag=(0.02,0.02,0.01), R_theta=0.0025):
        self.lock = threading.RLock()
        self.x = np.zeros(3)
        self.P = np.eye(3)*1e-3
        self.Q = np.diag(Q_diag)
        self.R_theta = R_theta
        self.min_cov = 1e-6
        self.max_cov = 1.0

    def predict(self, Vx, Vy, omega, dt):
        if dt <= 1e-6: return
        with self.lock:
            theta = self.x[2]
            ct, st = math.cos(theta), math.sin(theta)
            dx, dy, dth = Vx*dt, Vy*dt, omega*dt
            self.x[0] += dx*ct - dy*st
            self.x[1] += dx*st + dy*ct
            self.x[2] = self._wrap(self.x[2]+dth)
            F = np.eye(3)
            F[0,2] = -dt*(Vx*st + Vy*ct)
            F[1,2] = dt*(Vx*ct - Vy*st)
            self.P = F@self.P@F.T + self.Q*dt
            np.fill_diagonal(self.P, np.clip(np.diag(self.P), self.min_cov, self.max_cov))

    def update_theta(self, theta_meas):
        with self.lock:
            H = np.array([[0,0,1]])
            R = np.array([[self.R_theta]])
            y = self._wrap(theta_meas - self.x[2])
            S = H@self.P@H.T + R
            K = self.P@H.T@np.linalg.inv(S)
            self.x += (K.flatten()*y)
            self.x[2] = self._wrap(self.x[2])
            I = np.eye(3)
            self.P = (I-K@H)@self.P@(I-K@H).T + K@R@K.T
            np.fill_diagonal(self.P, np.clip(np.diag(self.P), self.min_cov, self.max_cov))

    def _wrap(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

# ----------------- Sensor Node -----------------
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameters('', [
            ('port','/dev/ttyACM0'), ('baudrate',57600),
            ('wheel_radius',0.0248), ('wheel_track',0.35),
            ('wheel_length',0.28), ('ticks_per_rev',4),
            ('ignore_O_for_raw_odom',True),
            ('odom_frame','odom'), ('base_frame','base_link'),
            ('use_scan_stamp_for_odom',True), ('scan_stamp_max_age',0.25),
            ('fallback_publish_hz',20.0), ('max_future_time_s',0.02),
            ('debug_logging',False), ('ekf_Q_scale',1.0), ('ekf_R_scale',1.0)
        ])

        # Params
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.wheel_length = self.get_parameter('wheel_length').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.ignore_O_for_raw_odom = self.get_parameter('ignore_O_for_raw_odom').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_scan_stamp_for_odom = self.get_parameter('use_scan_stamp_for_odom').value
        self.scan_stamp_max_age = self.get_parameter('scan_stamp_max_age').value
        self.fallback_publish_hz = self.get_parameter('fallback_publish_hz').value
        self.max_future_time_s = self.get_parameter('max_future_time_s').value
        self.debug_logging = self.get_parameter('debug_logging').value
        ekf_Q_scale = self.get_parameter('ekf_Q_scale').value
        ekf_R_scale = self.get_parameter('ekf_R_scale').value

        if self.debug_logging:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # EKF
        Q_diag = (0.02*ekf_Q_scale,0.02*ekf_Q_scale,0.01*ekf_Q_scale)
        R_theta = 0.0025*ekf_R_scale
        self.ekf = EnhancedEKFOdometry(Q_diag, R_theta)

        # Publishers
        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.ultra_pub_front = self.create_publisher(Range, 'ultrasonic/front', 10)
        self.ultra_pub_back  = self.create_publisher(Range, 'ultrasonic/back', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray,'/encoder_ticks',10)
        self.odom_pub = self.create_publisher(Odometry,'/odom',qos_odom)
        self.arduino_odom_pub = self.create_publisher(Odometry,'/arduino_odom',10)
        self.wheel_vels_pub = self.create_publisher(Float32MultiArray,'/wheel_vels',10)
        self.pp_enabled_pub = self.create_publisher(Bool,'/pure_pursuit/enabled',10)

        # Subscribers
        self.create_subscription(Path,'/pure_pursuit/path',self._path_callback,10)
        self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.create_subscription(LaserScan,'/scan',self.scan_callback,qos_profile_sensor_data)

        # State
        self.last_fl=self.last_fr=self.last_rl=self.last_rr=0
        self.last_time=None
        self.pending_odom=None
        self.pending_lock=threading.RLock()
        self.latest_scan_stamp=None
        self.scan_stamp_lock=threading.Lock()
        self.last_published_stamp=None

        # Serial
        self.ser=None
        self.serial_lock=threading.RLock()
        self.stop_reader=threading.Event()
        self.reader_thread=None

        # TF
        self.odom_broadcaster=tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster=tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        # Timers
        period=1.0/max(1.0,self.fallback_publish_hz)
        self.fallback_timer=self.create_timer(period,self._periodic_publish_fallback)
        self.health_timer=self.create_timer(5.0,self._health_monitor)

        # Serial thread
        self._open_serial()
        self.reader_thread=threading.Thread(target=self._serial_reader_loop,daemon=True)
        self.reader_thread.start()
        self.get_logger().info(f"Sensor node started on {self.port}@{self.baud}")

    # Static TFs (no imu_link anymore)
    def _publish_static_transforms(self):
        transforms=[]
        t_laser=TransformStamped()
        t_laser.header.stamp=self.get_clock().now().to_msg()
        t_laser.header.frame_id=self.base_frame
        t_laser.child_frame_id='laser'
        t_laser.transform.translation.z=0.08
        t_laser.transform.rotation.w=1.0
        transforms.append(t_laser)
        try:
            self.static_broadcaster.sendTransform(transforms)
            self.get_logger().info("Published static transforms")
        except Exception as e:
            self.get_logger().error(f"Static transform failed: {e}")

    # Serial handling
    def _open_serial(self):
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser=serial.Serial(port=self.port,baudrate=self.baud,timeout=0.1,write_timeout=0.1)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return True
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser=None
            return False

    def _serial_reader_loop(self):
        buffer=""
        while not self.stop_reader.is_set():
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(1.0)
                    self._open_serial()
                    continue
                raw=self.ser.read(self.ser.in_waiting or 1)
                if not raw: continue
                buffer+=raw.decode('utf-8',errors='ignore')
                while '\n' in buffer:
                    line, buffer=buffer.split('\n',1)
                    self._process_serial_line(line.strip())
            except Exception as e:
                time.sleep(0.01)
                self.get_logger().debug(f"Serial loop error: {e}")

    def _process_serial_line(self,line):
        if not line: return
        parts=line.split(',')
        now=self.get_clock().now()
        tag=parts[0].strip()
        try:
            if tag=='E' and len(parts)>=5: self._parse_encoder(parts,now)
            elif tag=='U' and len(parts)>=3: self._parse_ultrasonic(parts,now)
            elif tag=='O' and not self.ignore_O_for_raw_odom and len(parts)>=7: self._parse_odometry(parts,now)
        except Exception as e:
            self.get_logger().debug(f"Line parse error ({tag}): {e}")

    # Parser Methods
    def _parse_encoder(self,parts,now):
        try:
            fl,fr,rl,rr=[int(float(p)) for p in parts[1:5]]
            self.encoder_pub.publish(Int32MultiArray(data=[fl,fr,rl,rr]))
            dt=0.05 if self.last_time is None else max(0.001,(now-self.last_time).nanoseconds/1e9)
            circ=2*math.pi*self.wheel_radius
            vFL=(fl-self.last_fl)/self.ticks_per_rev*circ/dt
            vFR=(fr-self.last_fr)/self.ticks_per_rev*circ/dt
            vRL=(rl-self.last_rl)/self.ticks_per_rev*circ/dt
            vRR=(rr-self.last_rr)/self.ticks_per_rev*circ/dt
            self.wheel_vels_pub.publish(Float32MultiArray(data=[vFL,vFR,vRL,vRR]))
            Vx=(vFL+vFR+vRL+vRR)*0.25
            Vy=(-vFL+vFR+vRL-vRR)*0.25
            r_turn=0.5*(self.wheel_track+self.wheel_length)
            omega=(-vFL+vFR-vRL+vRR)/(4.0*max(0.001,r_turn))
            self.ekf.predict(Vx,Vy,omega,dt)

            odom=Odometry()
            odom.header.frame_id=self.odom_frame
            odom.child_frame_id=self.base_frame
            odom.pose.pose.position.x=self.ekf.x[0]
            odom.pose.pose.position.y=self.ekf.x[1]
            q=tf_transformations.quaternion_from_euler(0,0,self.ekf.x[2])
            odom.pose.pose.orientation.x=q[0]
            odom.pose.pose.orientation.y=q[1]
            odom.pose.pose.orientation.z=q[2]
            odom.pose.pose.orientation.w=q[3]
            cov=np.zeros(36)
            cov[0]=max(self.ekf.P[0,0],0.02)
            cov[7]=max(self.ekf.P[1,1],0.02)
            cov[35]=max(self.ekf.P[2,2],0.0025)
            odom.pose.covariance=cov.tolist()
            odom.twist.twist.linear.x=Vx
            odom.twist.twist.linear.y=Vy
            odom.twist.twist.angular.z=omega

            with self.pending_lock:
                self.pending_odom=odom

            self.last_fl,self.last_fr,self.last_rl,self.last_rr=fl,fr,rl,rr
            self.last_time=now
        except Exception as e:
            self.get_logger().debug(f"Encoder parse error: {e}")

    def _parse_ultrasonic(self, parts, now):
        try:
            front_dist = float(parts[1])
            back_dist  = float(parts[2])
            with self.scan_stamp_lock:
                stamp = self.latest_scan_stamp or now.to_msg()
            msg_front = Range()
            msg_front.header.stamp = stamp
            msg_front.header.frame_id = 'ultrasonic_front'
            msg_front.range = front_dist
            msg_front.radiation_type = Range.ULTRASOUND
            msg_front.field_of_view = 0.26
            msg_front.min_range = 0.02
            msg_front.max_range = 4.0
            self.ultra_pub_front.publish(msg_front)
            msg_back = Range()
            msg_back.header.stamp = stamp
            msg_back.header.frame_id = 'ultrasonic_back'
            msg_back.range = back_dist
            msg_back.radiation_type = Range.ULTRASOUND
            msg_back.field_of_view = 0.26
            msg_back.min_range = 0.02
            msg_back.max_range = 4.0
            self.ultra_pub_back.publish(msg_back)
        except Exception as e:
            self.get_logger().debug(f"Ultrasonic parse error: {e}")

    def _parse_odometry(self,parts,now):
        try:
            x,y,theta,vx,vy,vth=[float(p) for p in parts[1:7]]
            odom=Odometry()
            odom.header.stamp=now.to_msg()
            odom.header.frame_id=self.odom_frame
            odom.child_frame_id=self.base_frame
            odom.pose.pose.position.x=x
            odom.pose.pose.position.y=y
            q=tf_transformations.quaternion_from_euler(0,0,theta)
            odom.pose.pose.orientation.x=q[0]
            odom.pose.pose.orientation.y=q[1]
            odom.pose.pose.orientation.z=q[2]
            odom.pose.pose.orientation.w=q[3]
            odom.twist.twist.linear.x=vx
            odom.twist.twist.linear.y=vy
            odom.twist.twist.angular.z=vth
            self.arduino_odom_pub.publish(odom)
            # Update EKF theta
            self.ekf.update_theta(theta)
        except Exception as e:
            self.get_logger().debug(f"Arduino odom parse error: {e}")

    # ROS Callbacks
    def scan_callback(self,msg):
        try:
            now=self.get_clock().now()
            scan_time=rclpy.time.Time.from_msg(msg.header.stamp) if msg.header.stamp else now
            diff=(scan_time-now).nanoseconds/1e9
            if diff>self.max_future_time_s:
                scan_time=now+rclpy.time.Duration(seconds=self.max_future_time_s)
            with self.scan_stamp_lock:
                self.latest_scan_stamp=scan_time.to_msg()
            with self.pending_lock:
                if not self.pending_odom: return
                scan_time_obj=rclpy.time.Time.from_msg(self.latest_scan_stamp)
                age=(now-scan_time_obj).nanoseconds/1e9
                if self.use_scan_stamp_for_odom and age<=self.scan_stamp_max_age:
                    if self.last_published_stamp and scan_time_obj<=self.last_published_stamp:
                        scan_time_obj=self.last_published_stamp+rclpy.time.Duration(nanoseconds=1)
                    stamp=scan_time_obj.to_msg()
                    self.pending_odom.header.stamp=stamp
                    self.odom_pub.publish(self.pending_odom)
                    t=TransformStamped()
                    t.header.stamp=stamp
                    t.header.frame_id=self.odom_frame
                    t.child_frame_id=self.base_frame
                    t.transform.translation.x=self.pending_odom.pose.pose.position.x
                    t.transform.translation.y=self.pending_odom.pose.pose.position.y
                    t.transform.rotation=self.pending_odom.pose.pose.orientation
                    self.odom_broadcaster.sendTransform(t)
                    self.last_published_stamp=scan_time_obj
                    self.pending_odom=None
        except Exception as e:
            self.get_logger().error(f"Scan callback error: {e}")

    def _periodic_publish_fallback(self):
        with self.pending_lock:
            if not self.pending_odom: return
            now=self.get_clock().now()
            with self.scan_stamp_lock:
                if self.latest_scan_stamp and self.use_scan_stamp_for_odom:
                    age=(now-rclpy.time.Time.from_msg(self.latest_scan_stamp)).nanoseconds/1e9
                    if age<=self.scan_stamp_max_age:
                        return
            if self.last_published_stamp and now<=self.last_published_stamp:
                now=self.last_published_stamp+rclpy.time.Duration(nanoseconds=1)
            stamp=now.to_msg()
            self.pending_odom.header.stamp=stamp
            self.odom_pub.publish(self.pending_odom)
            t=TransformStamped()
            t.header.stamp=stamp
            t.header.frame_id=self.odom_frame
            t.child_frame_id=self.base_frame
            t.transform.translation.x=self.pending_odom.pose.pose.position.x
            t.transform.translation.y=self.pending_odom.pose.pose.position.y
            t.transform.rotation=self.pending_odom.pose.pose.orientation
            self.odom_broadcaster.sendTransform(t)
            self.last_published_stamp=now
            self.pending_odom=None

    def cmd_vel_callback(self,msg):
        try:
            lin,ang=msg.linear.x,msg.angular.z
            cmd='s\n'
            if lin>0.05: cmd='f\n'
            elif lin<-0.05: cmd='b\n'
            elif ang>0.05: cmd='l\n'
            elif ang<-0.05: cmd='r\n'
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warning(f"Cmd_vel failed: {e}")

    def _path_callback(self,msg):
        try:
            enabled=Bool()
            enabled.data=bool(msg.poses)
            self.pp_enabled_pub.publish(enabled)
        except Exception as e:
            self.get_logger().debug(f"Path callback error: {e}")

    def _health_monitor(self):
        try:
            if not (self.ser and self.ser.is_open):
                self.get_logger().warning("Serial unhealthy")
            if self.debug_logging:
                cov_diag=np.diag(self.ekf.P)
                self.get_logger().debug(f"EKF: x={self.ekf.x[0]:.2f}, y={self.ekf.x[1]:.2f}, θ={self.ekf.x[2]:.3f}, cov={cov_diag}")
        except Exception as e:
            self.get_logger().debug(f"Health monitor error: {e}")

    def destroy_node(self):
        self.stop_reader.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()

# ----------------- Main -----------------
def main(args=None):
    rclpy.init(args=args)
    try:
        node=SensorNode()
        executor=MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
            node.destroy_node()
    except Exception as e:
        print(f"Node init failed: {e}")
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()

