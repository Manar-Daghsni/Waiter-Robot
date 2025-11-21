def __init__(self):
    super().__init__('sensor_node')

    # Publishers
    self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
    self.odom_pub = self.create_publisher(Odometry, 'odom', 20)   # use 'odom' for Nav2
    self.ultrasonic_front_pub = self.create_publisher(Range, 'ultrasonic/front', 10)
    self.ultrasonic_right_pub = self.create_publisher(Range, 'ultrasonic/right', 10)
    self.ultrasonic_back_pub = self.create_publisher(Range, 'ultrasonic/back', 10)
    self.ultrasonic_left_pub = self.create_publisher(Range, 'ultrasonic/left', 10)

    # Serial connection
    self.serial_port = '/dev/ttyACM0'
    self.baudrate = 57600
    self.serial_conn = None
    self.connect_serial()

    # Timers
    self.create_timer(0.05, self.read_serial)  # 20 Hz
    self.create_timer(0.1, self.check_cmd_timeout)

    # Teleop
    self.last_cmd = None
    self.cmd_timeout = 0.5
    self.last_cmd_time = self.get_clock().now()
    self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    # Odom jump guard
    self.last_odom_x = 0.0
    self.last_odom_y = 0.0
    self.last_odom_theta = 0.0
