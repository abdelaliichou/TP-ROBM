from typing import Optional
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from rclpy import qos

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, BatteryState, Range
from geometry_msgs.msg import Twist, TwistStamped
from robm_interfaces.msg import Color, RoverMotorsCmd, GripperCommand, ServoCommand

import json
import msgpack
from math import pi, fmod

import os


class ClockEstimator:
    def __init__(self) -> None:
        self.x = 0.0
        self.P = (1.0e20) ** 2
        self.Q = (1.0e-5) ** 2
        self.R = (1.0e-2) ** 2
        self._outlier_count = 0
    
    def update(self, ros_time_s, robot_time_s):
        z = ros_time_s - robot_time_s
        R = self.R if (z-self.x) < 1.e-3 else self.R*1000.0
        # Prediction
        self.x = self.x
        self.P = self.P + self.Q
        # Correction
        S = self.P + R
        K = self.P / S
        innovation = z - self.x
        if abs(innovation) > (6.0 * S**0.5):
            self._outlier_count += 1
            if self._outlier_count < 5:
                rclpy.logging.get_logger('zenoh_bridge').warn(f"Timing outlier detected: {innovation:.2f} s for an expected {S**0.5:.3f} s standard dev. Using ROS clock fallback.")
                return ros_time_s
            else:
                rclpy.logging.get_logger('zenoh_bridge').warn(f"Time discontinuity detected: {innovation:.2f} s for an expected {S**0.5:.3f} s standard dev. Resetting clock estimator.")
                self.P = (1.0e20) ** 2
                self._outlier_count = 0
        else:
            self._outlier_count = 0
        self.x = self.x + K * innovation
        self.P = (1.0 - K) * self.P * (1.0 - K) + K * R * K

        return self.x + robot_time_s
    
    def estimate_ros_time(self, robot_time_s):
        return self.x + robot_time_s

class BridgeNode(Node):
    def __init__(self, node_name: str='bridge_node'):
        super().__init__(node_name)
        self._use_twist_stamped = self.declare_parameter('use_twist_stamped', True).value
        # Publishers
        self.pub_imu_ = self.create_publisher(Imu, 'imu', qos.qos_profile_system_default)
        self.pub_battery_ = self.create_publisher(BatteryState, 'battery', qos.qos_profile_system_default)
        self.pub_tof_ = self.create_publisher(Range, 'tof', qos.qos_profile_system_default)
        self.pub_color_ = self.create_publisher(Color, 'color', qos.qos_profile_system_default)
        # Subscribers
        if self._use_twist_stamped:
            self.get_logger().info("Using TwistStamped for cmd_vel")
            self.sub_vel_ = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_vel_stamped_callback, qos.qos_profile_system_default)
        else:
            self.get_logger().info("Using Twist for cmd_vel")
            self.sub_vel_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos.qos_profile_system_default)
        self.sub_motors_ = self.create_subscription(RoverMotorsCmd, 'cmd_motors', self.cmd_motors_callback, qos.qos_profile_system_default)
        self.sub_servo = self.create_subscription(ServoCommand, 'cmd_servo', self.cmd_servo_callback, qos.qos_profile_system_default)
        self.sub_gripper = self.create_subscription(GripperCommand, 'cmd_gripper', self.cmd_gripper_callback, qos.qos_profile_system_default)
        # Timer
        timer_period = 0.05  # seconds (20 Hz max publish rate)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.clear_to_send = True
        # Pending commands
        self.need_to_send = False
        self._cmd_vel = None
        self._cmd_motors = None
        self._cmd_servo = None
        self._cmd_gripper = None
        # For yaw rate derivation
        self._last_yaw_time = None
        self._last_yaw = None
        # Clock estimator
        self.clock_estimator = ClockEstimator()


    def data_listener(self, payload: bytes):
        if not rclpy.ok():
            self.get_logger().debug('Received message but ROS is shutting down')
            return
        # print(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")
        if len(payload) == 0:
            self.get_logger().debug('Empty payload')
            return
        if payload[0] >= 0x80:
            self.get_logger().debug('MsgPack payload')
            data = msgpack.unpackb(payload, raw=False)
        else:
            self.get_logger().debug('JSON payload')
            data = json.loads(payload)
        #if sample.timestamp.get_time > 0:
        # timestamp = Time(nanoseconds=sample.timestamp.get_time * 10**9 / 2**32 -2_208_988_800*10**9).to_msg()
        # timestamp = Time(nanoseconds = data['t']*1000000).to_msg()
        now = self.get_clock().now()
        timestamp = self.get_clock().now().to_msg()
        # header = Header()
        # header.stamp = timestamp
        if 'imu' in data:
            # Estimate ROS time from robot time
            timestamp = Time(nanoseconds = self.clock_estimator.update(now.nanoseconds / 1e9, data['imu']['t'] / 1e3) * 1e9).to_msg()
            self.get_logger().debug(f"ROS time: {now.nanoseconds}  Robot time: {data['imu']['t']}  offset: {self.clock_estimator.x} P: {self.clock_estimator.P}")

            self.get_logger().debug('- IMU data')
            imu = Imu()
            imu.header.stamp = timestamp
            imu.header.frame_id = 'base_link'
            # Orientation is not measured
            imu.orientation_covariance[0] = -1
            # Gyro measurement (convert from deg/s to rad/s)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(x * pi / 180.0) for x in data['imu']['gyro'])
            # Accel. measurement (convert from g to m/s^2)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(x * 9.80665) for x in data['imu']['acc'])
            # Get yaw rate from integrated value (protects from data loss)
            if 'yaw' in data['imu']:
                if self._last_yaw_time is not None:
                    dt = (data['imu']['t'] - self._last_yaw_time) * 1.0e-3
                    dyaw = fmod(data['imu']['yaw'] - self._last_yaw, 2**32) / 3600.0 * pi / 180.0
                    imu.angular_velocity.z = dyaw / dt
                    self.get_logger().debug(f"yaw: {data['imu']['yaw']} {dt} {dyaw}")
                else:
                    self.get_logger().info("Computing yaw rate from integrated yaw")
                self._last_yaw_time = data['imu']['t']
                self._last_yaw = data['imu']['yaw']
            
            self.pub_imu_.publish(imu)
        if 'batt' in data:
            self.get_logger().debug('- Batt data')
            batt = BatteryState()
            batt.header.stamp = timestamp
            batt.header.frame_id = 'base_link'
            batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            batt.voltage = float(data['batt']['V'])
            batt.current = float(data['batt'].get('I', 'NaN'))
            if batt.current == float('NaN'):
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            elif batt.current > 0.0:
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif batt.current < 0.0:
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            elif batt.voltage > 4.1:
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            batt.percentage = float(data['batt'].get('pc', 'NaN'))
            self.pub_battery_.publish(batt)
        
        if 'tof' in data:
            self.get_logger().debug('- ToF range data')
            tof = Range()
            tof.header.stamp = timestamp
            tof.header.frame_id = 'base_link'
            tof.radiation_type = Range.INFRARED
            tof.field_of_view = 0.436332313
            tof.min_range = 0.05
            tof.max_range = 1.2
            tof.range = float(data['tof']['d']/1000.0)
            self.pub_tof_.publish(tof)
        
        if 'color' in data:
            self.get_logger().debug('- Color data')
            color = Color()
            color.header.stamp = timestamp
            color.header.frame_id = 'base_link'
            color.r = float(data['color']['r'])
            color.g = float(data['color']['g'])
            color.b = float(data['color']['b'])
            color.lux = float(data['color']['lux'])
            self.pub_color_.publish(color)
    
    def _send_command_data(self, payload: bytes):
        pass
        # self.get_logger().debug(f"Publishing to '{self.zenoh_pub.key_expr}: {command_msg}'...")
    
    def send_pending_commands(self):
        if not self.clear_to_send:
            self.get_logger().debug('Delaying command (too frequent)')
            return
        
        if self.need_to_send:
            self.clear_to_send = False
            self.need_to_send = False
            self.timer.reset()

            command_msg = {}

            if self._cmd_vel is not None:
                command_msg['speed'] = [self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z]
            if self._cmd_motors is not None:
                command_msg['motors'] = [self._cmd_motors.front_left,
                                        self._cmd_motors.front_right,
                                        self._cmd_motors.rear_left,
                                        self._cmd_motors.rear_right,
                                        ]
            if self._cmd_servo is not None:
                command_msg['servo'] = self._cmd_servo.angle
            if self._cmd_gripper is not None:
                command_msg['gripper'] = self._cmd_gripper.position
            
            if len(command_msg) > 0:
                self.get_logger().debug(f"Publishing command: {command_msg}'...")
                payload = msgpack.packb(command_msg, use_bin_type=True)
                if payload:
                    self._send_command_data(payload)
                # self.get_logger().debug(f"Publishing to '{self.zenoh_pub.key_expr}: {command_msg}'...")

    def timer_callback(self):
        self.clear_to_send = True
        self.send_pending_commands()

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.cmd_vel_callback(msg.twist)

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().debug(f"Received speed command: {msg}")
        self._cmd_vel = msg
        self._cmd_motors = None
        self.need_to_send = True
        self.send_pending_commands()
    
    def cmd_motors_callback(self, msg: RoverMotorsCmd):
        self.get_logger().debug(f"Received motors command: {msg}")
        self._cmd_motors = msg
        self._cmd_vel = None
        self.need_to_send = True
        self.send_pending_commands()
    
    def cmd_servo_callback(self, msg: ServoCommand):
        self.get_logger().debug(f"Received servo command: {msg}")
        self._cmd_servo = msg
        self.need_to_send = True
        self.send_pending_commands()

    def cmd_gripper_callback(self, msg: GripperCommand):
        self.get_logger().debug(f"Received gripper command: {msg}")
        self._cmd_gripper = msg
        self.need_to_send = True
        self.send_pending_commands()
