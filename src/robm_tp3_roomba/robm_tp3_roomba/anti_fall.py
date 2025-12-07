import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robm_interfaces.msg import Color
import numpy as np

class AntiFall(Node):
    """Anti-fall node using color detection with precise left turn."""

    def __init__(self):
        super().__init__('anti_fall')

        # Subscribe to color sensor
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)

        # Publisher for robot velocity
        self.pub_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Timer for main loop (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Current color
        self.current_color = np.array([0.0, 0.0, 0.0])

        # Floor reference (set to your safe floor color)
        self.floor_ref = np.array([8.0, 16.0, 27.0])
        self.tolerance = 10.0  # increase for earlier detection

        # Speeds
        self.forward_speed = 0.05
        self.reverse_speed = -0.15
        self.turn_speed = 0.5  # rad/s

        # State machine: 'startup', 'forward', 'reverse', 'turn'
        self.state = 'startup'
        self.state_counter = 0  # counts timer ticks in current state

        # Grace period before checking for cliff (1 sec at 10 Hz)
        self.startup_ticks = 10

        # Turn duration in ticks for 90° turn
        self.turn_ticks = int((np.pi / 2) / self.turn_speed * 10)  # 10 Hz timer

    def color_callback(self, msg):
        self.current_color = np.array([msg.r, msg.g, msg.b])

    def timer_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        dist = np.linalg.norm(self.current_color - self.floor_ref)

        if self.state == 'startup':
            # Initial grace period: just go forward without checking
            cmd.twist.linear.x = self.forward_speed
            cmd.twist.angular.z = 0.0
            self.state_counter += 1
            if self.state_counter >= self.startup_ticks:
                self.state = 'forward'
                self.state_counter = 0
                self.get_logger().info("Startup complete. Edge detection active.")

        elif self.state == 'forward':
            if dist > self.tolerance:
                # Detected cliff → switch to reverse
                self.state = 'reverse'
                self.state_counter = 0
                self.get_logger().info("Edge detected! Reversing...")
                cmd.twist.linear.x = self.reverse_speed
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = self.forward_speed
                cmd.twist.angular.z = 0.0

        elif self.state == 'reverse':
            # Reverse for ~0.5 sec (5 ticks at 10Hz)
            if self.state_counter < 5:
                cmd.twist.linear.x = self.reverse_speed
                cmd.twist.angular.z = 0.0
            else:
                # After reversing, start turning left
                self.state = 'turn'
                self.state_counter = 0
                self.get_logger().info("Turning left 90°...")
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed  # positive = left

        elif self.state == 'turn':
            if self.state_counter < self.turn_ticks:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed
            else:
                # Finished turn → resume forward
                self.state = 'forward'
                self.state_counter = 0
                cmd.twist.linear.x = self.forward_speed
                cmd.twist.angular.z = 0.0
                self.get_logger().info("Resuming forward movement.")

        # Only increment state_counter for reverse/turn states
        if self.state not in ['startup', 'forward']:
            self.state_counter += 1

        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AntiFall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
