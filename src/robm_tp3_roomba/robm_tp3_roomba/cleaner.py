import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robm_interfaces.msg import Color
import numpy as np

class SpiralClean(Node):
    """Robot nettoyeur: forward, anti-fall, auto dock, spiral red patches."""

    def __init__(self):
        super().__init__('spiral_clean')

        # Color sensor subscription
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)

        # Velocity publisher
        self.pub_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Current color
        self.current_color = np.array([0.0, 0.0, 0.0])

        # References
        self.floor_ref = np.array([8.0, 16.0, 27.0])
        self.green_ref = np.array([7.31, 30.83, 12.60])
        self.red_ref = np.array([48.0, 6.90, 8.94])

        # Tolerances
        self.floor_tol = 10.0
        self.green_tol = 8.0
        self.red_tol = 12.0

        # Speeds
        self.forward_speed = 0.05
        self.reverse_speed = -0.15
        self.turn_speed = 0.5
        self.spiral_linear_speed = 0.04
        self.spiral_angular_speed = 0.3

        # State machine
        self.state = 'startup'
        self.state_counter = 0
        self.startup_ticks = 10
        self.turn_ticks = int((np.pi / 2) / self.turn_speed * 10)
        self.spiral_ticks = 50  # ~5 sec spiral at 10 Hz

    def color_callback(self, msg):
        self.current_color = np.array([msg.r, msg.g, msg.b])

    def timer_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Compute distances
        dist_floor = np.linalg.norm(self.current_color - self.floor_ref)
        dist_green = np.linalg.norm(self.current_color - self.green_ref)
        dist_red = np.linalg.norm(self.current_color - self.red_ref)

        # ---------------------
        # STATE MACHINE
        # ---------------------
        if self.state == 'startup':
            # Move forward at start
            cmd.twist.linear.x = self.forward_speed
            cmd.twist.angular.z = 0.0
            self.state_counter += 1
            if self.state_counter >= self.startup_ticks:
                self.state = 'forward'
                self.state_counter = 0
                self.get_logger().info("Startup complete. Normal operation.")

        elif self.state == 'forward':
            # Priority order: red patch → green base → edge → normal forward
            if dist_red < self.red_tol:
                self.state = 'spiral'
                self.state_counter = 0
                self.get_logger().info("Red patch detected! Starting spiral.")
                cmd.twist.linear.x = self.spiral_linear_speed
                cmd.twist.angular.z = self.spiral_angular_speed

            elif dist_green < self.green_tol:
                self.state = 'stop'
                self.get_logger().info("Green base detected! Stopping.")
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0

            elif dist_floor > self.floor_tol:
                self.state = 'reverse'
                self.state_counter = 0
                self.get_logger().info("Edge detected! Reversing...")
                cmd.twist.linear.x = self.reverse_speed
                cmd.twist.angular.z = 0.0

            else:
                cmd.twist.linear.x = self.forward_speed
                cmd.twist.angular.z = 0.0

        elif self.state == 'reverse':
            if self.state_counter < 5:
                cmd.twist.linear.x = self.reverse_speed
                cmd.twist.angular.z = 0.0
            else:
                self.state = 'turn'
                self.state_counter = 0
                self.get_logger().info("Turning left 90°...")
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed

        elif self.state == 'turn':
            if self.state_counter < self.turn_ticks:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = self.turn_speed
            else:
                self.state = 'forward'
                self.state_counter = 0
                self.get_logger().info("Turn complete. Resuming forward.")
                cmd.twist.linear.x = self.forward_speed
                cmd.twist.angular.z = 0.0

        elif self.state == 'spiral':
            if self.state_counter < self.spiral_ticks:
                cmd.twist.linear.x = self.spiral_linear_speed
                cmd.twist.angular.z = self.spiral_angular_speed
            else:
                self.state = 'forward'
                self.state_counter = 0
                self.get_logger().info("Spiral complete. Resuming forward.")
                cmd.twist.linear.x = self.forward_speed
                cmd.twist.angular.z = 0.0

        elif self.state == 'stop':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        # Increment counters for timed states
        if self.state in ['reverse', 'turn', 'spiral']:
            self.state_counter += 1

        # Publish command
        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpiralClean()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
