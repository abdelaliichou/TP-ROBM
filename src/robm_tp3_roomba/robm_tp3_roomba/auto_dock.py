import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robm_interfaces.msg import Color
import numpy as np

class AutoDock(Node):
    """Node to stop the robot on green base using color detection."""

    def __init__(self):
        super().__init__('auto_dock')

        # Subscribe to the color sensor
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)

        # Publisher for robot velocity
        self.pub_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Timer for periodic velocity commands
        timer_period = 0.2  # 5 Hz for slower updates
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Current color reading (r,g,b)
        self.current_color = np.array([0.0, 0.0, 0.0])

        # Green base RGB measured earlier
        self.green_ref = np.array([7.31, 30.83, 12.60])
        self.tolerance = 8.0  # Increase tolerance so it stops earlier

        self.forward_speed = 0.05  # slower speed

        self.get_logger().info("AutoDock node started (slower, safer).")

    def color_callback(self, msg):
        """Update current color reading."""
        self.current_color = np.array([msg.r, msg.g, msg.b])

    def timer_callback(self):
        """Publish velocity based on color."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Compute distance to green reference
        dist = np.linalg.norm(self.current_color - self.green_ref)

        if dist < self.tolerance:
            # Green detected → stop
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().info("Green base detected → STOP")
        else:
            # Move forward slowly
            cmd.twist.linear.x = self.forward_speed
            cmd.twist.angular.z = 0.0
            self.get_logger().info(f"Moving forward | dist to green: {dist:.2f}")

        self.pub_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
