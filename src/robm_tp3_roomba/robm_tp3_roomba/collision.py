import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range


class Anticollision(Node):
    """Improved anticollision with smoother, slower behavior and 90° turn."""
    def __init__(self):
        super().__init__('anticollision')

        self.sub_tof = self.create_subscription(
            Range, 'tof', self.tof_callback, 10
        )

        self.pub_vel = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Slower timer → smoother reactions (10 Hz instead of 20 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.current_range = None

        # For smooth speed change
        self.current_speed = 0.0

    def tof_callback(self, msg: Range):
        """Save distance from TOF sensor."""
        self.current_range = msg.range

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.current_range is None:
            msg.twist.linear.x = 0.0
            self.pub_vel.publish(msg)
            return

        d = self.current_range

        # ===== NEW THRESHOLDS =====
        safe_distance = 0.80      # Start slowing earlier
        stop_distance = 0.30    # At this distance → turn left

        # ===== NEW SPEEDS =====
        max_speed = 0.20          # Much slower
        min_speed = 0.05

        # ===== Behavior rules =====

        # 1) Obstacle extremely close → turn LEFT 90°
        if d < stop_distance:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 1.2     # turn left smoothly
            self.pub_vel.publish(msg)
            self.get_logger().info("TOO CLOSE → Turning left")
            return

        # 2) Between stop_distance and safe_distance → slow down
        if stop_distance <= d < safe_distance:
            # Linear scale from max_speed → min_speed
            speed = min_speed + (max_speed - min_speed) * ((d - stop_distance) / (safe_distance - stop_distance))
        else:
            # 3) Far away → full slow speed
            speed = max_speed

        # Smooth motion (avoid jerky behavior)
        alpha = 0.2  # smoothing factor
        self.current_speed = alpha * speed + (1 - alpha) * self.current_speed

        # Apply forward speed
        msg.twist.linear.x = float(self.current_speed)
        msg.twist.angular.z = 0.0

        self.pub_vel.publish(msg)
        self.get_logger().info(f"Distance: {d:.2f} m → vx: {self.current_speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = Anticollision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
