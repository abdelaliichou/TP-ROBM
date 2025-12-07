import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range


class Anticollision(Node):
    """Move forward, slow down near walls, reverse when too close."""

    def __init__(self):
        super().__init__('anticollision')

        # Subscribe to ToF sensor ("tof")
        self.sub_tof = self.create_subscription(
            Range,
            'tof',
            self.tof_callback,
            10
        )

        # Publish TwistStamped to cmd_vel
        self.pub_vel = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10
        )

        # Timer: 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Last measured distance (meters)
        self.current_range = 1.0   # default = safe distance

        self.get_logger().info("Anticollision node started.")

    # -------------------------------
    # TOF CALLBACK
    # -------------------------------
    def tof_callback(self, msg: Range):
        """Save the measured range from the ToF sensor."""
        self.current_range = msg.range   # <-- real distance in meters

    # -------------------------------
    # CONTROL LOOP (20 Hz)
    # -------------------------------
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        d = self.current_range   # short alias

        # --- BEHAVIOR RULES ---
        if d > 0.50:
            # Normal forward speed (0.3 m/s)
            vx = 0.2

        elif 0.20 < d <= 0.50:
            # Slow down linearly
            # when d = 0.50 → vx = 0.30
            # when d = 0.20 → vx = 0.00
            vx = 0.30 * (d - 0.20) / (0.50 - 0.20)

        else:
            # d < 0.20 → reverse proportionally
            # when d = 0.20 → 0.0
            # when d = 0.05 → -0.25 (example)
            min_dist = 0.05
            d_clamped = max(d, min_dist)
            vx = -0.2 * (0.20 - d_clamped) / (0.20 - min_dist)

        # Set TwistStamped fields
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0

        # Publish
        self.pub_vel.publish(msg)

        # Debug
        self.get_logger().info(
            f"Distance={d:.2f} m → vx={vx:.2f} m/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Anticollision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
