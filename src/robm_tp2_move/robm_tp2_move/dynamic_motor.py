import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robm_interfaces.msg import RoverMotorsCmd

class VelToMotor(Node):
    """Converts TwistStamped messages into individual motor commands."""

    def __init__(self):
        super().__init__('vel_to_motor')
        self.get_logger().info("Node started: converting velocities to motor commands.")

        # Subscriber to cmd_vel or any TwistStamped topic
        self.sub = self.create_subscription(
            TwistStamped,
            'vel',  # topic to subscribe to (you can remap key_teleop output)
            self.vel_callback,
            10
        )

        # Publisher to cmd_motors
        self.pub = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)

    def vel_callback(self, msg: TwistStamped):
        """Convert linear and angular velocities to motor commands."""
        # Extract velocities
        vx = msg.twist.linear.x      # forward/back
        vy = msg.twist.linear.y      # left/right
        wz = msg.twist.angular.z     # rotation

        # Compute each wheel command for Mecanum robot
        motor_cmd = RoverMotorsCmd()
        motor_cmd.front_left  = vx - vy - wz
        motor_cmd.front_right = vx + vy + wz
        motor_cmd.rear_left   = vx + vy - wz
        motor_cmd.rear_right  = vx - vy + wz

        # Ensure values are within [-1, 1] for motor driver
        motor_cmd.front_left  = max(-1.0, min(1.0, motor_cmd.front_left))
        motor_cmd.front_right = max(-1.0, min(1.0, motor_cmd.front_right))
        motor_cmd.rear_left   = max(-1.0, min(1.0, motor_cmd.rear_left))
        motor_cmd.rear_right  = max(-1.0, min(1.0, motor_cmd.rear_right))

        # Publish motor command
        self.pub.publish(motor_cmd)
        self.get_logger().info(
            f"Motors cmd | FL:{motor_cmd.front_left:.2f} FR:{motor_cmd.front_right:.2f} "
            f"RL:{motor_cmd.rear_left:.2f} RR:{motor_cmd.rear_right:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()