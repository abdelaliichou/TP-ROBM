import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from robm_interfaces.msg import RoverMotorsCmd


class VelToMotor(Node):
    """Convert incoming velocity commands into continuous motor commands."""

    def __init__(self):
        super().__init__('vel_to_motor')

        self.get_logger().info("vel_to_motor running (smooth mode enabled).")

        # Last received velocity (TwistStamped)
        self.last_vel = TwistStamped()

        # Subscriber to 'vel'
        self.sub = self.create_subscription(
            TwistStamped,
            'vel',
            self.vel_callback,
            10
        )

        # Publisher to motors
        self.pub = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)

        # Publish motor commands continuously (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_motor_cmd)

    def vel_callback(self, msg):
        """Store the last received velocity command."""
        self.last_vel = msg

    def publish_motor_cmd(self):
        """Convert last velocity to motor commands and publish continuously."""

        vx = self.last_vel.twist.linear.x
        vy = self.last_vel.twist.linear.y
        wz = self.last_vel.twist.angular.z

        motor_cmd = RoverMotorsCmd()

        # Mecanum kinematics
        motor_cmd.front_left  = vx - vy - wz
        motor_cmd.front_right = vx + vy + wz
        motor_cmd.rear_left   = vx + vy - wz
        motor_cmd.rear_right  = vx - vy + wz

        # Clamp to [-1, 1]
        motor_cmd.front_left  = max(-1.0, min(1.0, motor_cmd.front_left))
        motor_cmd.front_right = max(-1.0, min(1.0, motor_cmd.front_right))
        motor_cmd.rear_left   = max(-1.0, min(1.0, motor_cmd.rear_left))
        motor_cmd.rear_right  = max(-1.0, min(1.0, motor_cmd.rear_right))

        self.pub.publish(motor_cmd)

        # Optional debug
        # self.get_logger().info(
        #     f"FL={motor_cmd.front_left:.2f} FR={motor_cmd.front_right:.2f} "
        #     f"RL={motor_cmd.rear_left:.2f} RR={motor_cmd.rear_right:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = VelToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
