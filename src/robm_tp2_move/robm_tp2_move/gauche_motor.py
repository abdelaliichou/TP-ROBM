import rclpy
from rclpy.node import Node
from robm_interfaces.msg import RoverMotorsCmd

class GaucheMotor(Node):
    def __init__(self):
        super().__init__('gauche_motor')

        # Publisher vers cmd_motors
        self.publisher = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)

        # Timer à 10 Hz
        self.counter = 0
        self.max_count =2  # 20 messages → 2 secondes
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.counter < self.max_count:
            msg = RoverMotorsCmd()
            # Translation gauche (axe Y)
            msg.front_left = -0.5
            msg.front_right = 0.5
            msg.rear_left = 0.5
            msg.rear_right = -0.5

            self.publisher.publish(msg)
            self.get_logger().info(f"Commande gauche envoyée ({self.counter+1}/{self.max_count})")
            self.counter += 1
        else:
            # Stop le robot après 2 secondes
            msg = RoverMotorsCmd()
            msg.front_left = 0.0
            msg.front_right = 0.0
            msg.rear_left = 0.0
            msg.rear_right = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Arrêt du robot.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = GaucheMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
