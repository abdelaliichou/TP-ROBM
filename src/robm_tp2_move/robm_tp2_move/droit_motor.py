import rclpy
from rclpy.node import Node
from robm_interfaces.msg import RoverMotorsCmd

class DroiteMotor(Node):
    def __init__(self):
        super().__init__('droite_motor')
        self.publisher = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)
        self.counter = 0
        self.max_count = 2
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.counter < self.max_count:
            msg = RoverMotorsCmd()
            # Glisser à droite (axe Y)
            
            msg.front_left = 0.5
            msg.front_right = -0.5
            msg.rear_left = -0.5
            msg.rear_right = 0.5

            self.publisher.publish(msg)
            self.get_logger().info(f"Commande droite ({self.counter+1}/{self.max_count})")
            self.counter += 1
        else:
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
    node = DroiteMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
