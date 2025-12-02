import rclpy
from rclpy.node import Node
from robm_interfaces.msg import Color
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('couleur')

        # Abonnement au capteur couleur
        self.subscription = self.create_subscription(
            Color,
            'color',
            self.callback_color,
            10
        )

        # Valeurs moyennes que TU dois mesurer et mettre ici
        # Par exemple :
        self.known_colors = {
            "blue":   np.array([1.6578733921051025, 12.945362091064453, 60.32769775390625]),
            "black":  np.array([8.037386894226074, 16.358259201049805, 27.269371032714844]),
            "white":  np.array([4.608364105224609, 17.38037872314453, 31.706939697265625]),
            "orange": np.array([29.861305236816406, 13.12883472442627, 8.675752639770508]),
            "yellow": np.array([21.959293365478516, 23.051610946655273, 5.030496120452881]),
            "gray":   np.array([5.279051780700684, 17.190895080566406, 30.710670471191406]),
            "green":  np.array([7.309666156768799, 30.82889747619629, 12.597602844238281]),
            "red":    np.array([48.03636932373047, 6.9033050537109375, 8.942831039428711]),
            "violet": np.array([4.543237209320068, 10.904176712036133, 49.75279998779297])
        }


        self.get_logger().info("Nœud couleur lancé.")

    def callback_color(self, msg):
        current = np.array([msg.r, msg.g, msg.b])

        # Distance euclidienne à chaque couleur connue
        distances = {}
        for name, ref in self.known_colors.items():
            distances[name] = np.linalg.norm(current - ref)

        detected_color = min(distances, key=distances.get)

        self.get_logger().info(f"Robot is detecting => {detected_color}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
