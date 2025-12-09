#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path

from math import cos, sin

# Fonctions utilitaires pour créer un Quaternion à partir du yaw
def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Crée un Quaternion à partir d'un angle de lacet (yaw)"""
    from math import cos, sin
    roll = 0.0
    pitch = 0.0
    yaw /= 2.0
    cr = cos(roll / 2.0)
    sr = sin(roll / 2.0)
    cp = cos(pitch / 2.0)
    sp = sin(pitch / 2.0)
    cy = cos(yaw)
    sy = sin(yaw)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # Publisher pour publier la position estimée
        self._odom_pub = self.create_publisher(Odometry, 'odometry', 10)

        # Subscriber IMU
        self._imu_sub = self.create_subscription(Imu, 'calibrated_imu', self.imu_callback, 10)
        # Subscriber cmd_vel
        self._cmd_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_callback, 10)

        # Inside your OdometryNode.__init__():
        self._path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Inside imu_callback, after publishing odometry:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_stamped.header.frame_id = odom_msg.header.frame_id
        pose_stamped.pose = odom_msg.pose.pose

        self.path_msg.header.stamp = odom_msg.header.stamp
        self.path_msg.poses.append(pose_stamped)
        self._path_pub.publish(self.path_msg)

        # Vitesse linéaire et latérale (m/s)
        self.vx = 0.0
        self.vy = 0.0
        # Vitesse angulaire mesurée par le gyro (rad/s)
        self.omega = 0.0

        # Position et orientation initiales
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Temps précédent pour calculer Δt
        self.last_time = None

    def cmd_callback(self, msg: TwistStamped):
        """Mémorise la commande en vitesse"""
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y

    def imu_callback(self, msg: Imu):
        """Mise à jour de la position à partir de la vitesse angulaire et linéaire"""

        # Temps courant
        current_time = self.get_clock().now().nanoseconds * 1e-9  # secondes

        # Calcul Δt
        if self.last_time is None:
            dt = 1.0 / 20.0  # supposition fréquence IMU 20Hz pour la première mesure
        else:
            dt = current_time - self.last_time

        self.last_time = current_time

        # Lecture vitesse angulaire autour de Z
        self.omega = msg.angular_velocity.z

        # Intégration discrète (Euler) pour calculer la nouvelle position
        x_new = self.x + dt * (self.vx * cos(self.theta) - self.vy * sin(self.theta))
        y_new = self.y + dt * (self.vx * sin(self.theta) + self.vy * cos(self.theta))
        theta_new = self.theta + dt * self.omega

        self.x = x_new
        self.y = y_new
        self.theta = theta_new

        # Création du message Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Publication
        self._odom_pub.publish(odom_msg)

        # Log (optionnel)
        self.get_logger().info(f"x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
