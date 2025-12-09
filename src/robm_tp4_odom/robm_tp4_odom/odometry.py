#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # 1. Souscription à la commande de vitesse (pour vx, vy)
        # On estime la vitesse linéaire à partir de la commande [cite: 55]
        self.sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # 2. Souscription à l'IMU calibrée (pour omega_z) [cite: 53]
        self.sub_imu = self.create_subscription(
            Imu, 'calibrated_imu', self.imu_callback, 10)
        
        # 3. Publisher pour l'odométrie calculée [cite: 67]
        self.pub_odom = self.create_publisher(Odometry, 'odometry', 10)

        # État du robot (x, y, theta) initialisé à 0 [cite: 46]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Vitesses actuelles
        self.v_x = 0.0
        self.v_y = 0.0
        
        # Gestion du temps pour l'intégration
        self.last_time = self.get_clock().now()

    def cmd_callback(self, msg: Twist):
        # Mémorisation de la commande reçue [cite: 56]
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y

    def imu_callback(self, msg: Imu):
        # C'est la réception de l'IMU qui déclenche le calcul et la publication 
        current_time = self.get_clock().now()
        
        # Calcul du delta t (en secondes) [cite: 64]
        # On récupère les nanosecondes et on convertit en secondes
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # La vitesse de rotation vient du gyromètre (IMU) [cite: 51]
        omega_z = msg.angular_velocity.z

        # Intégration d'Euler (Modèle d'évolution) [cite: 64]
        # x_k+1 = x_k + dt * (vx * cos(theta) - vy * sin(theta))
        self.x += dt * (self.v_x * math.cos(self.theta) - self.v_y * math.sin(self.theta))
        
        # y_k+1 = y_k + dt * (vx * sin(theta) + vy * cos(theta))
        self.y += dt * (self.v_x * math.sin(self.theta) + self.v_y * math.cos(self.theta))
        
        # theta_k+1 = theta_k + dt * omega
        self.theta += dt * omega_z

        # Préparation du message Odometry [cite: 68]
        odom_msg = Odometry()
        
        # Header (Stamp et Frame ID) [cite: 70, 71]
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom' # [cite: 78]
        odom_msg.child_frame_id = 'base_link' # Convention standard

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (Conversion Yaw -> Quaternion) [cite: 81]
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        # Publication
        self.pub_odom.publish(odom_msg)

    def quaternion_from_yaw(self, yaw):
        # Conversion manuelle d'un angle lacet (yaw) en Quaternion [cite: 79]
        # q = (x, y, z, w) = (0, 0, sin(yaw/2), cos(yaw/2)) pour une rotation autour de Z
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()