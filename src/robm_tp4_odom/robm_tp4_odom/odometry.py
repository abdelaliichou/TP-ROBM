#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
# On importe TwistStamped au lieu de Twist pour être compatible avec TP5
from geometry_msgs.msg import TwistStamped, Quaternion 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # 1. Souscription à la commande de vitesse
        # MODIFICATION : On écoute TwistStamped car c'est ce que 'move' et le bridge envoient
        self.sub_cmd = self.create_subscription(
            TwistStamped, 'cmd_vel', self.cmd_callback, 10)
        
        # 2. Souscription à l'IMU calibrée
        self.sub_imu = self.create_subscription(
            Imu, 'calibrated_imu', self.imu_callback, 10)
        
        # 3. Publisher pour l'odométrie calculée
        self.pub_odom = self.create_publisher(Odometry, 'odometry', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        
        self.last_time = None

    def cmd_callback(self, msg: TwistStamped):
        # MODIFICATION : Avec TwistStamped, la vitesse est dans msg.twist.linear
        self.v_x = msg.twist.linear.x
        self.v_y = msg.twist.linear.y

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return 

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        omega_z = msg.angular_velocity.z

        # Intégration d'Euler
        self.x += dt * (self.v_x * math.cos(self.theta) - self.v_y * math.sin(self.theta))
        self.y += dt * (self.v_x * math.sin(self.theta) + self.v_y * math.cos(self.theta))
        self.theta += dt * omega_z

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        self.pub_odom.publish(odom_msg)

    def quaternion_from_yaw(self, yaw):
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