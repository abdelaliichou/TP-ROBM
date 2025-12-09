#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class CalibIMUNode(Node):
    def __init__(self):
        super().__init__('calib_imu')
        
        # Subscriber : IMU non calibrée
        self.sub_imu = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        
        # Publisher : IMU calibrée
        self.pub_imu = self.create_publisher(Imu, 'calibrated_imu', 10)
        
        # Stockage pour calculer le biais
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []
        self.bias_computed = False
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0

    def imu_callback(self, msg: Imu):
        if not self.bias_computed:
            # Accumuler les 20 premières mesures
            self.gyro_x.append(msg.angular_velocity.x)
            self.gyro_y.append(msg.angular_velocity.y)
            self.gyro_z.append(msg.angular_velocity.z)
            
            if len(self.gyro_x) >= 20:
                # Calcul du biais
                self.bias_x = sum(self.gyro_x)/len(self.gyro_x)
                self.bias_y = sum(self.gyro_y)/len(self.gyro_y)
                self.bias_z = sum(self.gyro_z)/len(self.gyro_z)
                self.bias_computed = True
                self.get_logger().info(f"Bias computed: x={self.bias_x:.5f}, y={self.bias_y:.5f}, z={self.bias_z:.5f}")
            return  # Ne publie rien pendant le calcul du biais

        # Après calcul du biais, soustraire et republier
        calibrated_msg = Imu()
        calibrated_msg.header = msg.header
        calibrated_msg.orientation = msg.orientation
        calibrated_msg.orientation_covariance = msg.orientation_covariance
        
        calibrated_msg.angular_velocity.x = msg.angular_velocity.x - self.bias_x
        calibrated_msg.angular_velocity.y = msg.angular_velocity.y - self.bias_y
        calibrated_msg.angular_velocity.z = msg.angular_velocity.z - self.bias_z
        calibrated_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        calibrated_msg.linear_acceleration = msg.linear_acceleration
        calibrated_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Publier le message corrigé
        self.pub_imu.publish(calibrated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CalibIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
