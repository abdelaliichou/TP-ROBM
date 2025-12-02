from typing import Optional
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

import zenoh
from zenoh import Reliability, Sample, Priority, CongestionControl

import json
import os

from .bridge_node import BridgeNode

class ZenohBridge(BridgeNode):
    def __init__(self, node_name: str = 'zenoh_bridge'):
        super().__init__(node_name)

        self.zenoh_session: Optional[zenoh.Session] = None

        self.declare_parameter('node_id', '')
        self.declare_parameter('use_robot_wifi', False)
        self.declare_parameter('connect_endpoint', 'tcp/192.168.0.10:7447')  # Raspi ROBM_0'

        # Check if robot name was provided
        robot_name = self.get_parameter('node_id').get_parameter_value().string_value.strip()
        if not robot_name:
            self.get_logger().fatal("No robot name provided. Please set the 'node_id' parameter.")
            exit(1)
        # Check robot name validity.
        if not robot_name.isascii() and robot_name.isidentifier():
            self.get_logger().fatal(f"Invalid robot name provided: {robot_name}. Please set the 'node_id' parameter.")
            exit(1)
        self.get_logger().info(f"Robot name: {robot_name}")

        # Zenoh stuff
        zenoh_conf = zenoh.Config()

        if self.get_parameter('use_robot_wifi').get_parameter_value().bool_value:
            # P2P connexion with the robot in WiFi AP mode (UDP multicast)
            self.get_logger().info(f"WiFi link: use Robot WiFi access point (ROBM_{self.get_parameter('node_id').get_parameter_value().string_value})")
            zenoh_conf.insert_json5("mode", json.dumps('peer'))
            ZENOH_CONNECTIONS = ['udp/224.0.1.226:7447'] # #iface=en0
            zenoh_conf.insert_json5("listen/endpoints", json.dumps(ZENOH_CONNECTIONS))
            zenoh_conf.insert_json5("transport/link/tx/batch_size", json.dumps(2048))
        else:
            # Client connexion to the zenoh router in WiFi client mode (TCP)
            self.get_logger().info("Wifi link: use ROBM_0_5G router")
            zenoh_conf.insert_json5("mode", json.dumps('client'))
            ZENOH_CONNECTIONS = [
                #'tcp/148.60.11.66:8080',  # VM Istic
                #'tcp/192.168.0.10:7447',  # Raspi ROBM_0
                self.get_parameter('connect_endpoint').get_parameter_value().string_value
                ]
            zenoh_conf.insert_json5("connect/endpoints", json.dumps(ZENOH_CONNECTIONS))

        self.get_logger().info("Opening Zenoh session...")

        try:
            self.zenoh_session = zenoh.open(zenoh_conf)
        except zenoh.ZError as e:
            self.get_logger().fatal(f"Failed to open Zenoh session: {e}")
            exit(1)

        zenoh_key_root = 'robm/robots/' + robot_name
        zenoh_key_sensors = zenoh_key_root + '/sensors'
        zenoh_key_effectors = zenoh_key_root + '/effectors'

        self.get_logger().info(f"Declaring Zenoh subscriber on '{zenoh_key_sensors}'...")
        self.zenoh_session.declare_subscriber(zenoh_key_sensors, self.zenoh_listener)
        self.get_logger().info(f"Declaring Zenoh subscriber on '{zenoh_key_effectors}'...")
        self.zenoh_pub = self.zenoh_session.declare_publisher(zenoh_key_effectors, encoding=zenoh.Encoding('application/msgpack'))

    def __del__(self):
        # Cleanup: note that even if you forget it, cleanup will happen automatically when 
        # the reference counter reaches 0
        if self.zenoh_session:
            self.get_logger().info("Closing Zenoh session...")
            self.zenoh_session.close()


    def zenoh_listener(self, sample: Sample):
        self.data_listener(sample.payload.to_bytes())

    
    def _send_command_data(self, payload: bytes):
        if payload:
            self.get_logger().debug(f"Publishing to '{self.zenoh_pub.key_expr}': {payload}")
            self.zenoh_pub.put(payload)


def main(args=None):
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') is not None:
        if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
            print("ERROS: 'ROS_AUTOMATIC_DISCOVERY_RANGE' is not set to 'LOCALHOST'. Please run 'export ROS_LOCALHOST_ONLY=1' and 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' before running this node.")
            exit(1)
    elif os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        print("ERROR: ROS_LOCALHOST_ONLY is not set to 1. Please run 'export ROS_LOCALHOST_ONLY=1' and 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' before running this node.")
        exit(1)
    
    rclpy.init(args=args)

    zenoh_bridge = ZenohBridge()

    rclpy.spin(zenoh_bridge)

    # zenoh_bridge.zenoh_session.close()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    # zenoh_bridge.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
