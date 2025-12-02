from typing import Callable, Optional
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from .bridge_node import BridgeNode
from socket import socket, AF_INET, SOCK_DGRAM
from select import select

import threading

try:
    from signal import signal, SIGPIPE, SIG_DFL, SIG_IGN
    # signal(SIGPIPE,SIG_DFL)
    signal(SIGPIPE,SIG_IGN)
except ImportError:  # If SIGPIPE is not available (windows),
    pass             # ignore it.

import logging, time
logger = logging.getLogger("udp_bridge")
logging.basicConfig(level=logging.INFO)
logger.setLevel(logging.DEBUG)


# class UdpClient:
#     def __init__(self, local_port: int, remote_ip: str, remote_port: int):
#         self.sock = socket(AF_INET, SOCK_DGRAM)
#         # self.sock.bind(('', local_port))
#         self.remote_addr = (remote_ip, remote_port)
#         self.sock.setblocking(False)
#         self._data_callback : Optional[Callable[[bytes], None]] = None
#         self._thread = None
    
#     def send(self, data: bytes):
#         self.sock.sendto(data, self.remote_addr)

#     def receive(self) -> Optional[bytes]:
#         try:
#             data, addr = self.sock.recvfrom(4096)
#             return data
#         except BlockingIOError:
#             return None

#     def close(self):
#         if self._thread is not None:
#             self._thread.join(timeout=0.5)
#             self._thread = None
#         self.sock.close()

#     def __del__(self):
#         self.close()

#     def start_listening(self):
#         """ Start a background thread to listen for incoming UDP messages. """
#         if self._thread is not None:
#             return  # Already started

#         def listen():
#             while True:
#                 data = self.receive()
#                 if data is not None:
#                     if self._data_callback:
#                         self._data_callback(data)
#                     rclpy.logging.get_logger('udp_client').info(f"Received UDP data: {data}")

#         self._thread = threading.Thread(target=listen, daemon=True)
#         self._thread.start()


class UdpClient:
    def __init__(self, remote_ip: str, remote_port: int, local_port: int = 0, local_ip: str = ''):
        self.sock = socket(AF_INET, SOCK_DGRAM)
        if local_port != 0 or local_ip != '':
            self.sock.bind((local_ip, local_port))
        self.remote_addr = (remote_ip, remote_port)
        self.sock.setblocking(False)
        self._data_callback : Optional[Callable[[bytes], None]] = None
        self._thread = None
        self._listening = False
    
    def send(self, data: bytes):
        if self.sock.fileno() == -1:
            logger.error("UDP socket is closed. Cannot send data.")
            return
        self.sock.sendto(data, self.remote_addr)
        # self.sock.send(data)

    def receive(self) -> Optional[bytes]:
        try:
            data, addr = self.sock.recvfrom(4096)
            # data = self.sock.recv(4096)
            return data
        except BlockingIOError:
            return None
        except ConnectionError:
            return None

    def close(self):
        if self._thread is not None:
            self._listening = False
            self._thread.join(timeout=0.5)
            self._thread = None
        self.sock.close()

    def __del__(self):
        self.close()

    def start_listening(self):
        # return
        """ Start a background thread to listen for incoming UDP messages. """
        if self._thread is not None:
            return  # Already started
        
        # self.sock.connect(self.remote_addr)

        try:
            logger.debug(f"Sending hello to UDP connection to {self.remote_addr}...")
            self.sock.sendto(b"hello", self.remote_addr)
        except BrokenPipeError as e:
            logger.error(f"Failed to connect to UDP server at {self.remote_addr}: {e}")

        class Accum:
            def __init__(self) -> None:
                self.sum = 0.0
                self.sum2 = 0.0
                self.n = 0
                self.max = -9e99
                self.min = 9e99

            def update(self, val: float) -> None:
                self.sum += val
                self.sum2 += val ** 2
                self.n += 1
                self.min = min(self.min, val)
                self.max = max(self.max, val)

            def mean(self) -> float:
                return self.sum / self.n
            
            def std(self) -> float:
                return ((self.sum2 / self.n) - self.mean() ** 2) ** 0.5

        def listen():
            last_time = 0
            last_activity_time = 0
            acc = Accum()
            while self._listening:
                if self.sock.fileno() == -1:
                    logger.warning("UDP socket is closed. Trying to reconnect...")
                    time.sleep(1.0)
                    self.sock = socket(AF_INET, SOCK_DGRAM)
                    self.sock.setblocking(False)
                    last_activity_time = 0
                    continue
                if last_activity_time + 2.0 < time.perf_counter():
                    logger.debug(f"Sending hello to UDP connection to {self.remote_addr}...")
                    last_activity_time = time.perf_counter()
                    try:
                       self.sock.sendto(b"hello", self.remote_addr)
                    except Exception as e:
                        logger.error(f"Failed to send hello to UDP server at {self.remote_addr}: {e}")
                try:
                    ready_to_read, ready_to_write, in_error = select([self.sock], [], [], 0.5)
                    data = self.receive()
                    now = time.perf_counter_ns()
                    if data is not None:
                        # logger.debug(f"Received UDP data ({len(data)}B): {data}")
                        if last_time:
                            dt = now - last_time
                            acc.update(dt*1e-6)
                            logger.debug(f"dt = {dt*1e-6:5.1f} ms ({acc.mean():5.1f} +/-{acc.std():5.2f}, [{acc.min:5.1f}, {acc.max:5.1f}], {acc.n} samples)")
                        last_time = now
                        last_activity_time = time.perf_counter()
                        if self._data_callback:
                            self._data_callback(data)
                except Exception as e:
                    logger.error(f"Error while receiving UDP data: {e}")
                    # Do not close the socket here; just wait and try again
                    time.sleep(0.1)

        self._listening = True
        self._thread = threading.Thread(target=listen, daemon=True)
        self._thread.start()


class UdpBridge(BridgeNode):
    def __init__(self):
        super().__init__('udp_bridge')

        self.declare_parameter('robot_ip', '192.168.4.1')
        self.declare_parameter('robot_port', 7448)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value.strip()
        robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value

        # UDP client
        self.get_logger().info(f"Starting UDP client, connecting to {robot_ip}:{robot_port}...")
        self.udp_client = UdpClient(remote_ip=robot_ip, remote_port=robot_port)
        self.udp_client._data_callback = self.data_listener
        self.udp_client.start_listening()
        self.get_logger().info("UDP client listening")

    def __del__(self):
        # Cleanup: note that even if you forget it, cleanup will happen automatically when 
        # the reference counter reaches 0
        self.udp_client.close()

    def _send_command_data(self, payload: bytes):
        self.udp_client.send(payload)



def main(args=None):
    # if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    #     print("ERROR: ROS_LOCALHOST_ONLY is not set to 1. Please run 'export ROS_LOCALHOST_ONLY=1' before running this node.")
    #     exit(1)
    
    rclpy.init(args=args)

    udp_bridge = UdpBridge()

    rclpy.spin(udp_bridge)

    # udp_bridge.zenoh_session.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    # udp_bridge.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
