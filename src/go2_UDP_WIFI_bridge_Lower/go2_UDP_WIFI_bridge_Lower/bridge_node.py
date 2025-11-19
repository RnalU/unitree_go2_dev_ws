from __future__ import annotations

import os
import queue
import socket
import threading
import time
from typing import Dict, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from .config_loader import BridgeConfig, RECEIVE, SEND
from .udp_transport import Packetizer, ReassemblyBuffer


class UdpListener(threading.Thread):
    def __init__(self, local_ip: str, port: int, callback):
        super().__init__(daemon=True)
        self._callback = callback
        self._stop_event = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((local_ip, port))
        self._socket.settimeout(1.0)
        self.port = port

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, addr = self._socket.recvfrom(65535)
                self._callback(data, addr, self.port)
            except socket.timeout:
                continue
            except OSError:
                break

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._socket.close()
        except OSError:
            pass


class LowerBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('go2_udp_bridge_lower')

        default_config = os.path.join(
            get_package_share_directory('go2_UDP_WIFI_bridge_Lower'),
            'config',
            'lower_bridge.example.yaml'
        )
        self.declare_parameter('config_file', default_config)
        config_path = self.get_parameter('config_file').get_parameter_value().string_value

        self.config = BridgeConfig.load(config_path)
        self.packetizer = Packetizer(self.config.transport.max_datagram_size)
        self.reassembly = ReassemblyBuffer()
        self.queue: 'queue.Queue[Tuple[dict, bytes]]' = queue.Queue()
        self.tx_sockets: Dict[Tuple[str, int], socket.socket] = {}
        self.rx_publishers: Dict[str, object] = {}
        self.rx_streams_by_topic: Dict[str, object] = {}
        self.listeners: Dict[int, UdpListener] = {}

        self.get_logger().info(f"Loaded {len(self.config.streams)} stream definitions from {config_path}")

        self._setup_streams()
        poll = float(self.config.transport.queue_poll_interval)
        self.timer = self.create_timer(max(poll, 0.005), self._process_incoming_queue)

    def _setup_streams(self) -> None:
        for stream in self.config.streams:
            msg_cls = get_message(stream.msg_type)
            qos = QoSProfile(depth=stream.qos_depth)
            if stream.direction == SEND:
                self.create_subscription(
                    msg_cls,
                    stream.local_topic,
                    lambda msg, s=stream: self._handle_outbound(s, msg),
                    qos
                )
                self.get_logger().info(
                    f"Forwarding {stream.local_topic} -> {stream.remote_ip}:{stream.remote_port} as {stream.remote_topic}"
                )
            else:
                publisher = self.create_publisher(msg_cls, stream.local_topic, qos)
                self.rx_publishers[stream.local_topic] = publisher
                self.rx_streams_by_topic[stream.remote_topic] = {
                    'publisher': publisher,
                    'msg_cls': msg_cls,
                }
                self._ensure_listener(stream.listen_port)
                self.get_logger().info(
                    f"Listening on port {stream.listen_port} for topic {stream.remote_topic} -> {stream.local_topic}"
                )

    def _handle_outbound(self, stream, msg) -> None:
        try:
            payload = serialize_message(msg)
            datagrams = self.packetizer.encode(stream.remote_topic, stream.msg_type, payload)
            for datagram in datagrams:
                self._send_datagram(stream.remote_ip, stream.remote_port, datagram)
        except Exception as exc:
            self.get_logger().error(f"Failed to send {stream.name}: {exc}")

    def _send_datagram(self, ip: str, port: int, data: bytes) -> None:
        key = (ip, port)
        sock = self.tx_sockets.get(key)
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.tx_sockets[key] = sock
        try:
            sock.sendto(data, key)
        except OSError as exc:
            self.get_logger().error(f"UDP send error to {ip}:{port} - {exc}")

    def _ensure_listener(self, port: int) -> None:
        if port in self.listeners:
            return
        listener = UdpListener(self.config.transport.local_ip, port, self._on_datagram)
        self.listeners[port] = listener
        listener.start()

    def _on_datagram(self, data: bytes, addr, port: int) -> None:
        try:
            datagram = self.packetizer.decode(data)
            self.queue.put(datagram)
        except Exception as exc:
            self.get_logger().warning(f"Dropping UDP packet on port {port}: {exc}")

    def _process_incoming_queue(self) -> None:
        processed = 0
        while not self.queue.empty() and processed < 128:
            datagram = self.queue.get()
            header = datagram.header
            payload = datagram.payload
            ready, joined = self.reassembly.add_chunk(header, payload)
            if not ready:
                continue
            topic = header.get('topic')
            stream = self.rx_streams_by_topic.get(topic)
            if not stream:
                self.get_logger().debug(f"Received topic {topic} with no matching stream")
                continue
            try:
                msg = deserialize_message(joined, stream['msg_cls'])
                stream['publisher'].publish(msg)
            except Exception as exc:
                self.get_logger().error(f"Failed to publish message for {topic}: {exc}")
            processed += 1

    def destroy_node(self) -> None:
        for listener in self.listeners.values():
            listener.stop()
        for sock in self.tx_sockets.values():
            try:
                sock.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
