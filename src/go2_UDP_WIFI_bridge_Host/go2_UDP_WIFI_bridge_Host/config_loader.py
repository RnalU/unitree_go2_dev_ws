from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List

import yaml

SEND = "send"
RECEIVE = "receive"


@dataclass
class TransportConfig:
    local_ip: str = "0.0.0.0"
    default_remote_ip: str = ""
    default_remote_port: int = 55000
    default_listen_port: int = 56000
    max_datagram_size: int = 60000
    queue_poll_interval: float = 0.01


@dataclass
class StreamConfig:
    name: str
    direction: str
    local_topic: str
    msg_type: str
    remote_topic: str
    remote_ip: str
    remote_port: int
    listen_port: int
    qos_depth: int = 10


@dataclass
class BridgeConfig:
    transport: TransportConfig
    streams: List[StreamConfig]

    @classmethod
    def load(cls, path: str) -> "BridgeConfig":
        if not os.path.exists(path):
            raise FileNotFoundError(f"Bridge config file not found: {path}")

        with open(path, "r", encoding="utf-8") as file:
            raw = yaml.safe_load(file) or {}

        transport_data = raw.get("transport", {}) or {}
        transport = TransportConfig(
            local_ip=str(transport_data.get("local_ip", "0.0.0.0")),
            default_remote_ip=str(transport_data.get("default_remote_ip", "")),
            default_remote_port=int(transport_data.get("default_remote_port", 55000)),
            default_listen_port=int(transport_data.get("default_listen_port", transport_data.get("listen_port", 56000))),
            max_datagram_size=int(transport_data.get("max_datagram_size", 60000)),
            queue_poll_interval=float(transport_data.get("queue_poll_interval", 0.01)),
        )

        streams_data = raw.get("streams", []) or []
        if not streams_data:
            raise ValueError("Bridge config must contain at least one stream entry")

        streams: List[StreamConfig] = []
        for entry in streams_data:
            direction = str(entry.get("direction", "")).strip().lower()
            if direction not in {SEND, RECEIVE}:
                raise ValueError(
                    f"Stream '{entry.get('name')}' has invalid direction '{direction}'. Use 'send' or 'receive'."
                )

            local_topic = entry.get("local_topic")
            msg_type = entry.get("msg_type")
            if not local_topic or not msg_type:
                raise ValueError(f"Stream '{entry.get('name')}' must define local_topic and msg_type")

            stream = StreamConfig(
                name=str(entry.get("name", local_topic)),
                direction=direction,
                local_topic=str(local_topic),
                msg_type=str(msg_type),
                remote_topic=str(entry.get("remote_topic", local_topic)),
                remote_ip=str(entry.get("remote_ip", transport.default_remote_ip)),
                remote_port=int(entry.get("remote_port", transport.default_remote_port)),
                listen_port=int(entry.get("listen_port", transport.default_listen_port)),
                qos_depth=int(entry.get("qos_depth", 10)),
            )
            streams.append(stream)

        return cls(transport=transport, streams=streams)
