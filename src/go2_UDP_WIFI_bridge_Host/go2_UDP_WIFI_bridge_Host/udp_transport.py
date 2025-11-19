from __future__ import annotations

import json
import struct
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class Datagram:
    header: Dict[str, object]
    payload: bytes


class Packetizer:
    """Encode/decode ROS message payloads into UDP datagrams."""

    def __init__(self, max_datagram_size: int = 60000) -> None:
        if max_datagram_size <= 512:
            raise ValueError("max_datagram_size must be greater than 512 bytes")
        self.max_datagram_size = max_datagram_size
        self.max_payload = max_datagram_size - 256

    def encode(self, topic: str, msg_type: str, payload: bytes) -> List[bytes]:
        total_len = len(payload)
        if total_len == 0:
            total_len = 1
            payload = b"\0"
        chunk_size = max(1, min(self.max_payload, total_len))
        total_chunks = (len(payload) + chunk_size - 1) // chunk_size
        message_id = uuid.uuid4().hex

        datagrams: List[bytes] = []
        for chunk_index in range(total_chunks):
            start = chunk_index * chunk_size
            end = min(len(payload), start + chunk_size)
            chunk = payload[start:end]
            header = {
                "topic": topic,
                "type": msg_type,
                "msg_id": message_id,
                "chunk": chunk_index,
                "total": total_chunks,
                "payload_size": len(chunk),
                "timestamp": time.time(),
            }
            header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
            if len(header_bytes) >= 65535:
                raise ValueError("Header too large for UDP datagram")
            prefix = struct.pack("!H", len(header_bytes))
            datagrams.append(prefix + header_bytes + chunk)
        return datagrams

    @staticmethod
    def decode(datagram: bytes) -> Datagram:
        if len(datagram) < 2:
            raise ValueError("Datagram too short")
        header_len = struct.unpack("!H", datagram[:2])[0]
        header_end = 2 + header_len
        if header_end > len(datagram):
            raise ValueError("Invalid header length")
        header_bytes = datagram[2:header_end]
        payload = datagram[header_end:]
        header = json.loads(header_bytes.decode("utf-8"))
        expected_size = int(header.get("payload_size", len(payload)))
        if expected_size != len(payload):
            raise ValueError("Payload size mismatch")
        return Datagram(header=header, payload=payload)


class ReassemblyBuffer:
    """Collects UDP chunks until a full message is available."""

    def __init__(self, timeout_sec: float = 2.0):
        self.timeout = timeout_sec
        self._lock = threading.Lock()
        self._messages: Dict[Tuple[str, str], Dict[str, object]] = {}

    def add_chunk(self, header: Dict[str, object], payload: bytes):
        key = (str(header["msg_id"]), str(header["topic"]))
        total = int(header.get("total", 1))
        chunk_idx = int(header.get("chunk", 0))
        now = time.time()

        if total == 1:
            return True, payload

        with self._lock:
            entry = self._messages.setdefault(
                key,
                {
                    "total": total,
                    "received": 0,
                    "chunks": {},
                    "created": now,
                    "type": header.get("type"),
                },
            )
            entry["chunks"][chunk_idx] = payload
            entry["received"] = len(entry["chunks"])
            entry["created"] = now

            if entry["received"] >= entry["total"]:
                ordered = [entry["chunks"][i] for i in sorted(entry["chunks"].keys())]
                data = b"".join(ordered)
                del self._messages[key]
                return True, data

            self._purge_expired(now)
            return False, b""

    def _purge_expired(self, now: float) -> None:
        expired = [key for key, entry in self._messages.items() if now - entry["created"] > self.timeout]
        for key in expired:
            del self._messages[key]
