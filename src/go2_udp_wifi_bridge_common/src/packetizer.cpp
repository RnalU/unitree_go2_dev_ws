#include "go2_udp_wifi_bridge/packetizer.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace go2_udp_wifi_bridge {

namespace {
#pragma pack(push, 1)
struct PacketHeader {
  uint16_t topic_len;
  uint16_t type_len;
  uint32_t payload_size;
  uint32_t chunk_index;
  uint32_t total_chunks;
  uint64_t message_id;
};
#pragma pack(pop)

std::atomic<uint64_t> g_counter{0};

inline uint64_t host_to_be64(uint64_t value) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

  const uint32_t high = static_cast<uint32_t>(value >> 32);
  const uint32_t low = static_cast<uint32_t>(value & 0xFFFFFFFFULL);
  return (static_cast<uint64_t>(htonl(low)) << 32) | htonl(high);
#else
  return value;
#endif
}

inline uint64_t be64_to_host(uint64_t value) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  const uint32_t high = ntohl(static_cast<uint32_t>(value & 0xFFFFFFFFULL));
  const uint32_t low = ntohl(static_cast<uint32_t>(value >> 32));
  return (static_cast<uint64_t>(high) << 32) | low;
#else
  return value;
#endif
}
}  // namespace

Packetizer::Packetizer(size_t max_datagram_size)
    : max_datagram_size_(max_datagram_size) {
  if (max_datagram_size_ < sizeof(PacketHeader) + 16) {
    throw std::runtime_error("max_datagram_size too small for header");
  }
  max_payload_size_ = max_datagram_size_ - sizeof(PacketHeader) - 32;  // reserve for topic/type strings
}

std::vector<std::vector<uint8_t>> Packetizer::encode(const std::string &topic, const std::string &msg_type,
                                                     const uint8_t *data, size_t size) const {
  if (topic.size() > UINT16_MAX || msg_type.size() > UINT16_MAX) {
    throw std::runtime_error("Topic or type string too long");
  }

  const size_t header_overhead = sizeof(PacketHeader) + topic.size() + msg_type.size();
  if (header_overhead >= max_datagram_size_) {
    throw std::runtime_error("Topic/type header exceeds datagram size budget");
  }

  const size_t chunk_payload = max_datagram_size_ - header_overhead;
  const size_t total_chunks = size == 0 ? 1 : (size + chunk_payload - 1) / chunk_payload;
  const uint64_t message_id = ++g_counter;

  std::vector<std::vector<uint8_t>> datagrams;
  datagrams.reserve(total_chunks);

  for (size_t chunk = 0; chunk < total_chunks; ++chunk) {
    const size_t start = chunk * chunk_payload;
    const size_t end = size == 0 ? 0 : std::min(size, start + chunk_payload);
    const size_t payload_size = (size == 0) ? 0 : (end - start);

    std::vector<uint8_t> buffer(sizeof(PacketHeader) + topic.size() + msg_type.size() + payload_size);
    PacketHeader header{};
    header.topic_len = htons(static_cast<uint16_t>(topic.size()));
    header.type_len = htons(static_cast<uint16_t>(msg_type.size()));
    header.payload_size = htonl(static_cast<uint32_t>(payload_size));
    header.chunk_index = htonl(static_cast<uint32_t>(chunk));
    header.total_chunks = htonl(static_cast<uint32_t>(total_chunks));
    header.message_id = host_to_be64(message_id);
    std::memcpy(buffer.data(), &header, sizeof(PacketHeader));

    size_t offset = sizeof(PacketHeader);
    std::memcpy(buffer.data() + offset, topic.data(), topic.size());
    offset += topic.size();
    std::memcpy(buffer.data() + offset, msg_type.data(), msg_type.size());
    offset += msg_type.size();
    if (payload_size > 0 && data != nullptr) {
      std::memcpy(buffer.data() + offset, data + start, payload_size);
    }

    datagrams.emplace_back(std::move(buffer));
  }

  if (size == 0) {
    // send empty payload chunk to signal empty message
    PacketHeader header{};
    header.topic_len = htons(static_cast<uint16_t>(topic.size()));
    header.type_len = htons(static_cast<uint16_t>(msg_type.size()));
    header.payload_size = 0;
    header.chunk_index = 0;
    header.total_chunks = htonl(1);
    header.message_id = host_to_be64(message_id);
    std::vector<uint8_t> buffer(sizeof(PacketHeader) + topic.size() + msg_type.size());
    std::memcpy(buffer.data(), &header, sizeof(PacketHeader));
    size_t offset = sizeof(PacketHeader);
    std::memcpy(buffer.data() + offset, topic.data(), topic.size());
    offset += topic.size();
    std::memcpy(buffer.data() + offset, msg_type.data(), msg_type.size());
    datagrams.clear();
    datagrams.emplace_back(std::move(buffer));
  }

  return datagrams;
}

Datagram Packetizer::decode(const uint8_t *data, size_t size) const {
  if (size < sizeof(PacketHeader)) {
    throw std::runtime_error("Datagram too small for header");
  }

  PacketHeader header{};
  std::memcpy(&header, data, sizeof(PacketHeader));
  const uint16_t topic_len = ntohs(header.topic_len);
  const uint16_t type_len = ntohs(header.type_len);
  const uint32_t payload_size = ntohl(header.payload_size);
  const uint32_t chunk_index = ntohl(header.chunk_index);
  const uint32_t total_chunks = ntohl(header.total_chunks);
  const uint64_t message_id = be64_to_host(header.message_id);

  const size_t required = sizeof(PacketHeader) + topic_len + type_len;
  if (size < required || size < required + payload_size) {
    throw std::runtime_error("Datagram payload truncated");
  }

  Datagram datagram;
  datagram.topic.assign(reinterpret_cast<const char *>(data + sizeof(PacketHeader)), topic_len);
  datagram.type.assign(reinterpret_cast<const char *>(data + sizeof(PacketHeader) + topic_len), type_len);
  datagram.message_id = std::to_string(message_id);
  datagram.chunk_index = chunk_index;
  datagram.total_chunks = total_chunks;
  datagram.payload.resize(payload_size);
  if (payload_size > 0) {
    std::memcpy(datagram.payload.data(), data + required, payload_size);
  }
  return datagram;
}

ReassemblyBuffer::ReassemblyBuffer(double timeout_sec)
    : timeout_(timeout_sec) {}

bool ReassemblyBuffer::add_chunk(const Datagram &datagram, std::vector<uint8_t> &output) {
  if (datagram.total_chunks <= 1) {
    output = datagram.payload;
    return true;
  }

  const std::string key = datagram.topic + ":" + datagram.message_id;
  const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

  std::lock_guard<std::mutex> lock(mutex_);
  auto &entry = messages_[key];
  if (entry.chunks.empty()) {
    entry.total_chunks = datagram.total_chunks;
    entry.chunks.resize(datagram.total_chunks);
  }
  entry.last_update = now;
  if (datagram.chunk_index < entry.chunks.size()) {
    entry.chunks[datagram.chunk_index] = datagram.payload;
  }

  size_t filled = 0;
  for (const auto &chunk : entry.chunks) {
    if (!chunk.empty()) {
      ++filled;
    }
  }

  if (filled >= entry.total_chunks) {
    size_t total_size = 0;
    for (const auto &chunk : entry.chunks) {
      total_size += chunk.size();
    }
    output.clear();
    output.reserve(total_size);
    for (const auto &chunk : entry.chunks) {
      output.insert(output.end(), chunk.begin(), chunk.end());
    }
    messages_.erase(key);
    return true;
  }

  purge_expired(now);
  return false;
}

void ReassemblyBuffer::purge_expired(double now) {
  for (auto it = messages_.begin(); it != messages_.end();) {
    if (now - it->second.last_update > timeout_) {
      it = messages_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace go2_udp_wifi_bridge
