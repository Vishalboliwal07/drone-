#pragma once

#include <cstdint>

// Keep this layout synchronized between the transmitter and receiver.
// The explicit reserved bytes preserve the original 28-byte ESP-NOW packet.
struct CommandPacket {
  int32_t throttle = 0;
  int32_t yaw = 0;
  int32_t pitch = 0;
  int32_t roll = 0;
  uint8_t armed = 0;
  uint8_t reserved[3] = {};
  int32_t flightMode = 0;
  int32_t yawBias = 0;
};

static_assert(sizeof(CommandPacket) == 28, "ESP-NOW packet layout changed");
