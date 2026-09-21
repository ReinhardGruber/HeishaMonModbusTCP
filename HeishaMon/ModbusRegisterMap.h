#pragma once

#include <stdint.h>

// Map v2: never derive block bases from the number of currently known topics.
// Topic numbers and command IDs are permanent; append new IDs, never reuse them.
namespace ModbusMap {
constexpr uint16_t VERSION = 2;
constexpr uint16_t TOPIC_CAPACITY = 1000;
constexpr uint16_t MAIN_TOPIC_BASE = 0;
constexpr uint16_t EXTRA_TOPIC_BASE = 1000;
constexpr uint16_t OPTIONAL_TOPIC_BASE = 2000;
constexpr uint16_t S0_TOPIC_BASE = 3000;
constexpr uint16_t S0_PORT_STRIDE = 100;
constexpr uint16_t S0_FIELD_COUNT = 6;
constexpr uint16_t VERSION_REGISTER = 9000;
constexpr uint16_t FLOAT_BASE = 10000;
constexpr uint16_t COMMAND_BASE = 20000;
constexpr uint16_t OPTIONAL_COMMAND_BASE = 21000;
constexpr uint16_t SYSTEM_COMMAND_BASE = 22000;
constexpr uint16_t RESET_COMMAND_ID = 100;
constexpr uint16_t RELAY_COUNT = 2;

constexpr uint16_t floatAddress(uint16_t integerAddress) {
  return FLOAT_BASE + 2 * integerAddress;
}

constexpr uint16_t commandAddress(uint16_t id) {
  return id == RESET_COMMAND_ID ? SYSTEM_COMMAND_BASE : COMMAND_BASE + id - 1;
}

// Only populated entries are valid. Reserved space must not alias another block.
inline bool decodeRange(uint16_t address, uint16_t base, uint16_t count,
                        uint16_t stride, uint16_t &index, bool &highWord) {
  if ((stride != 1 && stride != 2) || count > TOPIC_CAPACITY || address < base ||
      uint32_t(address) >= uint32_t(base) + uint32_t(count) * stride) {
    return false;
  }
  const uint16_t offset = address - base;
  index = offset / stride;
  highWord = (offset % stride) == 0;
  return true;
}

struct OptionalCommand {
  uint16_t id;
  const char *name;
};

// Explicit IDs keep addresses stable even if upstream reorders its command table.
constexpr OptionalCommand OPTIONAL_COMMANDS[] = {
  {0, "SetHeatCoolMode"},
  {1, "SetCompressorState"},
  {2, "SetSmartGridMode"},
  {3, "SetExternalThermostat1State"},
  {4, "SetExternalThermostat2State"},
  {5, "SetDemandControl"},
  {6, "SetPoolTemp"},
  {7, "SetBufferTemp"},
  {8, "SetZ1RoomTemp"},
  {9, "SetZ1WaterTemp"},
  {10, "SetZ2RoomTemp"},
  {11, "SetZ2WaterTemp"},
  {12, "SetSolarTemp"},
  {13, "SetOptPCBByte9"},
};
}  // namespace ModbusMap
