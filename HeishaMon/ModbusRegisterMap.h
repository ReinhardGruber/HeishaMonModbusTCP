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

// Modbus IDs are kept here, separate from the command tables in commands.h, so the
// command parsing code does not need to know about Modbus. Commands are matched by
// name, which keeps addresses stable even if the command table is reordered.
struct MainCommand {
  uint16_t id;
  const char *name;
};

constexpr MainCommand MAIN_COMMANDS[] = {
  {1, "SetHeatpump"},
  {2, "SetHolidayMode"},
  {3, "SetQuietMode"},
  {4, "SetPowerfulMode"},
  {5, "SetZ1HeatRequestTemperature"},
  {6, "SetZ1CoolRequestTemperature"},
  {7, "SetZ2HeatRequestTemperature"},
  {8, "SetZ2CoolRequestTemperature"},
  {9, "SetOperationMode"},
  {10, "SetForceDHW"},
  {11, "SetDHWTemp"},
  {12, "SetForceDefrost"},
  {13, "SetForceSterilization"},
  {14, "SetPump"},
  {15, "SetMaxPumpDuty"},
  {16, "SetCurves"},
  {17, "SetZones"},
  {18, "SetFloorHeatDelta"},
  {19, "SetFloorCoolDelta"},
  {20, "SetDHWHeatDelta"},
  {21, "SetHeaterDelayTime"},
  {22, "SetHeaterStartDelta"},
  {23, "SetHeaterStopDelta"},
  {24, "SetMainSchedule"},
  {25, "SetAltExternalSensor"},
  {26, "SetExternalPadHeater"},
  {27, "SetBufferDelta"},
  {28, "SetBuffer"},
  {29, "SetHeatingOffOutdoorTemp"},
  {30, "SetExternalControl"},
  {31, "SetExternalError"},
  {32, "SetExternalCompressorControl"},
  {33, "SetExternalHeatCoolControl"},
  {34, "SetBivalentControl"},
  {35, "SetBivalentMode"},
  {36, "SetBivalentStartTemp"},
  {37, "SetBivalentAPStartTemp"},
  {38, "SetBivalentAPStopTemp"},
  {39, "SetForceHeater"},
  {40, "SetHeatingControl"},
  {41, "SetSmartDHW"},
  {42, "SetQuietModePriority"},
  {43, "SetPumpFlowrateMode"},
  {44, "SetDHWSensorSelection"},
  {45, "SetDHWHeaterState"},
  {46, "SetRoomHeaterState"},
  {47, "SetHeaterOnOutdoorTemp"},
  {RESET_COMMAND_ID, "SetReset"},
};

struct OptionalCommand {
  uint16_t id;
  const char *name;
  // Temperatures are written like they are read: as an int16 with two implied decimals (2150 = 21.50).
  bool scale100;
};

constexpr OptionalCommand OPTIONAL_COMMANDS[] = {
  {0, "SetHeatCoolMode", false},
  {1, "SetCompressorState", false},
  {2, "SetSmartGridMode", false},
  {3, "SetExternalThermostat1State", false},
  {4, "SetExternalThermostat2State", false},
  {5, "SetDemandControl", false},
  {6, "SetPoolTemp", true},
  {7, "SetBufferTemp", true},
  {8, "SetZ1RoomTemp", true},
  {9, "SetZ1WaterTemp", true},
  {10, "SetZ2RoomTemp", true},
  {11, "SetZ2WaterTemp", true},
  {12, "SetSolarTemp", true},
  {13, "SetOptPCBByte9", false},
};
}  // namespace ModbusMap
