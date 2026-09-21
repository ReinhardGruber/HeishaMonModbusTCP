#include <cassert>
#include <set>
#include "../../HeishaMon/HeishaModBusServer.cpp"
#include "generated_stubs.h"
#include "../../HeishaMon/s0data.cpp"

char actData[DATASIZE] = {};
char actDataExtra[DATASIZE] = {};
char actOptData[OPTDATASIZE] = {};
String readings[3][1000];
volatile s0DataStruct actS0Data[NUM_S0_COUNTERS];
volatile s0SettingsStruct actS0Settings[NUM_S0_COUNTERS];
std::string lastCommand, lastPayload;
bool relays[2] = {};
String getDataValue(char *, unsigned int index) { return readings[0][index]; }
String getDataValueExtra(char *, unsigned int index) { return readings[1][index]; }
String getOptDataValue(char *, unsigned int index) { return readings[2][index]; }
bool send_command(byte *, int) { return true; }
void log_message(char *) {}
void setRelay1(bool state) { relays[0] = state; }
void setRelay2(bool state) { relays[1] = state; }
void send_heatpump_command(char *topic, char *payload, bool (*)(byte *, int), void (*)(char *), bool) {
  lastCommand = topic; lastPayload = payload;
}

ModbusMessage call(uint8_t fc, uint16_t address, uint16_t value) {
  ModbusMessage request;
  request.add(uint8_t(1), fc, address, value);
  return ModbusServerTCPasync::workers.at(fc)(request);
}

uint16_t read(uint16_t address) {
  auto response = call(3, address, 1);
  assert(response.bytes.size() == 5 && response.bytes[1] == 3);
  uint16_t value;
  response.get(3, value);
  return value;
}

float readFloat(uint16_t address) {
  const auto response = call(3, address, 2);
  assert(response.bytes.size() == 7 && response.bytes[1] == 3);
  uint32_t bits;
  response.get(3, bits);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void expectError(uint8_t fc, uint16_t address, uint16_t value, uint8_t error) {
  auto response = call(fc, address, value);
  assert(response.bytes == std::vector<uint8_t>({1, uint8_t(fc | 128), error}));
}

int main() {
  HeishaModBusServer server;
  server.setup(true);
  for (auto &group : readings) for (auto &value : group) value = "1";
  assert(read(9000) == 2);

  // Every actual topic has exactly one integer and one two-word float mapping.
  std::set<uint16_t> addresses;
  for (const auto &range : kTopicRanges) {
    for (uint16_t i = 0; i < range.count; ++i) {
      auto integer = uint16_t(range.baseAddress + i);
      auto floating = ModbusMap::floatAddress(integer);
      assert(addresses.insert(integer).second);
      assert(addresses.insert(floating).second && addresses.insert(floating + 1).second);
      TopicSource source;
      uint16_t decoded;
      bool high;
      assert(decodeTopicAddress(integer, source, decoded) && source == range.source && decoded == i);
      assert(decodeFloatTopicAddress(floating, source, decoded, high) && high && source == range.source && decoded == i);
      assert(decodeFloatTopicAddress(floating + 1, source, decoded, high) && !high && decoded == i);
      read(integer);
      auto pair = call(3, floating, 2);
      assert(pair.bytes == std::vector<uint8_t>({1, 3, 4, 0x3f, 0x80, 0, 0})); // float 1.0
    }
    expectError(3, range.baseAddress + range.count, 1, ILLEGAL_DATA_ADDRESS);
    expectError(3, ModbusMap::floatAddress(range.baseAddress + range.count), 1, ILLEGAL_DATA_ADDRESS);
  }
  // Growing any group to its full capacity never moves/overlaps another block.
  addresses.clear();
  for (uint16_t base : {0, 1000, 2000, 3000}) {
    for (uint16_t i = 0; i < 1000; ++i) {
      auto start = ModbusMap::floatAddress(base + i);
      assert(addresses.insert(base + i).second);
      assert(addresses.insert(start).second && addresses.insert(start + 1).second);
      uint16_t index; bool high;
      assert(ModbusMap::decodeRange(start + 1, ModbusMap::floatAddress(base), 1000, 2, index, high));
      assert(index == i && !high);
    }
  }
  assert(ModbusMap::floatAddress(139) == 10278);
  assert(ModbusMap::floatAddress(1000) == 12000);
  assert(ModbusMap::floatAddress(2000) == 14000);

  // Source-aware fixed scaling: extra/optional index 1 must not inherit Pump_Flow x100.
  readings[0][14] = "-5.25";
  readings[0][67] = "2.2";
  readings[1][1] = "800";
  readings[2][1] = "2";
  assert(int16_t(read(14)) == -525 && read(67) == 220);
  assert(read(1001) == 800 && read(2001) == 2);
  readings[0][44] = "H74";
  assert(read(44) == 8074);
  readings[0][14] = "999";
  assert(read(14) == 32767);

  // S0 values use independent fixed port blocks; disabled/uninitialized inputs are zero.
  for (uint16_t base : {3000, 3100}) {
    for (uint16_t field = 0; field < 6; ++field) {
      assert(read(base + field) == 0);
      assert(readFloat(ModbusMap::floatAddress(base + field)) == 0);
    }
  }
  server.loop(true);
  assert(read(3005) == 0); // initialization has not assigned a GPIO yet
  actS0Settings[0].gpiopin = 1;
  actS0Settings[0].ppkwh = 2000;
  actS0Data[0].watt = 2500;
  actS0Data[0].pulsesTotal = 10001;
  actS0Data[0].pulses = 17;
  actS0Data[0].lastReportWatthour = 0.5f;
  actS0Data[0].goodPulses = 99;
  actS0Data[0].badPulses = 25;
  actS0Data[0].avgPulseWidth = 37;
  actS0Settings[1].gpiopin = 2;
  actS0Settings[1].ppkwh = 1000;
  actS0Data[1].watt = 42000;
  actS0Data[1].pulsesTotal = 4000000000U;
  actS0Data[1].lastReportWatthour = 12.25f;
  actS0Data[1].avgPulseWidth = 50;
  const float expectedS0[2][6] = {{2500, 5000.5f, 0.5f, 80, 37, 1},
                                 {42000, 4000000000.0f, 12.25f, 100, 50, 1}};
  for (uint16_t port = 0; port < 2; ++port) {
    for (uint16_t field = 0; field < 6; ++field) {
      const uint16_t integer = 3000 + port * 100 + field;
      assert(readFloat(ModbusMap::floatAddress(integer)) == expectedS0[port][field]);
      const float value = expectedS0[port][field];
      assert(read(integer) == (value >= 32767 ? 32767 : uint16_t(value)));
      expectError(6, integer, 0, ILLEGAL_DATA_ADDRESS); // read-only
    }
    expectError(3, 3006 + port * 100, 1, ILLEGAL_DATA_ADDRESS);
    expectError(3, 16012 + port * 200, 1, ILLEGAL_DATA_ADDRESS);
    auto block = call(3, 16000 + port * 200, 12);
    assert(block.bytes.size() == 27); // all six floats in one request
  }
  assert(actS0Data[0].pulses == 17 && actS0Data[0].pulsesTotal == 10001);
  assert(actS0Data[0].lastReportWatthour == 0.5f); // reading never resets an interval
  S0Reading snapshot[NUM_S0_COUNTERS];
  readS0Readings(true, snapshot);
  actS0Data[0].watt = 1234;
  uint16_t high, low;
  assert(s0ToRegisterValue(16000, snapshot, high) && s0ToRegisterValue(16001, snapshot, low));
  uint32_t bits = (uint32_t(high) << 16) | low;
  float original;
  std::memcpy(&original, &bits, sizeof(original));
  assert(original == 2500); // both words use the captured sample
  actS0Settings[1].ppkwh = 0;
  assert(read(3105) == 0 && readFloat(16202) == 0); // no divide by zero
  server.loop(false);
  assert(readFloat(16000) == 0 && read(3005) == 0);

  // Commands dispatch by stable IDs and signed payload, never by table position.
  addresses.clear();
  for (const auto &command : commands) {
    assert(command.id > 0 && command.id <= 1000);
    const auto address = ModbusMap::commandAddress(command.id);
    assert(addresses.insert(address).second);
    if (std::strcmp(command.name, "SetCurves") == 0) {
      expectError(6, address, 0, ILLEGAL_DATA_VALUE);
    } else {
      call(6, address, uint16_t(-5));
      assert(lastCommand == command.name && lastPayload == "-5");
    }
  }
  assert(ModbusMap::commandAddress(1) == 20000);
  assert(ModbusMap::commandAddress(100) == 22000);
  for (const auto &command : ModbusMap::OPTIONAL_COMMANDS) {
    assert(command.id < 1000);
    auto address = uint16_t(21000 + command.id);
    assert(addresses.insert(address).second);
    bool exists = false;
    for (const auto &upstream : optionalCommands) exists |= std::strcmp(command.name, upstream.name) == 0;
    assert(exists);
    call(6, address, 7);
    assert(lastCommand == command.name && lastPayload == "7");
  }
  expectError(6, 1001, 1, ILLEGAL_DATA_ADDRESS);
  expectError(6, 2000, 1, ILLEGAL_DATA_ADDRESS);
  expectError(6, 20999, 1, ILLEGAL_DATA_ADDRESS);
  expectError(6, 21100, 1, ILLEGAL_DATA_ADDRESS);
  expectError(3, 65535, 2, ILLEGAL_DATA_ADDRESS);
  expectError(3, 0, 0, ILLEGAL_DATA_VALUE);
  expectError(3, 0, 126, ILLEGAL_DATA_VALUE);
  call(5, 0, 0xFF00);
  assert(relays[0] && !relays[1]);
  call(5, 1, 0xFF00);
  call(5, 0, 0);
  assert(!relays[0] && relays[1]);
  expectError(5, 2, 0, ILLEGAL_DATA_ADDRESS);
  expectError(5, 1, 1, ILLEGAL_DATA_VALUE);

  // The web table must enumerate every topic, command, coil and version entry once.
  unsigned count = 0;
  String row;
  while (HeishaModBusServer::registerRow(count, row)) {
    assert(std::string(row.c_str()).find("<tr ") == 0);
    ++count;
    assert(count < 1000);
  }
  assert(count == NUMBER_OF_TOPICS + NUMBER_OF_TOPICS_EXTRA + NUMBER_OF_OPT_TOPICS +
                  arraySize(commands) + arraySize(optionalCommands) + 3 + NUM_S0_COUNTERS * S0_FIELD_COUNT);
  std::puts("PASS: complete map, expansion, scaling, S0 values, command dispatch, coils, boundaries and register page");
}
