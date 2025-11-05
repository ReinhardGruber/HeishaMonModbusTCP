#ifdef ESP32
#include "HeishaModbusServer.h"
#include "gpio.h"
#include "ModbusRegisterMap.h"
#include "decode.h"
#include "commands.h"
#include "s0data.h"
#include <atomic>
#include <mutex>

#include <ctype.h>
#include <math.h>
#include <string.h>
#include <stdint.h>


extern char actData[DATASIZE];
extern char actDataExtra[DATASIZE];
extern char actOptData[OPTDATASIZE];
extern bool send_command(byte* command, int length);
extern void log_message(char *string);

namespace {

using namespace ModbusMap;

static_assert(NUMBER_OF_TOPICS <= TOPIC_CAPACITY, "Main Modbus block is full");
static_assert(NUMBER_OF_TOPICS_EXTRA <= TOPIC_CAPACITY, "Extra Modbus block is full");
static_assert(NUMBER_OF_OPT_TOPICS <= TOPIC_CAPACITY, "Optional Modbus block is full");

enum class TopicSource {
  Main,
  Extra,
  Optional
};

struct TopicRange {
  uint16_t baseAddress;
  uint16_t count;
  TopicSource source;
};

constexpr TopicRange kTopicRanges[] = {
  { MAIN_TOPIC_BASE, NUMBER_OF_TOPICS, TopicSource::Main },
  { EXTRA_TOPIC_BASE, NUMBER_OF_TOPICS_EXTRA, TopicSource::Extra },
  { OPTIONAL_TOPIC_BASE, NUMBER_OF_OPT_TOPICS, TopicSource::Optional }
};

template<typename T, size_t N>
constexpr size_t arraySize(const T (&)[N]) {
  return N;
}

static_assert(arraySize(MAIN_COMMANDS) == arraySize(commands),
              "Assign permanent Modbus IDs to new commands in ModbusRegisterMap.h");
static_assert(arraySize(OPTIONAL_COMMANDS) == arraySize(optionalCommands),
              "Assign permanent Modbus IDs to new optional commands in ModbusRegisterMap.h");

// The Modbus callbacks run in the AsyncTCP task. Everything they share with the main
// loop lives below and is only accessed while holding stateMutex.
std::mutex stateMutex;
char snapshotMain[DATASIZE] = { 0 };
char snapshotExtra[DATASIZE] = { 0 };
char snapshotOpt[OPTDATASIZE] = { 0 };
bool snapshotExtraAvailable = false;

struct WriteRequest {
  bool coil;
  uint16_t address;
  uint16_t value;
};

constexpr size_t WRITE_QUEUE_SIZE = 16;
WriteRequest writeQueue[WRITE_QUEUE_SIZE];
size_t writeQueueHead = 0;
size_t writeQueueCount = 0;

// Set once in setup() before the server accepts connections.
bool optionalPCB = false;
std::atomic<bool> s0Enabled{false};
std::atomic<bool> writesAllowed{false};

static_assert(NUM_S0_COUNTERS * S0_PORT_STRIDE <= TOPIC_CAPACITY, "S0 Modbus block is full");
static_assert(S0_FIELD_COUNT <= S0_PORT_STRIDE, "S0 port block is full");
const char *const s0FieldNames[] = {
  "Watt", "WatthourTotal", "Watthour_Last_Report", "PulseQuality", "AvgPulseWidth", "Enabled"
};
const char *const s0FieldUnits[] = {"W", "Wh", "Wh", "%", "ms", "0/1"};
static_assert(arraySize(s0FieldNames) == S0_FIELD_COUNT, "S0 field names missing");
static_assert(arraySize(s0FieldUnits) == S0_FIELD_COUNT, "S0 field units missing");

bool decodeS0Address(uint16_t address, uint16_t &port, uint16_t &field, bool &floating, bool &highWord) {
  for (port = 0; port < NUM_S0_COUNTERS; ++port) {
    const uint16_t base = S0_TOPIC_BASE + port * S0_PORT_STRIDE;
    floating = false;
    if (decodeRange(address, base, S0_FIELD_COUNT, 1, field, highWord)) return true;
    floating = true;
    if (decodeRange(address, floatAddress(base), S0_FIELD_COUNT, 2, field, highWord)) return true;
  }
  return false;
}

float s0FieldValue(const S0Reading &reading, uint16_t field) {
  switch (field) {
    case 0: return reading.watt;
    case 1: return reading.watthourTotal;
    case 2: return reading.watthour;
    case 3: return reading.pulseQuality;
    case 4: return reading.avgPulseWidth;
    case 5: return reading.enabled ? 1.0f : 0.0f;
  }
  return 0;
}

bool s0ToRegisterValue(uint16_t address, const S0Reading readings[NUM_S0_COUNTERS], uint16_t &result) {
  uint16_t port, field;
  bool floating, highWord;
  if (!decodeS0Address(address, port, field, floating, highWord)) return false;
  const float value = s0FieldValue(readings[port], field);
  if (floating) {
    uint32_t bits;
    static_assert(sizeof(value) == sizeof(bits), "Unexpected float size");
    memcpy(&bits, &value, sizeof(bits));
    result = highWord ? uint16_t(bits >> 16) : uint16_t(bits);
  } else {
    // Match the unscaled int16 ranges: truncate fractions, saturate large values.
    result = uint16_t(value >= 32767.0f ? 32767 : value <= 0.0f ? 0 : int16_t(value));
  }
  return true;
}

enum class CommandWriteResult {
  Success,
  InvalidAddress,
  UnsupportedValue,
  Unavailable,
  Busy
};

bool isNumericValue(const String &value) {
  if (value.length() == 0) {
    return false;
  }
  bool hasDigits = false;
  bool hasDecimal = false;
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value.charAt(i);
    if ((c == '-') && (i == 0)) {
      continue;
    }
    if ((c == '.') && !hasDecimal) {
      hasDecimal = true;
      continue;
    }
    if (!isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
    hasDigits = true;
  }
  return hasDigits;
}

bool isTopicScale100(TopicSource source, unsigned int topicNumber) {
  const char **description = nullptr;
  switch (source) {
    case TopicSource::Main:
      description = (const char **)pgm_read_ptr(&topicDescription[topicNumber]); break;
    case TopicSource::Extra:
      description = (const char **)pgm_read_ptr(&xtopicDescription[topicNumber]); break;
    case TopicSource::Optional:
      description = (const char **)pgm_read_ptr(&opttopicDescription[topicNumber]); break;
  }
  return
    (description == Celsius) ||
    (description == Kelvin) ||
    (description == LitersPerMin) ||
    (description == Pressure) ||
    (description == Bar) ||
    (description == Ampere);
}

bool isErrorState(const String &value, uint16_t &registerValue) {
  if (value.length() < 2) {
    return false;
  }

  char prefix = value.charAt(0);
  if (!isupper(static_cast<unsigned char>(prefix))) {
    return false;
  }

  String numericPart = value.substring(1);
  if (!isNumericValue(numericPart)) {
    return false;
  }

  long intValue = numericPart.toInt();
  intValue += ((prefix - 'A') + 1) * 1000;

  if (intValue > 32767) {
    intValue = 32767;
  }
  if (intValue < -32768) {
    intValue = -32768;
  }

  registerValue = static_cast<uint16_t>(static_cast<int16_t>(intValue));
  return true;
}

// Non-numeric values that are not error codes read as 0.
void stringToRegisterValue(const String &value, uint16_t &registerValue, TopicSource source, uint16_t topicIndex) {
  if (!isNumericValue(value)) {
    if (!isErrorState(value, registerValue)) {
      registerValue = 0;
    }
    return;
  }
  if (isTopicScale100(source, topicIndex)) {
    float fValue = value.toFloat();
    float scaled = fValue * 100.0f;
    if (scaled > 32767.0f) {
      scaled = 32767.0f;
    }
    if (scaled < -32768.0f) {
      scaled = -32768.0f;
    }
    int16_t intValue = static_cast<int16_t>(roundf(scaled));
    registerValue = static_cast<uint16_t>(intValue);
  } else {
    long intValue = value.toInt();
    if (intValue > 32767) {
      intValue = 32767;
    }
    if (intValue < -32768) {
      intValue = -32768;
    }
    registerValue = static_cast<uint16_t>(static_cast<int16_t>(intValue));
  }
}

bool decodeTopicAddress(uint16_t address, TopicSource &source, uint16_t &topicIndex) {
  for (const TopicRange &range : kTopicRanges) {
    bool highWord;
    if (decodeRange(address, range.baseAddress, range.count, 1, topicIndex, highWord)) {
      source = range.source;
      return true;
    }
  }
  return false;
}

// Must be called with stateMutex held. Returns false when the topic does not exist or
// the data behind it is not available on this heat pump / configuration, so that the
// caller answers with an exception instead of a value decoded from an empty buffer.
bool fetchTopicString(TopicSource source, uint16_t topicIndex, String &value) {
  switch (source) {
    case TopicSource::Main:
      if (topicIndex >= NUMBER_OF_TOPICS) {
        return false;
      }
      value = getDataValue(snapshotMain, topicIndex);
      return true;
    case TopicSource::Extra:
      if (topicIndex >= NUMBER_OF_TOPICS_EXTRA || !snapshotExtraAvailable) {
        return false;
      }
      value = getDataValueExtra(snapshotExtra, topicIndex);
      return true;
    case TopicSource::Optional:
      if (topicIndex >= NUMBER_OF_OPT_TOPICS || !optionalPCB) {
        return false;
      }
      value = getOptDataValue(snapshotOpt, topicIndex);
      return true;
  }
  return false;
}

bool decodeFloatTopicAddress(uint16_t address, TopicSource &source, uint16_t &topicIndex, bool &highWord) {
  for (const TopicRange &range : kTopicRanges) {
    if (decodeRange(address, floatAddress(range.baseAddress), range.count, 2, topicIndex, highWord)) {
      source = range.source;
      return true;
    }
  }
  return false;
}

void stringToFloatWords(const String &value, uint16_t &msw, uint16_t &lsw) {
  float fValue = 0;
  if (isNumericValue(value)) {
    fValue = value.toFloat();
  }
  uint32_t raw = 0;
  static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
  memcpy(&raw, &fValue, sizeof(raw));
  msw = static_cast<uint16_t>(raw >> 16);
  lsw = static_cast<uint16_t>(raw & 0xFFFF);
}

bool topicToRegisterValue(uint16_t address, uint16_t &registerValue) {
  TopicSource source;
  uint16_t topicIndex = 0;
  if (!decodeTopicAddress(address, source, topicIndex)) {
    return false;
  }

  String topicValue;
  if (!fetchTopicString(source, topicIndex, topicValue)) {
    return false;
  }

  stringToRegisterValue(topicValue, registerValue, source, topicIndex);
  return true;
}

bool topicToFloatRegisterValue(uint16_t address, uint16_t &registerValue) {
  TopicSource source;
  uint16_t topicIndex = 0;
  bool highWord = false;
  if (!decodeFloatTopicAddress(address, source, topicIndex, highWord)) {
    return false;
  }

  String topicValue;
  if (!fetchTopicString(source, topicIndex, topicValue)) {
    return false;
  }

  uint16_t msw = 0;
  uint16_t lsw = 0;
  stringToFloatWords(topicValue, msw, lsw);
  registerValue = highWord ? msw : lsw;
  return true;
}

struct CommandTarget {
  const char *name;
  bool optional;
  bool scale100;
};

bool resolveCommand(uint16_t address, CommandTarget &target) {
  for (const MainCommand &command : MAIN_COMMANDS) {
    if (commandAddress(command.id) == address) {
      target = { command.name, false, false };
      return true;
    }
  }
  for (const OptionalCommand &command : OPTIONAL_COMMANDS) {
    if (address == OPTIONAL_COMMAND_BASE + command.id) {
      target = { command.name, true, command.scale100 };
      return true;
    }
  }
  return false;
}

bool isJsonCommand(const char *commandTopic) {
  return strcmp(commandTopic, "SetCurves") == 0;
}

bool enqueueWrite(const WriteRequest &request) {
  std::lock_guard<std::mutex> lock(stateMutex);
  if (writeQueueCount >= WRITE_QUEUE_SIZE) {
    return false;
  }
  writeQueue[(writeQueueHead + writeQueueCount) % WRITE_QUEUE_SIZE] = request;
  ++writeQueueCount;
  return true;
}

bool dequeueWrite(WriteRequest &request) {
  std::lock_guard<std::mutex> lock(stateMutex);
  if (writeQueueCount == 0) {
    return false;
  }
  request = writeQueue[writeQueueHead];
  writeQueueHead = (writeQueueHead + 1) % WRITE_QUEUE_SIZE;
  --writeQueueCount;
  return true;
}

// Runs in the async task: only validates and queues, the command is executed by loop().
CommandWriteResult handleWriteCommand(uint16_t address, uint16_t registerValue) {
  CommandTarget target;
  if (!resolveCommand(address, target)) {
    return CommandWriteResult::InvalidAddress;
  }

  if (isJsonCommand(target.name)) {
    return CommandWriteResult::UnsupportedValue;
  }

  // send_heatpump_command() silently ignores optional PCB commands when the PCB is disabled.
  if (target.optional && !optionalPCB) {
    return CommandWriteResult::Unavailable;
  }

  return enqueueWrite({ false, address, registerValue }) ? CommandWriteResult::Success : CommandWriteResult::Busy;
}

// Runs in loop().
void executeWrite(const WriteRequest &request) {
  if (request.coil) {
    if (request.address == 0) setRelay1(request.value == 0xFF00);
    else setRelay2(request.value == 0xFF00);
    return;
  }

  CommandTarget target;
  if (!resolveCommand(request.address, target)) {
    return;
  }

  char topicName[32] = { 0 };
  strncpy(topicName, target.name, sizeof(topicName) - 1);

  char payload[16];
  const int value = static_cast<int16_t>(request.value);
  if (target.scale100) {
    const int magnitude = value < 0 ? -value : value;
    snprintf(payload, sizeof(payload), "%s%d.%02d", value < 0 ? "-" : "", magnitude / 100, magnitude % 100);
  } else {
    snprintf(payload, sizeof(payload), "%d", value);
  }
  send_heatpump_command(topicName, payload, send_command, log_message, optionalPCB);
}

}  // namespace

// Render from the same ranges and command tables used by Modbus itself.
// One row per callback keeps the HTTP response bounded on the device.
bool HeishaModbusServer::registerRow(uint16_t index, String &html) {
  for (const TopicRange &range : kTopicRanges) {
    if (index >= range.count) {
      index -= range.count;
      continue;
    }
    const char *name = nullptr;
    const char *group = nullptr;
    switch (range.source) {
      case TopicSource::Main: name = topics[index]; group = "Main"; break;
      case TopicSource::Extra: name = xtopics[index]; group = "Extra"; break;
      case TopicSource::Optional: name = optTopics[index]; group = "Optional PCB"; break;
    }
    const uint16_t floatRegister = floatAddress(range.baseAddress + index);
    html = "<tr data-kind='read'><td>";
    html += group;
    html += "</td><td>";
    html += String(FPSTR(name));
    html += "</td><td>";
    html += String(range.baseAddress + index);
    html += "</td><td>";
    html += String(floatRegister);
    html += " / ";
    html += String(floatRegister + 1);
    html += "</td><td>Read / FC03</td><td>";
    // The scale is fixed by the topic unit, never by the current value text.
    html += isTopicScale100(range.source, index) ? "Integer: x100" : "Integer: x1";
    html += "; float: x1";
    if (range.source == TopicSource::Main && strcmp_P("Error", name) == 0) {
      html += "; errors: letter block + code (H74 = 8074); float returns 0 for text";
    }
    if (range.source == TopicSource::Optional) {
      html += optionalPCB ? "; optional PCB enabled" : "; optional PCB disabled: illegal data address";
    } else if (range.source == TopicSource::Extra) {
      html += "; requires extra heat-pump data: illegal data address if not available";
    }
    html += "</td></tr>";
    return true;
  }
  if (index < NUM_S0_COUNTERS * S0_FIELD_COUNT) {
    const uint16_t port = index / S0_FIELD_COUNT;
    const uint16_t field = index % S0_FIELD_COUNT;
    const uint16_t integer = S0_TOPIC_BASE + port * S0_PORT_STRIDE + field;
    const uint16_t floating = floatAddress(integer);
    html = "<tr data-kind='read'><td>S0 ";
    html += String(port + 1);
    html += "</td><td>";
    html += s0FieldNames[field];
    html += "</td><td>";
    html += String(integer);
    html += "</td><td>";
    html += String(floating);
    html += " / ";
    html += String(floating + 1);
    html += "</td><td>Read / FC03</td><td>";
    html += s0FieldUnits[field];
    html += "; x1; int16 truncates fractions and saturates at 32767; use float for energy";
    if (field == 1) html += "; includes restored total; no automatic flash persistence";
    if (field == 2) html += "; last completed S0 report interval; reads do not reset it";
    if (field == 5) html += "; enabled and initialized with a valid pulses/kWh setting";
    html += "; enable S0 in Settings; disabled inputs return 0</td></tr>";
    return true;
  }
  index -= NUM_S0_COUNTERS * S0_FIELD_COUNT;
  const char *name;
  uint16_t address;
  bool optional = false;
  bool scale100 = false;
  if (index < arraySize(MAIN_COMMANDS)) {
    address = commandAddress(MAIN_COMMANDS[index].id);
    name = MAIN_COMMANDS[index].name;
  } else {
    index -= arraySize(MAIN_COMMANDS);
    if (index < arraySize(OPTIONAL_COMMANDS)) {
      address = OPTIONAL_COMMAND_BASE + OPTIONAL_COMMANDS[index].id;
      name = OPTIONAL_COMMANDS[index].name;
      scale100 = OPTIONAL_COMMANDS[index].scale100;
      optional = true;
    } else {
      index -= arraySize(OPTIONAL_COMMANDS);
      if (index < RELAY_COUNT) {
        html = "<tr data-kind='write'><td>Relay</td><td>Relay ";
        html += String(index + 1);
        html += "</td><td>";
        html += String(index);
        html += " (coil)</td><td>-</td><td>Write / FC05</td>"
                "<td>0x0000 = off; 0xFF00 = on</td></tr>";
      } else if (index == RELAY_COUNT) {
        html = "<tr data-kind='read'><td>Device</td><td>Register map version</td><td>";
        html += String(VERSION_REGISTER);
        html += "</td><td>-</td><td>Read / FC03</td><td>uint16: ";
        html += String(VERSION);
        html += "</td></tr>";
      } else {
        return false;
      }
      return true;
    }
  }
  html = "<tr data-kind='write'><td>";
  html += optional ? "Optional PCB" : (address >= SYSTEM_COMMAND_BASE ? "System command" : "Command");
  html += "</td><td>";
  html += name;
  html += "</td><td>";
  html += String(address);
  html += "</td><td>-</td><td>";
  html += isJsonCommand(name) ? "Unsupported" : "Write / FC06";
  html += "</td><td>";
  if (isJsonCommand(name)) {
    html += "JSON command; use MQTT or HTTP";
  } else if (scale100) {
    html += "Signed int16; x100 like the temperature readings (2150 = 21.50)";
  } else {
    html += "Signed int16; x1";
  }
  if (optional && !optionalPCB) html += "; optional PCB disabled: illegal data address";
  html += "</td></tr>";
  return true;
}

// FC 0x03: Read Holding Registers
ModbusMessage HeishaModbusServer::FC_03(ModbusMessage request) {
  ModbusMessage response;
  uint16_t addr = 0;
  uint16_t words = 0;
  request.get(2, addr);
  request.get(4, words);

  if (words == 0 || words > 125) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    return response;
  }
  if (uint32_t(addr) + words > 65536) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  S0Reading s0Readings[NUM_S0_COUNTERS];
  bool sampledS0 = false;
  // Sample once per request; never mix two readings in one float register pair.
  for (uint16_t i = 0; i < words && !sampledS0; ++i) {
    uint16_t port, field;
    bool floating, highWord;
    if (decodeS0Address(addr + i, port, field, floating, highWord)) {
      readS0Readings(s0Enabled.load(), s0Readings);
      sampledS0 = true;
    }
  }

  // Serve the whole request from one consistent snapshot of the heat pump data.
  std::lock_guard<std::mutex> lock(stateMutex);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = 0; i < words; ++i) {
    uint16_t registerValue = 0;
    uint16_t targetAddress = addr + i;
    if (targetAddress == VERSION_REGISTER) {
      registerValue = VERSION;
    } else if (!topicToRegisterValue(targetAddress, registerValue) &&
        !topicToFloatRegisterValue(targetAddress, registerValue) &&
        !(sampledS0 && s0ToRegisterValue(targetAddress, s0Readings, registerValue))) {
      response.clear();
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
      return response;
    }
    response.add(registerValue);
  }
  return response;
}

// FC 0x05: Write Single Coil
ModbusMessage HeishaModbusServer::FC_05(ModbusMessage request) {
  ModbusMessage response;

  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);

  if (!writesAllowed.load()) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
  } else if (start >= RELAY_COUNT) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  } else if (state != 0x0000 && state != 0xFF00) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
  } else if (!enqueueWrite({ true, start, state })) {
    response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
  } else {
    response = ECHO_RESPONSE;
  }
  return response;
}

// FC 0x06: Write Single Register
ModbusMessage HeishaModbusServer::FC_06(ModbusMessage request) {
  ModbusMessage response;
  uint16_t address = 0;
  uint16_t value = 0;
  request.get(2, address, value);

  if (!writesAllowed.load()) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
    return response;
  }

  switch (handleWriteCommand(address, value)) {
    case CommandWriteResult::Success:
      break;
    case CommandWriteResult::InvalidAddress:
    case CommandWriteResult::Unavailable:
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
      return response;
    case CommandWriteResult::UnsupportedValue:
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
      return response;
    case CommandWriteResult::Busy:
      response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_BUSY);
      return response;
  }

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(value);
  return response;
}

void HeishaModbusServer::setup(bool isOptionalPCB, bool isS0Enabled, bool allowWrites)
{
  optionalPCB = isOptionalPCB;
  s0Enabled.store(isS0Enabled);
  writesAllowed.store(allowWrites);

  _mbServer.registerWorker(1, WRITE_COIL,           &HeishaModbusServer::FC_05);
  _mbServer.registerWorker(1, READ_HOLD_REGISTER,   &HeishaModbusServer::FC_03);
  _mbServer.registerWorker(1, WRITE_HOLD_REGISTER,  &HeishaModbusServer::FC_06);
  _mbServer.start(502, 1, 20000);
}

void HeishaModbusServer::loop(bool isS0Enabled, bool extraDataBlockAvailable)
{
  s0Enabled.store(isS0Enabled);

  {
    std::lock_guard<std::mutex> lock(stateMutex);
    memcpy(snapshotMain, actData, sizeof(snapshotMain));
    memcpy(snapshotExtra, actDataExtra, sizeof(snapshotExtra));
    memcpy(snapshotOpt, actOptData, sizeof(snapshotOpt));
    snapshotExtraAvailable = extraDataBlockAvailable;
  }

  // Execute queued writes here, in the main loop, where sending commands, logging and
  // publishing to MQTT are safe.
  WriteRequest request;
  while (dequeueWrite(request)) {
    executeWrite(request);
  }
}
#endif
