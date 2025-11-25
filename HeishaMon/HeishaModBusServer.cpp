#include "HeishaModBusServer.h"
#include "gpio.h"
#include "decode.h"
#include "commands.h"

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

constexpr uint16_t EXTRA_TOPIC_BASE = 500;
constexpr uint16_t OPTIONAL_TOPIC_BASE = 600;
constexpr uint16_t COMMAND_BASE = 1000;
constexpr uint16_t OPTIONAL_COMMAND_BASE = 2000;

constexpr uint16_t FLOAT_TOPIC_BASE = 10000;
constexpr uint16_t FLOAT_EXTRA_TOPIC_BASE = FLOAT_TOPIC_BASE + (NUMBER_OF_TOPICS * 2);
constexpr uint16_t FLOAT_OPTIONAL_TOPIC_BASE = FLOAT_EXTRA_TOPIC_BASE + (NUMBER_OF_TOPICS_EXTRA * 2);



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
  { 0, NUMBER_OF_TOPICS, TopicSource::Main },
  { EXTRA_TOPIC_BASE, NUMBER_OF_TOPICS_EXTRA, TopicSource::Extra },
  { OPTIONAL_TOPIC_BASE, NUMBER_OF_OPT_TOPICS, TopicSource::Optional }
};

struct FloatTopicRange {
  uint16_t baseAddress;
  uint16_t topicCount;
  TopicSource source;
};

constexpr FloatTopicRange kFloatTopicRanges[] = {
  { FLOAT_TOPIC_BASE, NUMBER_OF_TOPICS, TopicSource::Main },
  { FLOAT_EXTRA_TOPIC_BASE, NUMBER_OF_TOPICS_EXTRA, TopicSource::Extra },
  { FLOAT_OPTIONAL_TOPIC_BASE, NUMBER_OF_OPT_TOPICS, TopicSource::Optional }
};

template<typename T, size_t N>
constexpr size_t arraySize(const T (&)[N]) {
  return N;
}

bool nonNumericMainReported[NUMBER_OF_TOPICS] = { false };
bool nonNumericExtraReported[NUMBER_OF_TOPICS_EXTRA] = { false };
bool nonNumericOptReported[NUMBER_OF_OPT_TOPICS] = { false };

bool optionalPCB = false;

enum class CommandWriteResult {
  Success,
  InvalidAddress,
  UnsupportedValue
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

bool isTopicScale100(unsigned int topicNumber) {
  const char **description = (const char **)pgm_read_ptr(&topicDescription[topicNumber]);
  return
    (description == Celsius) ||
    (description == Kelvin) ||
    (description == LitersPerMin) ||
    (description == Pressure) ||
    (description == Bar);
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

bool stringToRegisterValue(const String &value, uint16_t &registerValue, uint16_t &topicIndex) {
  if (!isNumericValue(value)) {
    if (!isErrorState(value, registerValue)) {
      registerValue = 0;
      return false;
    }
    return true;
  }
  bool hasDecimal = value.indexOf('.') >= 0;
  if (hasDecimal || isTopicScale100(topicIndex)) {
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
  return true;
}

bool decodeTopicAddress(uint16_t address, TopicSource &source, uint16_t &topicIndex) {
  for (const TopicRange &range : kTopicRanges) {
    uint16_t rangeEnd = range.baseAddress + range.count;
    if ((address >= range.baseAddress) && (address < rangeEnd)) {
      source = range.source;
      topicIndex = address - range.baseAddress;
      return true;
    }
  }
  return false;
}

bool fetchTopicString(TopicSource source, uint16_t topicIndex, String &value) {
  switch (source) {
    case TopicSource::Main:
      if (topicIndex >= NUMBER_OF_TOPICS) {
        return false;
      }
      value = getDataValue(actData, topicIndex);
      return true;
    case TopicSource::Extra:
      if (topicIndex >= NUMBER_OF_TOPICS_EXTRA) {
        return false;
      }
      value = getDataValueExtra(actDataExtra, topicIndex);
      return true;
    case TopicSource::Optional:
      if (topicIndex >= NUMBER_OF_OPT_TOPICS) {
        return false;
      }
      value = getOptDataValue(actOptData, topicIndex);
      return true;
  }
  return false;
}

bool fetchTopicString(uint16_t address, String &value) {
  TopicSource source;
  uint16_t topicIndex = 0;
  if (!decodeTopicAddress(address, source, topicIndex)) {
    return false;
  }
  return fetchTopicString(source, topicIndex, value);
}

bool decodeFloatTopicAddress(uint16_t address, TopicSource &source, uint16_t &topicIndex, bool &highWord) {
  for (const FloatTopicRange &range : kFloatTopicRanges) {
    uint16_t rangeEnd = range.baseAddress + (range.topicCount * 2);
    if ((address >= range.baseAddress) && (address < rangeEnd)) {
      uint16_t offset = address - range.baseAddress;
      topicIndex = offset / 2;
      highWord = (offset % 2) == 0;
      source = range.source;
      return true;
    }
  }
  return false;
}

bool stringToFloatWords(const String &value, uint16_t &msw, uint16_t &lsw) {
  if (!isNumericValue(value)) {
    msw = 0;
    lsw = 0;
    return false;
  }
  float fValue = value.toFloat();
  uint32_t raw = 0;
  static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
  memcpy(&raw, &fValue, sizeof(raw));
  msw = static_cast<uint16_t>(raw >> 16);
  lsw = static_cast<uint16_t>(raw & 0xFFFF);
  return true;
}

bool shouldLogNonNumeric(TopicSource source, uint16_t topicIndex) {
  switch (source) {
    case TopicSource::Main:
      if (topicIndex < arraySize(nonNumericMainReported)) {
        bool shouldLog = !nonNumericMainReported[topicIndex];
        nonNumericMainReported[topicIndex] = true;
        return shouldLog;
      }
      break;
    case TopicSource::Extra:
      if (topicIndex < arraySize(nonNumericExtraReported)) {
        bool shouldLog = !nonNumericExtraReported[topicIndex];
        nonNumericExtraReported[topicIndex] = true;
        return shouldLog;
      }
      break;
    case TopicSource::Optional:
      if (topicIndex < arraySize(nonNumericOptReported)) {
        bool shouldLog = !nonNumericOptReported[topicIndex];
        nonNumericOptReported[topicIndex] = true;
        return shouldLog;
      }
      break;
  }
  return false;
}

void logNonNumericTopicValue(TopicSource source, uint16_t topicIndex, uint16_t address, const String &topicValue) {
  if (!shouldLogNonNumeric(source, topicIndex)) {
    return;
  }
  char logMsg[128];
  snprintf_P(logMsg, sizeof(logMsg), PSTR("Modbus: non-numeric topic value for register %u: %s"), address, topicValue.c_str());
  log_message(logMsg);
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

  if (!stringToRegisterValue(topicValue, registerValue, topicIndex)) {
    logNonNumericTopicValue(source, topicIndex, address, topicValue);
  }

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
  if (!stringToFloatWords(topicValue, msw, lsw)) {
    logNonNumericTopicValue(source, topicIndex, address, topicValue);
  }
  registerValue = highWord ? msw : lsw;
  return true;
}

bool copyMainCommandTopic(uint16_t address, char *topicName, size_t length) {
  if (length == 0) {
    return false;
  }

  for (size_t i = 0; i < arraySize(commands); ++i) {
    cmdStruct cmd;
    memcpy_P(&cmd, &commands[i], sizeof(cmd));
    if ((COMMAND_BASE + cmd.id) == address) {
      strncpy(topicName, cmd.name, length);
      topicName[length - 1] = '\0';
      return true;
    }
  }
  return false;
}

bool copyOptionalCommandTopic(uint16_t address, char *topicName, size_t length) {
  if (length == 0) {
    return false;
  }

  if (address < OPTIONAL_COMMAND_BASE) {
    return false;
  }

  uint16_t index = address - OPTIONAL_COMMAND_BASE;
  if (index >= arraySize(optionalCommands)) {
    return false;
  }

  optCmdStruct cmd;
  memcpy_P(&cmd, &optionalCommands[index], sizeof(cmd));
  strncpy(topicName, cmd.name, length);
  topicName[length - 1] = '\0';
  return true;
}

bool getCommandTopic(uint16_t address, char *topicName, size_t length) {
  if (copyMainCommandTopic(address, topicName, length)) {
    return true;
  }
  else if (copyOptionalCommandTopic(address, topicName, length)) {
    return true;
  }

  return false;
}

bool isJsonCommand(const char *commandTopic) {
  return strcmp(commandTopic, "SetCurves") == 0;
}

CommandWriteResult handleWriteCommand(uint16_t address, uint16_t registerValue) {
  char topicName[32] = { 0 };
  if (!getCommandTopic(address, topicName, sizeof(topicName))) {
    return CommandWriteResult::InvalidAddress;
  }

  if (isJsonCommand(topicName)) {
    return CommandWriteResult::UnsupportedValue;
  }

  char payload[16];
  snprintf(payload, sizeof(payload), "%d", static_cast<int16_t>(registerValue));
  send_heatpump_command(topicName, payload, send_command, log_message, optionalPCB);
  return CommandWriteResult::Success;
}

}  // namespace

// FC 0x03 / 0x04: Read Holding/Input Registers
ModbusMessage HeishaModBusServer::FC_03(ModbusMessage request) {
  ModbusMessage response;
  uint16_t addr = 0;
  uint16_t words = 0;
  request.get(2, addr);
  request.get(4, words);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = 0; i < words; ++i) {
    uint16_t registerValue = 0;
    uint16_t targetAddress = addr + i;
    if (!topicToRegisterValue(targetAddress, registerValue) &&
        !topicToFloatRegisterValue(targetAddress, registerValue)) {
      response.clear();
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
      return response;
    }
    response.add(registerValue);
  }
  return response;
}

// FC 0x05: Write Single Coil
ModbusMessage HeishaModBusServer::FC_05(ModbusMessage request) {
  ModbusMessage response;

  uint16_t start = 0;
  uint16_t state = 0;
  request.get(2, start, state);

  if (start <= 2) {
    if (state == 0x0000) {
      setRelay1(0);
      response = ECHO_RESPONSE;
    } else if (state == 0xFF00) {
      setRelay1(1);
      response = ECHO_RESPONSE;
    } else {
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    }
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

// FC 0x06: Write Single Register
ModbusMessage HeishaModBusServer::FC_06(ModbusMessage request) {
  ModbusMessage response;
  uint16_t address = 0;
  uint16_t value = 0;
  request.get(2, address, value);

  CommandWriteResult result = handleWriteCommand(address, value);
  if (result == CommandWriteResult::InvalidAddress) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  if (result == CommandWriteResult::UnsupportedValue) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    return response;
  }

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(value);
  return response;
}

void HeishaModBusServer::setup(bool isOptionalPCB) 
{
  optionalPCB = isOptionalPCB;

  _mbServer.registerWorker(1, WRITE_COIL,           &HeishaModBusServer::FC_05);
  _mbServer.registerWorker(1, READ_HOLD_REGISTER,   &HeishaModBusServer::FC_03);
  _mbServer.registerWorker(1, WRITE_HOLD_REGISTER,  &HeishaModBusServer::FC_06);
  _mbServer.start(502, 1, 20000);
}

void HeishaModBusServer::loop() 
{
  
}