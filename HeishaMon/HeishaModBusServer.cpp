#include "HeishaModBusServer.h"
#include "gpio.h"
#include "decode.h"
#include "commands.h"

#include <ctype.h>
#include <math.h>
#include <string.h>


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

enum class CommandSource {
  Main,
  Optional
};

struct CommandRange {
  uint16_t baseAddress;
  uint16_t count;
  CommandSource source;
};

template<typename T, size_t N>
constexpr size_t arraySize(const T (&)[N]) {
  return N;
}

constexpr CommandRange kCommandRanges[] = {
  { COMMAND_BASE, arraySize(commands), CommandSource::Main },
  { OPTIONAL_COMMAND_BASE, arraySize(optionalCommands), CommandSource::Optional }
};

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

bool stringToRegisterValue(const String &value, uint16_t &registerValue) {
  if (!isNumericValue(value)) {
    registerValue = 0;
    return false;
  }

  bool hasDecimal = value.indexOf('.') >= 0;
  if (hasDecimal) {
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

bool fetchTopicString(uint16_t address, String &value) {
  TopicSource source;
  uint16_t topicIndex = 0;
  if (!decodeTopicAddress(address, source, topicIndex)) {
    return false;
  }

  switch (source) {
    case TopicSource::Main:
      value = getDataValue(actData, topicIndex);
      return true;
    case TopicSource::Extra:
      value = getDataValueExtra(actDataExtra, topicIndex);
      return true;
    case TopicSource::Optional:
      value = getOptDataValue(actOptData, topicIndex);
      return true;
  }
  return false;
}

bool topicToRegisterValue(uint16_t address, uint16_t &registerValue) {
  String topicValue;
  if (!fetchTopicString(address, topicValue)) {
    return false;
  }

  if (!stringToRegisterValue(topicValue, registerValue)) {
    TopicSource source;
    uint16_t topicIndex = 0;
    bool shouldLog = decodeTopicAddress(address, source, topicIndex);
    if (shouldLog) {
      switch (source) {
        case TopicSource::Main:
          shouldLog = !nonNumericMainReported[topicIndex];
          nonNumericMainReported[topicIndex] = true;
          break;
        case TopicSource::Extra:
          shouldLog = !nonNumericExtraReported[topicIndex];
          nonNumericExtraReported[topicIndex] = true;
          break;
        case TopicSource::Optional:
          shouldLog = !nonNumericOptReported[topicIndex];
          nonNumericOptReported[topicIndex] = true;
          break;
      }
    }
    if (shouldLog) {
      char logMsg[128];
      snprintf_P(logMsg, sizeof(logMsg), PSTR("Modbus: non-numeric topic value for register %u: %s"), address, topicValue.c_str());
      log_message(logMsg);
    }
  }

  return true;
}

bool getCommandTopic(uint16_t address, char *topicName, size_t length) {
  for (const CommandRange &range : kCommandRanges) {
    uint16_t rangeEnd = range.baseAddress + range.count;
    if ((address >= range.baseAddress) && (address < rangeEnd)) {
      uint16_t index = address - range.baseAddress;
      if (range.source == CommandSource::Main) {
        cmdStruct cmd;
        memcpy_P(&cmd, &commands[index], sizeof(cmd));
        strncpy(topicName, cmd.name, length);
      } else {
        optCmdStruct cmd;
        memcpy_P(&cmd, &optionalCommands[index], sizeof(cmd));
        strncpy(topicName, cmd.name, length);
      }
      topicName[length - 1] = '\0';
      return true;
    }
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
    if (!topicToRegisterValue(targetAddress, registerValue)) {
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