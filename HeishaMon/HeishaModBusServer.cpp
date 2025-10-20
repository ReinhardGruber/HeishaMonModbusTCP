#include "HeishaModBusServer.h"
#include "gpio.h"

// FC 0x03 / 0x04: Read Holding/Input Registers
ModbusMessage HeishaModBusServer::FC_03(ModbusMessage request) {
  ModbusMessage response;
  uint16_t addr = 0;
  uint16_t words = 0;
  request.get(2, addr);
  request.get(4, words);

  if ((addr + words) > 20) {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  if (request.getFunctionCode() == READ_HOLD_REGISTER) {
    for (uint8_t i = 0; i < words; ++i) {
      response.add((uint16_t)(addr + i));
    }
  } else {
    for (uint8_t i = 0; i < words; ++i) {
      response.add((uint16_t)random(1, 65535));
    }
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

void HeishaModBusServer::setup() {
  _mbServer.registerWorker(1, WRITE_COIL,           &HeishaModBusServer::FC_05);
  _mbServer.registerWorker(1, READ_HOLD_REGISTER,   &HeishaModBusServer::FC_03);
  _mbServer.start(502, 1, 20000);
}

void HeishaModBusServer::loop() {
  
}
