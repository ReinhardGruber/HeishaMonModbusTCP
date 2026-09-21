#pragma once
#include <Arduino.h>
#ifdef ESP32
#include "ModbusServerTCPasync.h"


class HeishaModBusServer {
public:
    void setup(bool isOptionalPCB, bool isS0Enabled = false);
    void loop(bool isS0Enabled);
    static bool registerRow(uint16_t index, String &html);

private:
    // Callbacks für eModbus (müssen static sein)
    static ModbusMessage FC_03(ModbusMessage request);
    static ModbusMessage FC_05(ModbusMessage request);
    static ModbusMessage FC_06(ModbusMessage request);

private:
    ModbusServerTCPasync _mbServer;
};
#else
class HeishaModBusServer {
public:
    void setup(bool, bool = false) {}
    void loop(bool) {}
};
#endif
