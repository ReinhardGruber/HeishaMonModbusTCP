#pragma once
#include <Arduino.h>
#ifdef ESP32
#include "ModbusServerTCPasync.h"


class HeishaModBusServer {
public:
    void setup(bool isOptionalPCB);
    void loop();

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
    void setup(bool) {}
    void loop() {}
};
#endif
