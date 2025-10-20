#pragma once
#include <Arduino.h>
#include "ModbusServerTCPasync.h"


class HeishaModBusServer {
public:
    void setup();
    void loop();

private:
    // Callbacks für eModbus (müssen static sein)
    static ModbusMessage FC_03(ModbusMessage request);
    static ModbusMessage FC_05(ModbusMessage request);

private:
    ModbusServerTCPasync _mbServer;
};
