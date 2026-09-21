# Modbus register map v2

Firmware: **4.2.2-ModbusTCP**. This is a breaking change: update every existing
PLC/Loxone mapping before using the new firmware. Old addresses are not aliases;
some now refer to different values.

TCP port **502**, unit ID **1**. All addresses below are zero-based protocol offsets,
without a 40001 prefix. Add 1 only if your client expects one-based addressing.
ESP32 supports Modbus TCP; ESP8266 does not.

## Fixed blocks

Each measurement block reserves room for **1,000 topics**. Counts can grow without
moving any other block. Only implemented topics/commands are accessible; reserved
addresses return `ILLEGAL_DATA_ADDRESS`.

| Group | Reserved integer block (FC03, int16) | Reserved float block (FC03, float32) | Currently implemented |
| --- | --- | --- | --- |
| Main TOPn | 0-999 | 10000-11999 | TOP0-TOP143: integer 0-143, float 10000-10287 |
| Extra XTOPn | 1000-1999 | 12000-13999 | XTOP0-XTOP5: integer 1000-1005, float 12000-12011 |
| Optional PCB OPTn | 2000-2999 | 14000-15999 | OPT0-OPT6: integer 2000-2006, float 14000-14013 |

**Integer address = group base + topic number.**
**Float start = 10000 + 2 * integer address.**
Read two registers for a float: MSW at the start address, LSW at start + 1.
All floats are IEEE 754 float32, unscaled.

| Other group | Reserved block | Access | Implemented |
| --- | --- | --- | --- |
| Device information | 9000-9999 | FC03, uint16 | 9000 = register map version, currently 2 |
| Heat-pump commands | 20000-20999 | FC06, int16 | 20000-20046; 20015 is reserved for JSON-only SetCurves and rejects writes |
| Optional PCB commands | 21000-21999 | FC06, int16 | 21000-21013 |
| System commands | 22000-22999 | FC06, int16 | 22000 = SetReset |
| Relay coils (separate coil address space) | 0-999 | FC05 | 0 = relay 1, 1 = relay 2 |

FC05 accepts 0x0000 for off and 0xFF00 for on. Coil 2 is no longer an alias.
FC03 supports 1-125 registers per request; reads across a reserved gap are rejected.
Only FC03, FC05 and FC06 are supported.

## Register page

Open **Modbus registers** in the device menu or `http://<heishamon-ip>/modbus`.
The searchable page is generated from the actual firmware ranges and command IDs.
It lists both addresses for each measurement, scaling, function codes and every
command, sorted numerically within read values and write commands. It only displays the map; it does not send commands or show live values.

## Scaling

The integer multiplier is fixed by the topic's unit, not the current value text:

- Temperature (Celsius/Kelvin), flow, pressure and current (Ampere): **x100**.
  Divide by 100 in your client; 2050 means 20.50, -525 means -5.25.
- States, counters, power (W), rotational speed and other units: **x1**.
- Integer values are saturated to -32768 through 32767. Use floats for large counters/power values.
- Extra/optional topics use their own unit definitions. They no longer accidentally
  inherit the main topic's scaling at the same index.
- Non-numeric readings return 0 and are logged once. Error codes in the integer
  Error register 44 use A=1000, B=2000, ..., H=8000 plus the number: H74=8074.
  Its float counterpart returns 0 for text; use register 44 for error information.

**Write commands are always unscaled signed int16**, including temperatures.
For example, write 45 to SetDHWTemp, not 4500. Allowed values are those of the
regular HeishaMon command handlers. SetCurves needs JSON and must use MQTT/HTTP.

## Main commands

Addresses are based on permanent command IDs, not array order. IDs 1-1000 map to
20000 + ID - 1; legacy ID 100 is reserved permanently and maps to SetReset at 22000.

| Address | Command |
| --- | --- |
| 20000 | `SetHeatpump` |
| 20001 | `SetHolidayMode` |
| 20002 | `SetQuietMode` |
| 20003 | `SetPowerfulMode` |
| 20004 | `SetZ1HeatRequestTemperature` |
| 20005 | `SetZ1CoolRequestTemperature` |
| 20006 | `SetZ2HeatRequestTemperature` |
| 20007 | `SetZ2CoolRequestTemperature` |
| 20008 | `SetOperationMode` |
| 20009 | `SetForceDHW` |
| 20010 | `SetDHWTemp` |
| 20011 | `SetForceDefrost` |
| 20012 | `SetForceSterilization` |
| 20013 | `SetPump` |
| 20014 | `SetMaxPumpDuty` |
| 20015 | `SetCurves` |
| 20016 | `SetZones` |
| 20017 | `SetFloorHeatDelta` |
| 20018 | `SetFloorCoolDelta` |
| 20019 | `SetDHWHeatDelta` |
| 20020 | `SetHeaterDelayTime` |
| 20021 | `SetHeaterStartDelta` |
| 20022 | `SetHeaterStopDelta` |
| 20023 | `SetMainSchedule` |
| 20024 | `SetAltExternalSensor` |
| 20025 | `SetExternalPadHeater` |
| 20026 | `SetBufferDelta` |
| 20027 | `SetBuffer` |
| 20028 | `SetHeatingOffOutdoorTemp` |
| 20029 | `SetExternalControl` |
| 20030 | `SetExternalError` |
| 20031 | `SetExternalCompressorControl` |
| 20032 | `SetExternalHeatCoolControl` |
| 20033 | `SetBivalentControl` |
| 20034 | `SetBivalentMode` |
| 20035 | `SetBivalentStartTemp` |
| 20036 | `SetBivalentAPStartTemp` |
| 20037 | `SetBivalentAPStopTemp` |
| 20038 | `SetForceHeater` |
| 20039 | `SetHeatingControl` |
| 20040 | `SetSmartDHW` |
| 20041 | `SetQuietModePriority` |
| 20042 | `SetPumpFlowrateMode` |
| 20043 | `SetDHWSensorSelection` |
| 20044 | `SetDHWHeaterState` |
| 20045 | `SetRoomHeaterState` |
| 20046 | `SetHeaterOnOutdoorTemp` |

## Optional PCB commands

Optional command IDs are explicitly assigned in `HeishaMon/ModbusRegisterMap.h`.
Their address is 21000 + ID. Enabling optional PCB emulation is still required
for the corresponding heat-pump command handlers.

| Address | Command |
| --- | --- |
| 21000 | `SetHeatCoolMode` |
| 21001 | `SetCompressorState` |
| 21002 | `SetSmartGridMode` |
| 21003 | `SetExternalThermostat1State` |
| 21004 | `SetExternalThermostat2State` |
| 21005 | `SetDemandControl` |
| 21006 | `SetPoolTemp` |
| 21007 | `SetBufferTemp` |
| 21008 | `SetZ1RoomTemp` |
| 21009 | `SetZ1WaterTemp` |
| 21010 | `SetZ2RoomTemp` |
| 21011 | `SetZ2WaterTemp` |
| 21012 | `SetSolarTemp` |
| 21013 | `SetOptPCBByte9` |

## System commands

| Address | Command |
| --- | --- |
| 22000 | `SetReset` |

## Migration from the previous fork

| Previous mapping | Map v2 |
| --- | --- |
| Main integer TOPn at n | Unchanged: n |
| Extra integer 500+n | 1000+n |
| Optional integer 600+n | 2000+n |
| Main float TOP0-TOP138 at 10000+2*n | Unchanged |
| New main float TOP139-TOP143 at 11000+2*(n-139) | 10000+2*n (10278-10287) |
| Extra float 10278+2*n | 12000+2*n |
| Optional float 10290+2*n | 14000+2*n |
| Main command 1000+ID | 20000+ID-1 (except SetReset) |
| SetReset 1100 | 22000 |
| Optional command 2000+ID | 21000+ID |
| Coils 0/1/2 all controlled relay 1 | 0 = relay 1; 1 = relay 2; 2 invalid |

Also review integer scaling as described above. The included Loxone XML template
uses map v2; screenshots and old binary releases may show legacy addresses.
Read register 9000 before enabling command writes to verify the map version is 2.

## Extension rules

1. Append topic numbers within their existing group; never renumber or reuse a topic.
2. Keep block bases fixed. Compile-time checks reject more than 1,000 topics per group.
3. Give each new command a permanent, unused ID. Keep ID 100 reserved for SetReset.
4. Add optional command IDs explicitly, regardless of upstream table order.
5. Update the documentation and run `python tests/modbus/run_tests.py` and the firmware build.
6. Changing an assigned address, its meaning or scaling requires a new map version.
