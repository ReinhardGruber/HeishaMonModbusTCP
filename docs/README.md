# HeishaMon documentation

[← Back to the project page](../README.md)

## Set up

| Page | Contents |
|---|---|
| [Getting started](getting-started.md) | Choose firmware, connect, first configuration, update, factory reset |
| [Hardware and connection](hardware.md) | Pin-outs, long-distance cabling, connectors, the PCB, building the firmware |
| [Heat pump types](../HeatPumpType.md) | Models confirmed by users |

## Use it

| Page | Contents |
|---|---|
| [MQTT topics](../MQTT-Topics.md) | All published topics |
| [Manage topics](../Manage-Topics.md) | Settable values and how to add new ones |
| [JSON output and REST](json-api.md) | `/json`, single values, `/command` |
| [Rules engine](rules.md) | Automate the heat pump on the device itself; [example rulesets](../Examples/Rules/) |
| [Integrations](integrations.md) | Home Assistant, openHAB, ioBroker, Domoticz, Node-RED, Loxone |

## Features and extensions

| Page | Contents |
|---|---|
| [Sensors: 1-wire and S0](sensors.md) | DS18B20 temperature sensors, calibration offset, S0 kWh meters |
| [GPIO and relays](gpio-relays.md) | Relays on the large board, extra GPIOs, restore at boot |
| [Modbus TCP](modbus.md) | Modbus server on the large board; [register map](../Modbus-Register-Mapping.md) |
| [OpenTherm](opentherm.md) | Bridge an OpenTherm thermostat |
| [Optional PCB](../OptionalPCB.md) | Emulating the optional Panasonic PCB |
| [MQTT TLS](mqtt-tls.md) | Encrypted MQTT connection (ESP32) |

## Good to know

| Page | Contents |
|---|---|
| [Reliability and EEPROM wear](reliability.md) | Dropped commands, retry logic, how often to write |

## Protocol reference

| Page | Contents |
|---|---|
| [Protocol bytes](../ProtocolByteDecrypt.md) | Main data block |
| [Protocol bytes, extra block](../ProtocolByteDecrypt-extra.md) | Extra data block (K/L series) |
| [Aquarea parity](../AquareaParity.md) | Installer-menu settings located in the datagrams, evidence and open tasks |
