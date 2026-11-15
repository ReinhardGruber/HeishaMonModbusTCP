# Getting started

[← Documentation index](README.md)

## 1. Get the firmware

The latest release is available on the [releases page](https://github.com/Egyras/HeishaMon/releases). Pick the binary that matches your board:

| Board | Chip | Binary |
|---|---|---|
| HeishaMon small PCB, Wemos D1 mini, any ESP8266 board with at least 4 MB flash | ESP8266 | `model-type-small` |
| HeishaMon large PCB (optional Ethernet, two relays, optional OpenTherm) | ESP32-S3 | `model-type-large` |

You can also build the firmware yourself, see [Hardware and connection](hardware.md#building-the-arduino-image-yourself). When compiling for the ESP8266, select a flash layout with a filesystem.

HeishaMon works with the Panasonic Aquarea H, J, K, L and M series. [Heat pump types confirmed by users](../HeatPumpType.md).

## 2. Connect it to the heat pump

Plug the HeishaMon into the CN-CNT (or CN-NMODE) socket of the heat pump. It is powered from the heat pump over that cable. Pin-outs, cable length, connectors and the long-distance variant are described in [Hardware and connection](hardware.md).

If you want to keep the original Panasonic CZ-TAW1 module, switch HeishaMon to *Listen only* mode. The trade-off is that it can then neither send commands nor use the optional PCB.

## 3. First start and configuration

Without a configured WiFi network, HeishaMon opens an open WiFi hotspot. Join it and open <http://192.168.4.1> to configure your WiFi network and your MQTT server.

After configuring and rebooting, HeishaMon reads from and talks to your heat pump. On the ESP8266 the GPIO13/GPIO15 connection is used for communication, so you can keep a computer or uploader connected to the board. Serial 1 (GPIO2) can be connected (GND and TX of the board only) to read debug data.

Afterwards the web interface is available at `http://heishamon.local` (or the IP address of the device).

## 4. What you get

- **MQTT:** all received data is published on separate topics below `panasonic_heat_pump/`, see [MQTT topics](../MQTT-Topics.md). A `panasonic_heat_pump/log` topic provides debug logging and a hexdump of received packets (if enabled in the web portal). Commands go back through the set topics, see [Manage topics](../Manage-Topics.md).
- **JSON and REST:** [JSON output and single values](json-api.md).
- **Extra data block:** some heat pumps (K/L series and newer) also send an additional block with more precise power values. HeishaMon detects this at boot and, if supported, publishes it as XTOP topics under `panasonic_heat_pump/extra/...` (full list in [MQTT-Topics.md](../MQTT-Topics.md)). On such heat pumps the "old" power topics (TOP15/16, TOP38-41) are no longer populated by the heat pump and report invalid values (for example -200), so use the XTOP power values instead.
- **Automation without a server:** the [rules engine](rules.md).
- **Home automation:** [integration examples](integrations.md).

## Updating the firmware

Open the firmware menu in the web interface and, after authenticating with user `admin` and password `heisha` (or the password you set during setup), upload the new binary.

## Factory reset

A factory reset can be performed in the web interface. If the web interface is unavailable (no WiFi or another reason) you can perform a hard factory reset. With recent firmware, hold the boot/flash button for more than 10 seconds. With older firmware, do a double reset: not too fast and not too slow, usually about half a second between both resets. The blue LED flashes rapidly to indicate that the factory reset was performed. Press reset again to restart HeishaMon normally, after which the WiFi hotspot should be visible again.
