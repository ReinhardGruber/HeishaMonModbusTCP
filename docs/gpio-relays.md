# GPIO and relays

[← Documentation index](README.md)

## Large board relay control

Relay output states are published as `0` (off) or `1` (on), with MQTT retain, to `<base>/gpio/relay/one/state` and `<base>/gpio/relay/two/state`. The default base is `panasonic_heat_pump`. A state is published whenever the output changes, whatever changed it (MQTT, REST, Modbus or a rule), and all states are published again after each MQTT (re)connect. Failed publications are retried. These values report the GPIO output levels, not mechanical contact feedback. Send commands to the topics without `/state`; HeishaMon never acts on its own `/state` topics, so a state publish cannot loop back as a command. See [GPIO states and restore at boot](#gpio-states-and-restore-at-boot) for how the outputs are restored after a reboot.

The newer, large, heishamon contains two onboard relays which can be switched on and off using MQTT commands. The relays can be used for any contact switching, even 230V mains (max 5A). For example to switch the 230V contacts in the heatpump for controlling the 'external thermostat', switching a pump on or off or other lower power devices. I do not recommend to use the relay as a switch for a electric heater as they use too much power. To control the relay just send a value of 1 or 0 to the MQTT topic "panasonic_heat_pump/gpio/relay/one" for relay one or "panasonic_heat_pump/gpio/relay/two" for relay two.

The ESP32 large board also supports relay control through the existing HTTP REST `/command` endpoint (GET):

| Action | Request |
| --- | --- |
| Relay 1 on | `http://<HeishaMon-IP>/command?SetRelayOne=1` |
| Relay 1 off | `http://<HeishaMon-IP>/command?SetRelayOne=0` |
| Relay 2 on | `http://<HeishaMon-IP>/command?SetRelayTwo=1` |
| Relay 2 off | `http://<HeishaMon-IP>/command?SetRelayTwo=0` |

Both parameters may be combined, for example `/command?SetRelayOne=1&SetRelayTwo=0`. The plain-text response reports the resulting output level, for example `SetRelayOne: 1`. As with MQTT relay commands, `1`, `true`, `on`, and `enable` turn the relay on (text is case-insensitive); other values turn it off. REST switching works without an MQTT connection and publishes retained relay states when MQTT is connected. These commands are only available on ESP32.

Read the output states with `GET http://<HeishaMon-IP>/gpio` (or the `gpio` list in `/json`): the relays are the entries with `name` `relay/one` and `relay/two`, `state` is `0` / `1`. A single relay is `GET /json?gpio=relay/one`. These are GPIO output levels, not mechanical contact feedback.

## User-configurable extra GPIO
In addition to the fixed relay pins, HeishaMon exposes a number of user-configurable GPIO pins that can each be independently set to Input (pull-up), Input, or Output mode. Configure them in the settings web menu, under the **Extra GPIO** panel.

**Available pins per platform:**
- **ESP8266:** 3 extra GPIOs (pins 1, 3, 16)
- **ESP32:** 5 extra GPIOs (pins 33–37); relay pins 21 and 47 remain fixed

Each pin is listed in the panel as "GPIO 1", "GPIO 2", etc., with its physical pin number shown alongside (e.g. "GPIO 1 (pin 1)"). The chosen mode is saved in settings and (re-)applied on boot and whenever settings are saved, so a pin already wired as an input can be safely repurposed as an output (or vice versa) without a firmware reflash.

**Controlling output GPIOs over MQTT:**
For pins set to Output, send `1`, `0`, `on`, `off`, `true`, or `false` to:
```
panasonic_heat_pump/gpio/extra/N
```
where `N` is the 1-based index of the extra GPIO shown in the settings panel (e.g. `extra/1` for "GPIO 1"), not its physical pin number. Values sent to a pin that isn't configured as Output are ignored.

**Reading GPIO states over MQTT:**
Both input and output pin states are published (retained) to `panasonic_heat_pump/gpio/extra/N/state` whenever a state changes, and for all extra GPIOs after each MQTT (re)connect. Input states are also resent at the configured full-data resend interval (`updateAllTime`). Output states are not resent periodically; they only change when switched.

### GPIO states and restore at boot
The relays and the extra GPIOs work the same way: commands go to `<base>/gpio/relay/one`, `relay/two` or `extra/N`, and the actual level is published retained to the same topic with `/state` added. The state is read from the pin itself, so changes made by rules (the `gpio()` function), REST, the web page or Modbus are published too.

After a reboot, HeishaMon restores its outputs from their retained `/state` topics. On the first MQTT connect after boot it waits up to 5 seconds for the retained states and switches each output back to its last known level before publishing any GPIO state. An output is not restored when it was already switched after boot: by a command (MQTT, REST, web or Modbus) or by a rule that changed its level, for example in `System#Boot`. Later MQTT reconnects do not restore anything; they only publish the current states again.

Retained commands still work as before: if you publish a command topic with retain, the broker sends it again on every reconnect and HeishaMon applies it. To keep the state of a rule-controlled output, publish commands without retain.

**HTTP API:**
You can read all GPIO states (the relays on the large board and the extra GPIOs) as JSON via a GET request:
```
GET /gpio
```
Response example:
```json
[{"pin":33,"mode":2,"state":1,"name":"extra/1","topic":"panasonic_heat_pump/gpio/extra/1/state"},{"pin":34,"mode":0,"state":0,"name":"extra/2","topic":"panasonic_heat_pump/gpio/extra/2/state"}, ...]
```
where `pin` is the physical pin number, `mode` is 0 = Input (pull-up), 1 = Input, 2 = Output, `name` is the MQTT name and `topic` the MQTT state topic.

To set an output pin, use its physical pin number (as in the `pin` field above):
```
GET /gpio?pin=33&value=1
```
`value` takes the same values as the MQTT command (`1`, `true`, `on`, `enable` switch on; anything else switches off). The arguments may be in any order. If one is missing, or the pin is not an available output, the reply is HTTP 400 with an `error` message; otherwise the GPIO list as above.

**Using extra GPIOs in rules:**
The `gpio()` rules function works with the extra GPIO pins by their physical pin number, the same as the relay pins (see the `gpio` function in the [rules engine](rules.md#functions)).
