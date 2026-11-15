# Sensors: 1-wire and S0

[← Documentation index](README.md)

## DS18B20 1-wire
The software also supports ds18b20 1-wire temperature sensors reading. A proper 1-wire configuration (with 4.7kohm pull-up resistor) connected to GPIO4 will be read each configured secs (minimal 5) and send at the panasonic_heat_pump/1wire/"sensor-hex-address" topic. On the pre-made boards this 4.7kohm resistor is already installed.

### Calibration offset

Each sensor can get a calibration offset (in °C, steps of 0.1, range -10.0 to +10.0) which is added to every raw reading. The corrected value is what is published to MQTT, shown on the web page, returned in the JSON output and used by the rules (`ds18b20#...`). The offset can be set:

- in the web interface, in the "Offset" column of the 1-wire sensor table
- over HTTP: `http://heishamon.local/dallasoffset?<sensor-hex-address>=0.3`
- over MQTT: publish the value to `panasonic_heat_pump/1wire/<sensor-hex-address>/offset/set` (both `0.3` and `0,3` are accepted)

The active offset is published (retained) on `panasonic_heat_pump/1wire/<sensor-hex-address>/offset` and returned as `Offset` in the `/json` output. Offsets are stored in `/dallasoffset.json` on the device and survive a reboot. A typical calibration: put all sensors together in one well-stirred container of water, compare them against a reference thermometer (or against their average) and set the difference as offset.

## S0 kWh meters

The software is also able to measure Watt on a S0 port of two kWh meters. You only need to connect GPIO12 and GND to the S0 of one kWh meter and if you need a second kWh meter use GPIO14 and GND. It will report on MQTT topic panasonic_heat_pump/s0/Watt/1 and panasonic_heat_pump/s0/Watt/2 and also in the JSON output. You can replace 'Watt' in the previous topic with 'Watthour' to get consumption counter in WattHour (per mqtt message) or to 'WatthourTotal' to get the total consumption measured in WattHour. To sync the WatthourTotal with your kWh-meter, publish the correct value to MQTT to the panasonic_heat_pump/s0/WatthourTotal/1 or panasonic_heat_pump/s0/WatthourTotal/2 topic with the 'retain' option while heishamon is rebooting. Upon reboot, heishamon reads this value as the last known value to you can sync using this method.
