# OpenTherm

[← Documentation index](README.md)

If your heishamon board supports opentherm the software can also be used to bridge opentherm information from a compatible thermostat to your home automation over MQTT or JSON and as mentioned above it can also be connected directly in the rules to connect opentherm information to the heatpump and back, for example to display the outside temperature from the heatpump on your opentherm thermostat. If you enable opentherm support in settings there will be a new tab visible in the web page. On that tab you will see opentherm values. Some are of type R(ead) and some are W(rite), and some are both. Read means that the thermostat can read that information from the heishamon. You provide that information over MQTT (or using the rules) by updating this value on the mqtt 'opentherm/read' topic, for example 'panasonic_heat_pump/opentherm/read/outsideTemp'. The write values are information from the thermostat, like 'roomTemp'. These are available on mqtt topic 'opentherm/write'. You can use these values to change the heatpump behaviour in anyway you want using your home automation and mqtt set-commands to heishamon on using the internal rules.

The available opentherm variables are:
### WRITE values
- chEnable which is a boolean showing if central heating shoud be enabled. This is often used when the thermostat wants to heat up your house.
- dhwEnable which is a boolean showing if the dhw heating should be enabled. Often used as a user option on the thermostat to disable DHW heating during vacation
- coolingEnable which is a boolean showing if  cooling should be enabled. Amount of cooling is request in 'coolingControl', see below.
- roomTemp is the floating point value of the measured room temp by thermostat
- roomTempSet is the floating point value of the requested room temp setpoint on the thermostat
- chSetpoint is the floating point value of the calculated water setpoint by thermostat. Opentherm thermostats try to set this chSetpoint to not overshoot the room setpoint. Could be used to set the water setpoint on the heatpump but most thermostats are too fast responding compared to how heatpumps work
- maxRelativeModulation is the amount of modulation (0-100%) the heatpump (opentherm slave) is allowed to use (see relativeModulation in READ values, which should always be equal or lower than this max)
- coolingControl is the amount of cooling (0-100%) the thermostat requests from the heatpump. Requires an opentherm thermostat with cooling support.
### READ AND WRITE values
- dhwSetpoint is the floating point value which is the current DHW setpoint by thermostat, but can also be set by heishamon to override it. Not all thermostat support this though. It should not be set higher than dhwSetUppBound, see below.
- maxTSet is the floating point value which defines the maximum water setpoint. The user can set this on the thermostat or can also set from heishamon. It should not be set higher than chSetUppBound, see below.
### READ values
- chPressure is the floating point value which defines measured water pressure of central heating provided by heishamon
- outsideTemp is the floating point value which defines measured outside temperature of central heating provided by heishamon
- inletTemp is the floating point value which defines measured water inlet temperature of central heating provided by heishamon
- outletTemp is the floating point value which defines measured water outlet temperature of central heating provided by heishamon
- dhwTemp is the floating point value which defines measured dhw temperature of central heating provided by heishamon
- relativeModulation is the amount (0-100%) of modulation the heatpump (opentherm slave) is currently running on, should always be lower or equal than the maxRelativeModulation set by the thermostat
- flameState is a boolean value (send 'true', 'on' or '1' to enable) which defines if the central heating is providing heat central
- chState is a boolean value (send 'true', 'on' or '1' to enable) which defines if the heatpump is on room/central heating mode (for example 3-way valve on room, in heating mode)
- dhwState is a boolean value (send 'true', 'on' or '1' to enable) which defines if the heatpump is on DHW mode (for example 3-way valve on dhw)
- coolingState is a boolean value (send 'true', 'on' or '1' to enable) which defines if the heatpump is on room/central cooling mode (for example 3-way valve on room, in cooling mode)
- dhwSetUppBound is a integer value from 0 to 127 which sets the max DHW temperature supported so the thermostat can not request a dhwSetpoint higher than this. Default is set to 75. To override, send a MQTT message to this topic and make it retained so heishamon receives it again after reboot.
- dhwSetLowBound is a integer value from 0 to 127 which sets the min DHW temperature supported so the thermostat can not request a dhwSetpoint lower than this. Default is set to 40. To override, send a MQTT message to this topic and make it retained so heishamon receives it again after reboot.
- chSetUppBound is a integer value from 0 to 127 which sets the max CH (heating water) temperature supported so the thermostat can not request a chSetpoint higher than this. Default is set to 65. To override, send a MQTT message to this topic and make it retained so heishamon receives it again after reboot.
- chSetLowBound is a integer value from 0 to 127 which sets the min CH (heating water) temperature supported so the thermostat can not request a chSetpoint lower than this. Default is set to 20. To override, send a MQTT message to this topic and make it retained so heishamon receives it again after reboot.
