# Rules engine

[← Documentation index](README.md)

The rules functionality allows you to control the heatpump from within the HeishaMon itself. Which makes it much more reliable then having to deal with external domotica over WiFi. When posting a new ruleset, it is immidiatly validated and when valid used. When a new ruleset is invalid it will be ignored and the old ruleset will be loaded again. You can check the console for feedback on this. If somehow a new valid ruleset crashes the HeishaMon, it will be automatically disabled the next reboot, allowing you to make changes. This prevents the HeishaMon getting into a boot loop.

Rules monitoring is available through two read-only MQTT topics on both ESP8266 and ESP32:

- `<base>/rules/state` is retained and reports `1` when at least one rule block is loaded, or `0` otherwise. It is sent on connection, changes, and the full-data resend interval. The existing `<base>/stats` field `"rules active"` gives the number of loaded blocks. Use `<base>/LWT` to check device availability.
- `<base>/rules/executed` publishes a non-retained JSON event after an execution returns, when **Publish rule executions to MQTT** is enabled in the settings (off by default). It includes the block name and ID, interpreter result, device uptime, and whether it is a nested helper call. Outer invocations also include elapsed microseconds. Validation does not generate events; disconnected events are not replayed.

The default base is `panasonic_heat_pump`. See [rules status](../MQTT-Topics.md#rules-engine-status) and [execution events](../MQTT-Topics.md#rule-execution-events) for the payload format and limitations. These are monitoring topics for MQTT clients, not `@` variables or commands in a ruleset.

The techniques used in the rule library allows you to work with very large rulesets, but best practice is to keep it below 10.000 bytes.

Notice that sending commands to the heatpump is done asynced. So, commands sent to the heatpump at the beginning of your syntax will not immediatly be reflected in the values from the heatpump later on. Therefor, heatpump values should be read from the heatpump itself instead of those based on the values you keep yourself.

## Syntax
Two general rules are that spaces are mandatory and semicolons are used as end-of-line character.

### Variables
The ruleset uses the following variable structure:

- `#`: Globals
These variables can be accessed throughout the ruleset but have to defined inside a rule block. Don't use globals for all your variables, because it will persistently use memory.

- `$`: Locals
These variables live inside a rule block. When a rule block finishes, these variables will be cleaned up, freeing any memory used.

- `@`: Heatpump parameters
These are the same as listed in the Manage Topics documentation page and as found on the HeishaMon homepage. The ruleset also follows the R/W logic as used through the MQTT and REST API. That means that the read topics differ from the write topics. So reading the heatpump state is done through `@Heatpump_State`, changing the heatpump state through `@SetHeatpump`.

- `%`: Datetime variables
These can be used for date and time based rules. Currently `%hour` (0 - 23), `%minute` (0 - 59), `%month` (1 - 12), and `%day` (1 - 7) are supported. All are plain integers. A proper NTP configuration is needed to set the correct system date and time on the HeishaMon.

- `?`: Thermostat parameters
These variables reflect parameters read from the connected thermostat when using the OpenTherm functionality. When OpenTherm is supported this documentation will be extended with more precise information. The can check the opentherm tab for the variables that can be used. The names are the same for reading and writing, but not all values support reading and/or writing. The opentherm tab also lists this.

- `ds18b20#2800000000000000`: Dallas 1-wire temperature values
Use these variables to read the temperature of the connected sensors. These values are of course readonly. The id of the sensor should be placed after the hashtag.

- `s0#watt[hour[total]]_?`: The S0 values for port 1 and 2. Replace the question mark with the port number.
For example s0#watt_1 to get watt from port 1 and s0#watthourtotal_2 to get total watthours from port 2


When a variable is called but not yet set to a value, the value will be `NULL`.

Variables can be of boolean (`1` or `0`), float (`3.14`), integer (`10`), and string type. Defining strings is done with single or double quotes.

### Events or functions
Rules are written in `event` or `function` blocks. These are blocks that are triggered when something happened; either a new heatpump or thermostat value has been received or a timer fired. Or can be used as plain functions

```
on [event] then
  [...]
end

on [name] then
  [...]
end
```

Events can be Heatpump or thermostat parameters or timers:
```
on @Heatpump_State then
  [...]
end

on ?setpoint then
  [...]
end

on timer=1 then
  [...]
end
```

When defining functions, you just name your block and then you can call it from anywhere else:
```
on foobar then
  [...]
end

on @Heatpump_State then
  foobar();
end
```

Functions can have parameters which you can call:
```
on foobar($a, $b, $c) then
  [...]

on @Heatpump_State then
  foobar(1, 2, 3);
end
```

If you call a function less values then the function takes, all other parameters will have a NULL value.

There is currently one special function that calls when the system is booted on when a new ruleset is saved:
```
on System#Boot then
  [...]
end
```

This special function can be used to initially set your globals or certain timers.

### Operators
Regular operators are supported with their standard associativity and precedence. This allows you to also use regular math.
- `&&`: And
- `||`: Or
- `==`: Equals`
- `!=`: Not Equals`
- `>=`: Greater or equal then
- `>`: Greater then
- `<`: Lesser then
- `<=`: Lesser or equal then
- `-`: Minus
- `%`: Modulus
- `*`: Multiply
- `/`: Divide
- `+`: Plus
- `^`: Power

Parenthesis can be used to prioritize operators as it would work in regular math.

### Functions
- `coalesce`
Returns the first value not `NULL`. E.g., `$b = NULL; $a = coalesce($b, 1);` will return 1. This function accepts an unlimited number of arguments.

- `max`
Returns the maximum value of the input parameters.

- `min`
Returns the minimum value of the input parameters.

- `isset`
Returns `1` when the input variable has a numeric value and `0` when it is still `NULL` (a string value also returns `0`). Use it to test for `NULL`: a comparison with `NULL` (e.g. `#g == NULL`) always results in `NULL`, which is false, so write `if isset(#g) == 0 then` instead.

- `round`
Rounds the input float to the nearest integer.

- `floor`
The largest integer value less than or equal to the input float.

- `ceil`
The smallest integer value greater than or equal to the input float.

- `setTimer`
Sets a timer to trigger in X seconds. The first parameter is the timer number and the second parameters the number of seconds before it fires. A timer only fires once so it has to be re-set for recurring events. When a timer triggers it will can the timer event as described above. E.g.

- `print`
Prints a value to the console.

- `concat`
Concatenates various values into a combined string. E.g.: `@SetCurves = concat('{zone1:{heat:{target:{high:', @Z1_Heat_Curve_Target_High_Temp, ',low:32}}}}');`

- `gpio`
Allows setting or getting a GPIO state. When called with a single argument, a GPIO state is returned. When called with two arguments the state of a GPIO is set. This function only sets digital pins so the state can only be 0 or 1. The two relays on the large heishamon are gpio21 and gpio47. The user-configurable extra GPIOs (see [GPIO and relays](gpio-relays.md)) can also be addressed by their pin number. See the example to switch the relays each two seconds.

```
on System#Boot then
   setTimer(10, 2);
end

on timer=10 then
   setTimer(20, 2);
   gpio(21,0);
   gpio(47,1);
end

on timer=20 then
   setTimer(10, 2);
   gpio(21,1);
   gpio(47,0);
end
```

### Conditions
The only supported conditions are `if`, `else`, and `elseif`:

```
if [condition] then
  [...]
else
  if [condition] then
    [...]
  end
end
```

```
if [condition] then
  [...]
elseif [condition] then
  if [condition] then
    [...]
  else
    [...]
  end
elseif [condition] then
  [...]
else
  [...]
end
```

### Examples
Complete, documented example rulesets live in [`Examples/Rules/`](../Examples/Rules/) — each comes with a README explaining its design and a `tests/` directory of regression scenarios. The scenarios run on your PC (no device needed) through the host-side harness in [`Examples/Rules/harness/`](../Examples/Rules/harness/), which compiles the real HeishaMon rules engine with plain `g++` and performs the same validation as an on-device upload, then simulates topic changes and timers while checking the commands your ruleset sends. Run all tests with `Examples/Rules/run_tests.sh`, or validate your own ruleset before uploading with `harness/harness <rules.txt> parse` — see the harness README for the scenario language.

Once the rules system is in used by more and more users, additional examples will be added to the documentation.

*Calculating WAR*
```
on calcWar($Ta1, $Tb1, $Ta2, $Tb2) then
	#maxTa = $Ta1;

	if @Outside_Temp >= $Tb1 then
		#maxTa = $Ta1;
	elseif @Outside_Temp <= $Tb2 then
		#maxTa = $Ta2;
	else
		#maxTa = $Ta1 + (($Tb1 - @Outside_Temp) * ($Ta2 - $Ta1) / ($Tb1 - $Tb2));
	end
end
```

*Thermostat setpoint*
```
on ?roomTemp then
	calcWar(32, 14, 41, -4);

	$margin = 0.25;

	if ?roomTemp > (?roomTempSet + $margin) then
		if @Heatpump_State == 1 then
			@SetHeatpump = 0;
		end
	elseif ?roomTemp < (?roomTempSet - $margin) then
		if @Heatpump_State == 0 then
			@SetHeatpump = 1;
		end
	else
		@SetZ1HeatRequestTemperature = round(#maxTa);
	end
end
```
