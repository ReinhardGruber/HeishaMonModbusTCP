# JSON output and REST

[← Documentation index](README.md)

A JSON output of heat pump data and enabled 1-wire/S0 sensors is available at http://heishamon.local/json (replace heishamon.local with the IP address of your HeishaMon device if mDNS is not working). On the ESP32 large board, the two relays are the `relay/one` and `relay/two` entries of the `gpio` list. See [relay control](gpio-relays.md#large-board-relay-control) for reading and switching those outputs.

To get a single value, add one of these arguments (one per request):

| Argument | Value | Example | Returns |
|---|---|---|---|
| `topic=` | a heat pump topic number (`TOP1`, `XTOP3`, `OPT0`) or name (`Main_Outlet_Temp`, case-insensitive) | `/json?topic=Main_Outlet_Temp` | `{"Topic":"TOP6","Name":"Main_Outlet_Temp","Value":35,"Description":"°C"}` |
| `opentherm=` | an OpenTherm value name | `/json?opentherm=roomTemp` | `{"roomTemp":{"type": "W","value":21.50}}` |
| `1wire=` | a sensor address or alias | `/json?1wire=28ff123456789abc` | the sensor's entry from the `1wire` list |
| `s0=` | the S0 port, `1` or `2` | `/json?s0=1` | the port's entry from the `s0` list, without `Watthour` |
| `gpio=` | the GPIO's MQTT name, case-insensitive (`extra/1`, or `relay/one`/`relay/two` on the large board) or just the extra GPIO number | `/json?gpio=1` | the pin's entry from the `gpio` list; the relays are `gpio=relay/one` and `gpio=relay/two` |

`Watthour` in the full output is the energy since the previous `/json` request, so single S0 requests leave it out and don't reset it. An unknown value, or a section that is not enabled, returns HTTP 404. More than one of these arguments in one request returns HTTP 400. A heat pump topic requested before any data has arrived from the heat pump (after boot, or with no heat pump connected) returns HTTP 503.

Commands can be sent with `/command` (GET), for example `/command?SetRelayOne=1` on the large board, see [GPIO and relays](gpio-relays.md). The names of the settable values are listed in [Manage topics](../Manage-Topics.md).
