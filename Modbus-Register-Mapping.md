# Modbus Register Mapping

The Modbus TCP server exposes the existing MQTT topics without duplicating data. Registers are grouped into ranges that map directly to the topic numbering already used throughout HeishaMon:

| Register range | Contents | Notes |
| --- | --- | --- |
| `0` – `138` | Main topics `TOP0` – `TOP138` | Register equals the topic number from [MQTT-Topics.md](MQTT-Topics.md). |
| `500` – `505` | Extra topics `XTOP0` – `XTOP5` | Values provided by the extra buffer. |
| `600` – `606` | Optional PCB topics `OPT0` – `OPT6` | Available when the optional PCB is enabled. |
| `1000` – `1033` | Write commands `Set…` | Forwarded to the regular command handlers. |
| `2000` – `2013` | Optional PCB write commands | Forwarded to the optional command handlers. |

| `10000` – `10277` | Main topics as IEEE 754 floats | Register `10000 + 2·TOPn` holds the MSW, the following register the LSW. |
| `10278` – `10289` | Extra topics as IEEE 754 floats | Register `10278 + 2·XTOPn` holds the MSW, the following register the LSW. |
| `10290` – `10303` | Optional PCB topics as IEEE 754 floats | Register `10290 + 2·OPTn` holds the MSW, the following register the LSW. |


## Reading registers

Reading a holding register returns the latest value that HeishaMon already keeps in memory for the matching topic. Values are scaled exactly as published on MQTT. Non-numeric topics produce an `ILLEGAL_DATA_VALUE` response once and are logged for reference.

For compatibility with existing PLC mappings the 16-bit registers continue to use the historic **×100** scaling for temperatures, flow, and pressure topics.

If you prefer raw floating-point values, use the float ranges listed above. Each topic consumes **two consecutive registers**: the even address contains the most-significant word (MSW) and the following address the least-significant word (LSW). The start address within a range is simply `BASE + 2·TOPIC_NUMBER`, so topic `TOP23` is available both at register `23` (scaled integer) and at registers `10046`/`10047` (32-bit float) without further calculations.

## Writing registers

Writing a single register dispatches to the same command handler that is used for MQTT `Set…` topics. JSON commands (currently only `SetCurves`) are rejected with `ILLEGAL_DATA_VALUE` because they cannot be represented inside a 16-bit register payload.

## Error handling

* Requests outside the ranges listed above respond with `ILLEGAL_DATA_ADDRESS`.
* Writing a register that resolves to a JSON-only command responds with `ILLEGAL_DATA_VALUE`.

This file documents the static mapping that is implemented in `HeishaMon/HeishaModBusServer.cpp` so future changes can keep the Modbus and MQTT topic numbering consistent.