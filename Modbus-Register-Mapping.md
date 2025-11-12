# Modbus Register Mapping

The Modbus TCP server exposes the existing MQTT topics without duplicating data. Registers are grouped into ranges that map directly to the topic numbering already used throughout HeishaMon:

| Register range | Contents | Notes |
| --- | --- | --- |
| `0` – `138` | Main topics `TOP0` – `TOP138` | Register equals the topic number from [MQTT-Topics.md](MQTT-Topics.md). |
| `500` – `505` | Extra topics `XTOP0` – `XTOP5` | Values provided by the extra buffer. |
| `600` – `606` | Optional PCB topics `OPT0` – `OPT6` | Available when the optional PCB is enabled. |
| `1000` – `1033` | Write commands `Set…` | Forwarded to the regular command handlers. |
| `2000` – `2013` | Optional PCB write commands | Forwarded to the optional command handlers. |

## Reading registers

Reading a holding register returns the latest value that HeishaMon already keeps in memory for the matching topic. Values are scaled exactly as published on MQTT. Non-numeric topics produce an `ILLEGAL_DATA_VALUE` response once and are logged for reference.

This Topics are scalled x100 !!!

    bool isTopicScale100(unsigned int topicNumber) {
    const char **description = (const char **)pgm_read_ptr(&topicDescription[topicNumber]);
    return 
        (description == Celsius) || 
        (description == Kelvin) ||
        (description == LitersPerMin) ||
        (description == Pressure) ||
        (description == Bar);
    }


## Writing registers

Writing a single register dispatches to the same command handler that is used for MQTT `Set…` topics. JSON commands (currently only `SetCurves`) are rejected with `ILLEGAL_DATA_VALUE` because they cannot be represented inside a 16-bit register payload.

## Error handling

* Requests outside the ranges listed above respond with `ILLEGAL_DATA_ADDRESS`.
* Writing a register that resolves to a JSON-only command responds with `ILLEGAL_DATA_VALUE`.

This file documents the static mapping that is implemented in `HeishaMon/HeishaModBusServer.cpp` so future changes can keep the Modbus and MQTT topic numbering consistent.