# Modbus TCP

[← Documentation index](README.md)

The large (ESP32-S3) heishamon can act as a Modbus TCP server (port 502, unit ID 1) so PLCs and building automation systems such as Loxone can read the heat pump values with FC03 and, optionally, send commands with FC06 and switch the relays with FC05. Modbus has no authentication, so it is disabled by default. Enable it in settings under 'Enable Modbus TCP server' and only additionally enable 'Allow Modbus writes' if you want to be able to control the heat pump over Modbus. Changes take effect when the settings are saved, without a reboot. The register map is documented in [Modbus register mapping](../Modbus-Register-Mapping.md) and is also available on the device on the 'Modbus' page. A Loxone template is available in the [Integrations/Loxone](../Integrations/Loxone) directory.
