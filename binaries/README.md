This directory contains released and test versions for the HeishaMon software.

The latest production release is v4.2.2.

Make sure you use the correct firmware for your model. The 'small' version is for the HeishaMon PCB which has the ESP8266 chip. The 'large' version is for the newer ESP32-S3 HeishaMon PCB. You can recognize the 'large' version as the one with optional ethernet support and the two blue relays. The large board is about 10x10cm while the small version is about 4x4cm (without OT) or 4x5cm(with OT).

If you upload the wrong firmware you can brick your HeishaMon and you need to recover it using a USB-TTL cable for the small version and a USB-C cable for the large version.


## Modbus TCP fork builds

PlatformIO (`pio run -e esp32`) and the scripts in `scripts/build_*.sh` automatically
copy successful builds here with the version from `HeishaMon/version.h`:

- ESP32: `model-type-large/HeishaMon.ino.esp32-v4.2.2-ModbusTCP.bin`
- Matching checksum: `model-type-large/HeishaMon.ino.esp32-v4.2.2-ModbusTCP.md5`

The `.md5` file contains the 32-character MD5 digest used by the firmware updater.
These are application binaries for firmware updates, not merged USB flash images.
An existing build of the same version is replaced; other releases are retained.
PlatformIO also refreshes the exported files when the build is already up to date.
Arduino ESP8266 builds use `model-type-small/HeishaMon.ino.d1-v<VERSION>.bin`
and `.md5`; that target does not provide Modbus TCP.

The plain upstream binaries without the `ModbusTCP` suffix do not contain the
fork's Modbus server. The Modbus build uses register map v2; see the
[register map and migration guide](../Modbus-Register-Mapping.md).
