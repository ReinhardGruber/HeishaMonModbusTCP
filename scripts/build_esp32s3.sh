#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../HeishaMon"
arduino-cli compile --output-dir . \
  --fqbn=esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=enabled,PartitionScheme=min_spiffs \
  --warnings=none --verbose HeishaMon.ino
python3 ../scripts/export_firmware.py --source HeishaMon.ino.bin --target esp32
