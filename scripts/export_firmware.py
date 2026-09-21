"""Copy a successful firmware build into the repository's release directory."""

import argparse
import hashlib
from pathlib import Path
import re


def export_firmware(project_dir, source, target):
    project_dir = Path(project_dir)
    version_header = (project_dir / "HeishaMon/version.h").read_text(encoding="utf-8")
    match = re.search(r'^#define HEISHAMON_VERSION "([A-Za-z0-9._-]+)"$', version_header, re.M)
    if not match:
        raise ValueError("HEISHAMON_VERSION must contain a filename-safe version string")
    version = match.group(1)
    model, chip = {"esp32": ("large", "esp32"), "esp8266": ("small", "d1")}[target]
    payload = Path(source).read_bytes()
    if not payload:
        raise ValueError("Firmware binary is empty")

    directory = project_dir / "binaries" / f"model-type-{model}"
    directory.mkdir(parents=True, exist_ok=True)
    binary = directory / f"HeishaMon.ino.{chip}-v{version}.bin"
    checksum = binary.with_suffix(".md5")
    digest = hashlib.md5(payload).hexdigest()
    for destination, content in ((binary, payload), (checksum, digest.encode("ascii"))):
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_bytes(content)
        temporary.replace(destination)
    print(f"Firmware exported: {binary}")
    print(f"MD5: {digest}")
    return binary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--target", choices=("esp32", "esp8266"), required=True)
    arguments = parser.parse_args()
    export_firmware(Path(__file__).resolve().parents[1], arguments.source, arguments.target)
