"""Compile the actual Modbus server with host shims; no device/network is used."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = ROOT / ".pio" / "modbus-tests"
OUT.mkdir(parents=True, exist_ok=True)

# Only heat-pump decoding/command execution is stubbed. The register map, unit
# descriptions, real command tables, request handlers and HTML rows are compiled.
decode = (ROOT / "HeishaMon/decode.h").read_text(encoding="utf-8")
commands = (ROOT / "HeishaMon/commands.h").read_text(encoding="utf-8")
definitions = []
for result, signature in re.findall(r"^(String|unsigned int) ([^;\n]+);", decode + commands, re.M):
    if signature.startswith(("getDataValue(", "getDataValueExtra(", "getOptDataValue(")):
        continue
    definitions.append(f"{result} {signature} {{ return " + ('"0"' if result == "String" else "0") + "; }")
(OUT / "generated_stubs.h").write_text("\n".join(definitions) + "\n", encoding="utf-8")

source = HERE / "test_modbus.cpp"
executable = OUT / ("test_modbus.exe" if os.name == "nt" else "test_modbus")
if os.name == "nt":
    candidates = [p for p in Path("C:/Program Files/Microsoft Visual Studio").glob("*/Community/VC/Auxiliary/Build/vcvars64.bat")
                  if p.with_name("vcvarsall.bat").exists()]
    if candidates:
        batch = OUT / "build.cmd"
        batch.write_text(
            f'@echo off\ncall "{sorted(candidates)[-1]}" >nul\n'
            'if errorlevel 1 exit /b 1\n'
            f'cl /nologo /EHsc /std:c++17 /utf-8 /D ESP32 /D _CRT_SECURE_NO_WARNINGS '
            f'/I"{HERE / "stubs"}" /I"{OUT}" /Fe:"{executable}" /Fo:"{OUT / "test_modbus.obj"}" "{source}"\n'
            'if errorlevel 1 exit /b 1\n',
            encoding="utf-8",
        )
        subprocess.run(["cmd", "/d", "/c", str(batch)], cwd=OUT, check=True)
    else:
        # The .NET WebAssembly workload also provides a complete C++ toolchain.
        # Use its prebuilt, read-only cache when MSVC is not fully installed.
        packs = Path("C:/Program Files/dotnet/packs")
        sdks = sorted(packs.glob("Microsoft.NET.Runtime.Emscripten.*.Sdk.win-x64/*/tools/emscripten/em++.py"))
        if not sdks:
            raise SystemExit("Install MSVC C++ tools or the .NET WebAssembly workload")
        emcc = sdks[-1]
        sdk_tools = emcc.parents[1]
        version = emcc.parents[2].name
        package = emcc.parents[3].name
        def companion(kind):
            return packs / package.replace(".Sdk.", f".{kind}.") / version / "tools"
        node = companion("Node") / "bin/node.exe"
        environment = dict(os.environ, DOTNET_EMSCRIPTEN_LLVM_ROOT=str(sdk_tools / "bin"),
                           DOTNET_EMSCRIPTEN_BINARYEN_ROOT=str(sdk_tools),
                           DOTNET_EMSCRIPTEN_NODE_JS=str(node),
                           EM_CACHE=str(companion("Cache") / "emscripten/cache"))
        javascript = OUT / "test_modbus.js"
        subprocess.run([str(companion("Python") / "python.exe"), str(emcc), "-std=c++17", "-DESP32",
                        "-I" + str(HERE / "stubs"), "-I" + str(OUT), str(source), "-o", str(javascript)],
                       env=environment, check=True)
        subprocess.run([str(node), str(javascript)], check=True)
        executable = None
else:
    compiler = shutil.which("g++")
    if not compiler:
        raise SystemExit("g++ is required")
    subprocess.run([compiler, "-std=c++17", "-DESP32", "-I" + str(HERE / "stubs"),
                    "-I" + str(OUT), str(source), "-o", str(executable)], check=True)
if executable is not None:
    subprocess.run([str(executable)], check=True)

# Check the shipped integration against the new map, including migrated extras.
loxone = ET.parse(ROOT / "Integrations/Loxone/MB_HeishaMon.xml").getroot()
expected = {"Heat_Power_Consumption": 12000, "Heat_Power_Production": 12006,
            "ErrorInformation": 44, "SetHeatpump": 20000,
            "SetZ1HeatRequestTemperature": 20004, "SetOperationMode": 20008,
            "SetQuietMode": 20002, "SetMaxPumpDuty": 20014}
actual = {entry.attrib["Title"]: int(entry.attrib["ModbusAddress"])
          for entry in loxone.findall("ModbusCmd")}
assert all(actual[name] == address for name, address in expected.items())
print("PASS: Loxone map v2 migration")
