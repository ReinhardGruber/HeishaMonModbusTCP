## HeishaMon – Modbus-Enabled Fork

This project is a fork of the original [Egyras/HeishaMon](https://github.com/Egyras/HeishaMon), extended with **Modbus** integration for **low-level PLC/SCADA control** of Panasonic *Aquarea / Heisha / Jeisha / Meisha* heat pumps.

### Why this fork?
- Native **Modbus/TCP** access to Heisha telemetry and settings
- Built for **industrial PLCs** and **home/Building automation**
- Stable, low-latency interface for automation tasks

### Key features
- **Modbus server** exposing live metrics and writable settings  
  (temps, states, duty cycles, modes, setpoints, etc.)
- Readable & writable registers with **bounds checking**
- Deterministic polling/update cycle suitable for PLC logic

### Works with (tested/targeted)
- **Siemens LOGO!** (8.x) via Modbus/TCP (client)
- **Eaton easyE4** via Modbus/TCP (client)
- **Loxone** (Modbus TCP client or Modbus Extension via gateway)
- Generic SCADA/HMI tools (e.g., QModMaster, Kepware, Node-RED Modbus)

### Quick start
1. Flash firmware and set network (static IP recommended).
2. Point your PLC/BCU/SCADA Modbus **client** to `HOST:PORT` (default port in firmware).
3. Read a few safe **holding/input registers** (e.g., supply/return temps) to verify.
4. Enable writes (if required), then test with a **non-critical** setpoint first.

### Safety
- Keep write access **disabled** until your logic is proven.
- Validate ranges in the PLC and ramp conservatively.
- Start read-only monitoring in production, then enable writes.

### Documentation
- **Register map:** [Modbus Register Mapping](Modbus-Register-Mapping.md)
- Upstream project: [Egyras/HeishaMon](https://github.com/Egyras/HeishaMon)

---

> If you find a mismatch between firmware fields and the Modbus map, open an issue with device model, firmware version, and a short capture.
