# ESP32 Honda Drive-By-Wire Cruise Control

A high-performance, open-source Cruise Control system engineered for Honda vehicles equipped with electronic throttle control (Drive-By-Wire - DBW). Specifically developed, reverse-engineered, and road-tested on the **Honda Fit / Jazz (2nd Generation - GE6 / GE8, 2009–2014)**, with direct compatibility for related Honda platforms (Civic, City, Insight).

The system integrates an **ESP32** microcontroller, an **MCP4728 12-bit I2C DAC**, an **SN65HVD230 / VP230 CAN bus transceiver** interfaced directly to the **Honda F-CAN (500 kbps)** network, and a **DPDT signal safety relay**.

---

## 🚀 Key Features

* **Native Honda F-CAN Bus Integration (500 kbps):**
  * Real-time vehicle speed and engine RPM acquisition via broadcast frame **ID `0x158`**.
  * Direct digital decoding of OEM steering wheel cruise buttons (**CRUISE MAIN**, **SET / DECEL**, **RES / ACCEL**, **CANCEL**) broadcast by the Gauge Control Module (GCM) on **ID `0x164`**. No additional wiring to the steering column or clock spring is required!
  * Sub-millisecond brake pedal switch monitoring broadcast by the ECM on **ID `0x17C`**.
  * Dynamic green **`CRUISE CONTROL`** cluster indicator lamp injection via **ID `0x17C`** frame synthesis.
* **Dual-Slope Electronic Accelerator Pedal (APP) Emulation:**
  * Generates synchronized paired analog curves: **APP1** (0.85V to 4.50V) and **APP2** (0.42V to 2.25V) with a strict 2:1 voltage correlation via the 12-bit MCP4728 DAC, preventing any ECM diagnostic trouble codes (DTC P2138 / limp-home mode).
* **Glitchless Relay Transition (Zero-Drop Engagement):**
  * When `SET` is pressed, the ESP32 measures the driver's current pedal position, pre-charges the DAC to the exact matching voltage, and only then energizes the DPDT relay contacts. The transition from physical pedal to electronic control is 100% seamless and imperceptible to the vehicle's engine control module.
* **Progressive Driver Override with Zero Dead Zone:**
  * Automatic boot-time baseline calibration of the physical pedal resting voltage.
  * Instantaneous, proportional acceleration boost as soon as the driver rests their foot on the gas pedal to overtake, smoothly returning to closed-loop PID speed control once released.
* **Hardware Failsafe Architecture:**
  * DPDT signal relay is normally de-energized: if the MCU loses power, crashes, or triggers a safety fault, the physical pedal lines immediately revert to their factory connection to the ECM.
  * Instant cutoff on: brake pedal switch (CAN and hardware redundant), CANCEL button, CRUISE MAIN switch disarm, vehicle speed out of bounds (min 25 km/h, max 140 km/h), or CAN communication watchdog timeout (1000ms).

---

## 🛠️ Hardware Architecture & System Diagram

```text
+--------------------------------------------------------------------------+
|                                 VEHICLE                                  |
|                                                                          |
|   +-----------------------+                    +--------------------+    |
|   | Steering Wheel Switch |                    |   Honda F-CAN      |    |
|   | (SET-, RES+, CANCEL)  |                    | (CAN H / CAN L)    |    |
|   +-----------+-----------+                    +---------+----------+    |
|               | (3 Wires -> Cluster)                     |               |
|               v                                          v               |
|       [Gauge Control Module]                   [CAN Transceiver]         |
|         (Broadcast F-CAN)                     (SN65HVD230 / VP230)       |
|               |                                          |               |
|               +------------------+  +--------------------+               |
|                                  |  |                                    |
|                                  v  v                                    |
|                             +-------------+                              |
|                             |    ESP32    |                              |
|                             |  Controller |                              |
|                             +------+------+                              |
|                                    |                                     |
|               +--------------------+---------------------+               |
|               | (I2C)                                    | (DPDT Relay   |
|               v                                          v   GPIO 25)    |
|          +---------+                               +-------------+       |
|          | MCP4728 |                               |  DPDT Signal|       |
|          |  12-Bit |                               |    Relay    |       |
|          |   DAC   |                               +------+------+       |
|          +----+----+                                      |              |
|               | (APP1 & APP2                              v              |
|               |  Spoofed Voltages)                 +-------------+       |
|               +───────────────────┐                | NO Contacts |       |
|                                   │                |             |       |
|                                   └───────────────>|  (Failsafe) |       |
|  +--------------------+                            |             |       |
|  | Physical Gas Pedal |──────── NC Contacts ──────>|             |──────> ECM / PCM
|  |   (APP1 & APP2)    |   (De-energized = Factory) |             |        (Pedal Input)
|  +--------------------+                            +-------------+
+--------------------------------------------------------------------------+
```

### ESP32 Pin Assignment Table

| Peripheral | Signal / Function | ESP32 GPIO | Description |
| :--- | :--- | :---: | :--- |
| **CAN Transceiver** | `CTX` (TXD) | **GPIO 5** | Native ESP32 TWAI controller transmit line |
| **CAN Transceiver** | `CRX` (RXD) | **GPIO 4** | Native ESP32 TWAI controller receive line |
| **CAN Bus H / L** | F-CAN High / Low | — | Vehicle F-CAN at ECM Pin A3 (White) and A4 (Red) |
| **MCP4728 DAC** | I2C `SDA` | **GPIO 21** | 100 kHz I2C Data bus |
| **MCP4728 DAC** | I2C `SCL` | **GPIO 22** | 100 kHz I2C Clock bus |
| **MCP4728 DAC** | `VOUTA` (Channel A) | — | Emulated **APP2** signal $\to$ Relay NO2 |
| **MCP4728 DAC** | `VOUTB` (Channel B) | — | Emulated **APP1** signal $\to$ Relay NO1 |
| **DPDT Relay** | Coil Control | **GPIO 25** | Drives NPN / N-channel MOSFET switch (`HIGH` = Active) |
| **Pedal Sensor** | Analog Sense | **GPIO 36 (VP)**| Reads physical gas pedal before relay for driver override |
| **Status LED** | Cruise Active | **GPIO 26** | Illuminates when cruise is actively controlling throttle |
| **Status LED** | Cruise Main Arm | **GPIO 27** | Illuminates when Cruise Main switch is enabled |

---

## 📡 Reverse-Engineered Honda F-CAN Protocol Reference

### 1. Steering Wheel Cruise Buttons (`ID 0x164` @ 50 Hz)
The Gauge Control Module (GCM) samples the steering wheel resistive network and continuously broadcasts button states in **Byte 0**:

| Switch / Button State | Byte 0 (Hex) | Byte 0 (Binary) | System State / Action |
| :--- | :---: | :---: | :--- |
| **Cruise Main OFF (Idle)** | `0x04` | `0000 0100` | System disarmed (`OFF`) |
| **CRUISE MAIN ON** | `0x24` | `0010 0100` | System armed (`STANDBY`) |
| **SET / DECEL Pressed** | `0x64` | `0110 0100` | Engages cruise or decrements target (-1 km/h) |
| **RES / ACCEL Pressed** | `0xA4` | `1010 0100` | Resumes target or increments target (+1 km/h) |
| **CANCEL Pressed** | `0xE4` | `1110 0100` | Instantly disengages cruise to `STANDBY` |

### 2. Vehicle Speed and Engine RPM (`ID 0x158` @ 100 Hz)
* **Engine RPM:** `(Byte 0 << 8) | Byte 1`
* **Vehicle Speed (km/h):** `((Byte 4 << 8) | Byte 5) / 100.0`

### 3. Brake Switch and Cluster "CRUISE CONTROL" Lamp (`ID 0x17C` @ 100 Hz)
* **Brake Switch Bit:** `(Byte 4 & 0x01)` (`1` = depressed, `0` = released).
* **Cluster Indicator Lamp Bit:** `Byte 4, Bit 3 (0x08)`. When engaged, the ESP32 synthesizes and transmits updated 0x17C frames to illuminate the solid green `CRUISE CONTROL` indicator on the instrument panel.

---

## 🛡️ Disengage Diagnostics & Safety Table

Whenever the cruise control disengages, the system outputs an event log with the exact reason and numerical diagnostic code:

```text
>>> [CRUISE DESATIVOU] Motivo: [REASON] (código X) | Vel: Y km/h <<<
```

| Code | Enumeration | Trigger Condition |
| :---: | :--- | :--- |
| **1** | **`BRAKE_PRESSED`** | Digital brake flag detected on CAN frame `0x17C` (or hardware switch opens) |
| **3** | **`CANCEL_BUTTON`** | Steering wheel CANCEL button confirmed (Bits 6 + 7 active on `0x164`) |
| **4** | **`MAIN_SWITCH_OFF`** | Cruise Main switch toggled off (Bit 5 cleared on `0x164`) |
| **5** | **`SPEED_TOO_LOW`** | Vehicle speed dropped more than 5 km/h below configured minimum (< 20 km/h) |
| **6** | **`SPEED_TOO_HIGH`**| Vehicle speed exceeded safety ceiling (> 150 km/h) |
| **7** | **`CAN_TIMEOUT`** | No valid speed frames received on CAN bus for > 1000 ms |

---

## 💻 Real-Time Serial Telemetry Dashboard

The firmware provides a 5 Hz fixed-width in-place telemetry dashboard over USB serial (115200 baud):

```text
State: ACTIVE  | Spd: 70.0/70.0 km/h | Thr:18.5% | Ped: 0.0%(0.22V) | APP: 0.43/0.85V | DAC:OK | CAN:OK  | Brk:OFF | Lamp:ON
```

* **State:** State machine mode (`OFF`, `STANDBY`, `ACTIVE`, `OVERRIDE`).
* **Spd:** Current vehicle speed vs Target setpoint (km/h).
* **Thr:** Calculated PID throttle command (%).
* **Ped:** Driver physical gas pedal position (%) and real-time analog sense voltage (V).
* **APP:** Real voltages driven on the MCP4728 hardware outputs (Channel A: APP2 / Channel B: APP1).
* **DAC / CAN / Brk / Lamp:** Health and heartbeat status of hardware peripherals.

---

## ⚙️ Building & Flashing (PlatformIO)

Requirements: [PlatformIO Core](https://platformio.org/) or the PlatformIO IDE extension in VS Code.

### Compile Firmware:
```powershell
pio run
```

### Upload to ESP32:
```powershell
pio run --target upload --upload-port COM8
```

### Interactive Serial Monitor (115200 baud):
```powershell
pio device monitor -p COM8 -b 115200
```

#### Interactive Debug Commands:
* `m`: Toggle virtual Cruise Main switch
* `s`: Press SET / DECEL button
* `r`: Press RES / ACCEL button
* `c`: Press CANCEL button
* `b`: Simulate 500ms brake pedal press
* `+` / `-`: Step virtual vehicle speed up/down by 5 km/h (bench simulation mode)
* `i`: Run I2C bus scanner and MCP4728 device ping diagnostic
* `x`: Cycle CAN sniffer modes (Disabled $\to$ Filtered Cruise IDs $\to$ All Raw Frames)

---

## 📁 Repository Layout

```text
├── docs/                              # Technical documentation and protocol specs
│   ├── HONDA_CAN_CRUISE_REFERENCE.md  # Detailed F-CAN frame mapping and logs
│   ├── WIRING_AND_ARCHITECTURE.md     # Wiring schematic and electrical pinouts
│   └── CAN_SNIFFER_GUIDE.md           # Raspberry Pi Pico CAN sniffer guide
├── include/                           # C++ Header files
│   ├── config.h                       # Pin assignments, thresholds, and calibration
│   ├── cruise_controller.h            # State machine and closed-loop PID algorithm
│   ├── can_speed.h                    # ESP32 TWAI driver for F-CAN read/write
│   ├── mcp4728_dac.h                  # 12-bit DAC dual-slope voltage generator
│   └── safety_supervisor.h            # Interlock supervision and pedal auto-calibration
├── src/                               # Application source files
│   ├── main.cpp                       # Dual-core FreeRTOS setup and debug console
│   ├── cruise_controller.cpp          # Cruise control logic and smooth engagement
│   ├── can_speed.cpp                  # Honda F-CAN message parsing and lamp injector
│   ├── mcp4728_dac.cpp                # I2C fast-mode multi-channel DAC driver
│   └── safety_supervisor.cpp          # Real-time safety supervisor and pedal calibration
├── tools/                             # Python diagnostic scripts and test suites
└── platformio.ini                     # PlatformIO environment and board configuration
```

---

## ⚠️ Safety Disclaimer

> [!CAUTION]
> Interfacing directly with a motor vehicle's Drive-By-Wire (DBW) electronic throttle system involves inherent safety risks. This project is shared solely for research, educational, and reverse-engineering purposes. All implementation and road testing is performed strictly at the user's own risk. Ensure proper electrical isolation, automotive-grade wire harnesses, and always verify that the redundant brake pedal physical interlock disengages the system immediately and unconditionally.
