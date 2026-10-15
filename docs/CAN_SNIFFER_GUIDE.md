# Honda Fit (2nd Generation) CAN Bus Sniffer & Reverse Engineering Guide

This guide details how to sniff, monitor, and reverse engineer CAN traffic on the **Honda Fit 2nd Generation (GE6/GE7/GE8/GE9, 2008–2014)** using the ESP32 and native TWAI peripheral.

---

## 1. Safety & Architecture: Listen-Only Mode

When tapping into a live vehicle bus, an improperly configured microcontroller can transmit recessive/dominant bit mismatches, flood the bus with error frames, cause a **Bus-Off** state, or trigger instrument cluster warning lights (VSA, EPS, Check Engine).

To prevent this, our sniffer runs by default in **`TWAI_MODE_LISTEN_ONLY`**:
* **100% Passive (High-Impedance):** The ESP32 does not transmit frame bits, acknowledge (ACK) bits, or error frames.
* **Zero Bus Disturbance:** The vehicle's ECUs (ECM, ABS/VSA, EPS, MICU) cannot detect the presence of the sniffer.
* **Safe for Real-Time Probing:** You can connect and disconnect while the vehicle is idling or being driven without tripping DTCs.

---

## 2. Hardware Wiring & OBD-II Pinout

### A. Standard Honda 16-Pin OBD-II Diagnostic Port

The OBD-II port is located under the driver's side dashboard (above the pedals):

```
       1   2   3   4   5   6   7   8
     ┌───┬───┬───┬───┬───┬───┬───┬───┐
     │   │   │   │GND│GND│CAN│   │   │
     └───┴───┴───┴───┴───┴───┴───┴───┘
     │   │   │   │   │   │CAN│   │12V│
     └───┴───┴───┴───┴───┴───┴───┴───┘
       9  10  11  12  13  14  15  16
```

| OBD-II Pin | Signal | Vehicle Wire Color (Fit 2G) | Connection to Pi Pico / MCP2515 |
| :--- | :--- | :--- | :--- |
| **Pin 4** | Chassis Ground | Black | Connect to Pico **GND (Pin 38)** & MCP2515 **GND** |
| **Pin 5** | Signal Ground | Black | (Alternative / redundant chassis ground) |
| **Pin 6** | **F-CAN High** (500 kbps) | White | Connect to MCP2515 **CANH** |
| **Pin 14** | **F-CAN Low** (500 kbps) | Red | Connect to MCP2515 **CANL** |
| **Pin 16** | Battery +12V Constant | White | (Unused for sniffer; Pico is powered via USB) |

> [!IMPORTANT]
> **Ground Reference:** You **must** connect Pi Pico `GND` and MCP2515 `GND` to OBD-II Pin 4 or Pin 5. Without a common vehicle ground reference, common-mode voltage differences will corrupt CAN packets.

---

### B. Raspberry Pi Pico (RP2040) + MCP2515 Wiring (Top-Left Pins 1 to 5)

We use the **consecutive top-left corner pins (Pins 1 to 5)** of the Raspberry Pi Pico. This is the official native **SPI0** bus and avoids all pin counting confusion:

```
               ┌─────────────────────── USB ───────────────────────┐
               │  Pin 1: GP0 (SPI0 RX / MISO)  ───► MCP2515 SO     │ Pin 40: VBUS (5V) ──► MCP2515 VCC
               │  Pin 2: GP1 (SPI0 CSn / CS)   ───► MCP2515 CS     │
               │  Pin 3: GND                   ───► MCP2515 GND    │
               │  Pin 4: GP2 (SPI0 SCK)        ───► MCP2515 SCK    │
               │  Pin 5: GP3 (SPI0 TX / MOSI)  ───► MCP2515 SI     │
               │  Pin 6: GP4 (INT, optional)   ───► MCP2515 INT    │
               └───────────────────────────────────────────────────┘
```

#### Detailed Pin Connection Table:
| MCP2515 Pin | Pi Pico Physical Pin | GPIO / Function | Location on Pico |
| :--- | :--- | :--- | :--- |
| **SO (MISO)**| **Physical Pin 1** | **GP0** (SPI0 RX) | **Very Top-Left corner hole** (next to USB) |
| **CS** | **Physical Pin 2** | **GP1** (SPI0 CSn) | 2nd hole down on the left |
| **GND** | **Physical Pin 3** | **GND** | 3rd hole down on the left (also connect to OBD-II Pin 4/5) |
| **SCK** | **Physical Pin 4** | **GP2** (SPI0 SCK) | 4th hole down on the left |
| **SI (MOSI)**| **Physical Pin 5** | **GP3** (SPI0 TX) | 5th hole down on the left |
| **VCC** | **Physical Pin 40**| **VBUS (5V USB)** | **Very Top-Right corner hole** (opposite Pin 1) |
| **CANH** | — | — | OBD-II **Pin 6** (White wire, F-CAN High) |
| **CANL** | — | — | OBD-II **Pin 14** (Red wire, F-CAN Low) |

> [!WARNING]
> **Remove Jumper J1 on MCP2515 Board (120 $\Omega$ Termination):**
> Look at the 2-pin header labeled **`J1`** or **`120R`** next to the blue screw terminals.
> Your Honda Fit factory F-CAN bus already has two internal $120\,\Omega$ termination resistors ($60\,\Omega$ net).
> **PULL OFF the jumper cap from J1** so you do not load the car bus down to $40\,\Omega$.

> [!TIP]
> **Crystal Frequency Check (8 MHz vs 16 MHz):**
> Look at the small silver metal oval crystal oscillator on the MCP2515 board:
> - If stamped **`8.000`** $\to$ Board has an **8 MHz** crystal (Default in [`tools/pico_mcp2515_sniffer.py`](file:///C:/Users/vrmi/.gemini/antigravity-ide/scratch/esp32-cruise-control/tools/pico_mcp2515_sniffer.py)).
> - If stamped **`16.000`** $\to$ Set `CRYSTAL_MHZ = 16` at the top of the script.

---

### C. ESP32 Alternative Wiring (If Using ESP32 Instead)
* **With MCP2515:** CS=GPIO5, SCK=GPIO18, MISO=GPIO19, MOSI=GPIO23, VCC=5V (VIN), GND=GND.
* **With 3.3V Transceiver (SN65HVD230 - Native TWAI):** CRX=GPIO4, CTX=GPIO5, VCC=3V3, GND=GND.


---

## 3. Running the Sniffer

### Method 1: Raspberry Pi Pico (MicroPython — Zero Compiler Setup)
This is the fastest method to get up and running:

1. Connect your Raspberry Pi Pico running standard MicroPython to your PC via USB.
2. Open [Thonny](https://thonny.org/) (or VS Code).
3. Open [`tools/pico_mcp2515_sniffer.py`](file:///C:/Users/vrmi/.gemini/antigravity-ide/scratch/esp32-cruise-control/tools/pico_mcp2515_sniffer.py).
4. Click **Run** (or save as `main.py` onto the Pico so it boots automatically).
5. The Pico begins capturing vehicle CAN frames in **100% safe Listen-Only mode** at 500 kbps and streams them over USB serial.
6. Launch the live visual dashboard on your PC:
   ```powershell
   python tools/can_sniffer.py
   ```

---

### Method 2: Dedicated ESP32 Firmware (`can_sniffer`)
Dedicated C++ firmware with 128-frame deep queue, multi-format output (Table, SLCAN, CSV), runtime baud rate switching, and hardware ID filtering.

To compile and flash to ESP32:
```powershell
# Build and flash using PlatformIO
pio run -e can_sniffer -t upload

# Open Serial Monitor at 115200 baud
pio device monitor -b 115200
```

#### Interactive Serial Commands:
| Key | Function |
| :---: | :--- |
| `m` | Cycle output format: **Table** $\to$ **SLCAN (SavvyCAN)** $\to$ **CSV** |
| `f <hex>` | Filter by specific CAN ID (e.g. `f 158` or `f 0` to clear) |
| `i <hex>` | Mute / ignore noisy ID (e.g. `i 158` to hide 100Hz engine frames) |
| `b <baud>` | Switch baud rate (`b 500`, `b 250`, `b 125`) |
| `l` | Toggle Listen-Only mode vs Normal mode |
| `s` | Display bus error counters, frame rates, and buffer utilization |
| `p` | Pause / resume streaming |
| `?` or `h` | Display help menu |

---

### Option 2: Live Sniffer in Cruise Control Firmware (`esp32dev`)
If the ESP32 is already running the cruise control firmware (`main.cpp`), you can inspect raw packets on the fly without reflashing:
1. Connect via Serial Monitor (`115200` baud).
2. Press **`x`** to toggle the live CAN packet sniffer dump on and off!

---

## 4. Desktop Real-Time Analyzer Tool (`tools/can_sniffer.py`)

A terminal dashboard with **Byte Diff Highlighting** and automatic Honda Fit 2nd Gen parameter decoding.

### Features:
1. **Live Matrix View:** Displays all active CAN IDs, frequency (Hz), frame counts, and hex payload.
2. **Byte Diff Highlighting:** Whenever a byte changes value, it is highlighted in **bright yellow** for 1.2 seconds. This allows you to immediately pinpoint which byte and bit flip when you press a pedal or switch!
3. **Automatic Signal Decoding:** Decodes Vehicle Speed, Engine RPM, Gas Pedal %, Brake Switch, Wheel Speeds, and Steering Angle in real time.
4. **SavvyCAN / Candump Logging:** Automatically writes captures to `.csv` and `.log` (candump format) for playback and deep analysis.

### Running with Real Hardware:
```powershell
python tools/can_sniffer.py
# Or specify a port explicitly:
python tools/can_sniffer.py --port COM3
```

### Running in Bench Simulation Mode:
Test and verify the sniffer right on your computer without hardware connected:
```powershell
python tools/can_sniffer.py --sim
```

### Filtering & Muting Options:
```powershell
# Only watch Engine (0x158) and Brake (0x1A6):
python tools/can_sniffer.py --filter 158,1A6

# Mute noisy 100Hz engine packets:
python tools/can_sniffer.py --mute 158
```

---

## 5. Honda Fit (2nd Generation) CAN ID Reference Table

| CAN ID | Typical Hz | Module / Subsystem | Signals & Byte Definitions |
| :---: | :---: | :--- | :--- |
| **`0x158`** | 100 Hz | **ECM (Engine / PCM)** | **B0-B1:** Engine RPM = `(B0 << 8) \| B1`<br>**B4-B5:** Vehicle Speed = `((B4 << 8) \| B5) / 100.0` km/h |
| **`0x17C`** | 100 Hz | **ECM (Throttle & APP)** | **B0:** Throttle Position Sensor (TPS)<br>**B4:** Accelerator Pedal Position (APP) |
| **`0x1A6`** | 50 Hz | **PCM / AT Controller** | **B2, bit 5:** Brake pedal switch (`0x20` = Pressed)<br>**B4:** Automatic/CVT Shift Position (`0x00`=P, `0x10`=R, `0x20`=N, `0x30`=D, `0x40`=S) |
| **`0x1DC`** | 100 Hz | **EPS (Steering Angle)** | **B0-B1:** Signed Steering Wheel Angle ($0.1^\circ$/LSB)<br>**B2-B3:** Steering Angular Velocity |
| **`0x245`** | 50 Hz | **EPS (Torque Assist)** | **B0-B1:** Driver steering input torque<br>**B2-B3:** EPS electric motor assist torque |
| **`0x305`** | 50 Hz | **VSA / ABS Controller** | **B0-B1:** VSA reference vehicle speed<br>**B2:** ABS active / fault flags |
| **`0x309`** | 50 Hz | **VSA (Wheel Speeds)** | **B0-B1:** Front-Left Wheel Speed (km/h / 100)<br>**B2-B3:** Front-Right Wheel Speed<br>**B4-B5:** Rear-Left Wheel Speed<br>**B6-B7:** Rear-Right Wheel Speed |
| **`0x324`** | 50 Hz | **VSA (Yaw & Lateral G)** | **B0-B1:** Yaw rate sensor ($^\circ$/s)<br>**B2-B3:** Lateral acceleration G-sensor |
| **`0x405`** | 10 Hz | **Gauge Cluster (Meter)** | **B0-B3:** Odometer mileage<br>**B4:** Fuel level gauge<br>**B5:** Seatbelt / Door ajar / Handbrake indicator |
| **`0x7E8`** | On Demand | **OBD-II Diagnostic ECU** | Standard OBD-II response to diagnostic request `0x7E0` |

---

## 6. Reverse Engineering Walkthrough

When investigating unmapped signals (such as steering wheel buttons or custom switches):

1. **Park with Ignition ON (Engine OFF or Idling):**
   Run the sniffer:
   ```powershell
   python tools/can_sniffer.py
   ```
2. **Observe the Base Bus State:**
   Wait 5 seconds for the live matrix to stabilize. Most values will stop lighting up yellow.
3. **Perform a Single Isolated Action:**
   * **Brake Switch:** Press and release the brake pedal 3 times. Watch for the byte in `0x1A6` that alternates between `0x00` and `0x20`.
   * **Transmission Shifter:** Move shifter from **P** $\to$ **R** $\to$ **N** $\to$ **D**. Observe Byte 4 of `0x1A6` changing (`0x00`, `0x10`, `0x20`, `0x30`).
   * **Steering Wheel Buttons:** Press SET, RES, or CANCEL. If the steering switches are broadcast over CAN on your specific trim level, a single byte will flip bitwise.
4. **Import into SavvyCAN for Graphing:**
   The sniffer automatically generates `honda_fit_can_<timestamp>.log`. You can open this file directly in [SavvyCAN](https://www.savvycan.com/) or Wireshark to plot signals against time or create custom DBC files.
