#!/usr/bin/env python3
"""
Interactive CAN Bus Sniffer & Reverse Engineering Tool for Honda Fit (2nd Gen)
Connects to ESP32 running CAN sniffer firmware over USB Serial (or runs in --sim mode).

Key Features:
  - Live ID Matrix Dashboard: Shows all active CAN IDs, frequency (Hz), frame counts, delta-t
  - Byte Diff Highlighting: Brightly highlights bytes that change in real time (crucial for reverse engineering)
  - Built-in Honda Fit 2nd Gen Signature Decoder:
      * 0x158: Engine RPM & Vehicle Speed (ECM)
      * 0x17C: Throttle & Accelerator Pedal Position (APP)
      * 0x1A6: Brake Switch & Transmission / Gear Position (AT/CVT)
      * 0x1DC: Steering Wheel Angle & Rate
      * 0x245: Electric Power Steering (EPS) Status & Assist Torque
      * 0x309: Individual 4-Wheel Speeds (VSA/ABS)
      * 0x324: VSA Lateral G & Yaw Rate
  - CSV & Candump (.log) recording for SavvyCAN, Wireshark, Excel
  - Simulation Mode (--sim) for bench testing without hardware
"""

import sys
import os
import time
import re
import argparse
import threading
from collections import defaultdict
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ANSI Color codes for cross-platform rich terminal display
CLR_RESET   = "\033[0m"
CLR_BOLD    = "\033[1m"
CLR_DIM     = "\033[2m"
CLR_RED     = "\033[91m"
CLR_GREEN   = "\033[92m"
CLR_YELLOW  = "\033[93m"
CLR_BLUE    = "\033[94m"
CLR_MAGENTA = "\033[95m"
CLR_CYAN    = "\033[96m"
CLR_WHITE   = "\033[97m"
BG_HIGHLIGHT= "\033[43;30m" # Yellow background, black text for changed bytes
BG_CYAN     = "\033[46;30m"

# Honda Fit (2nd Gen / GE6/GE8) Known CAN ID Database
HONDA_FIT_CAN_DATABASE = {
    0x158: {
        "name": "ECM: RPM & Vehicle Speed",
        "system": "Powertrain",
        "expected_hz": 100,
        "desc": "B0-B1: RPM, B4-B5: Speed (raw/100 km/h)"
    },
    0x17C: {
        "name": "ECM: Throttle & Pedal (APP)",
        "system": "Powertrain",
        "expected_hz": 100,
        "desc": "B0: Throttle Position, B4: Gas Pedal (APP)"
    },
    0x1A6: {
        "name": "PCM/AT: Brake & Gear Position",
        "system": "Transmission/Brake",
        "expected_hz": 50,
        "desc": "Brake switch bits & AT Gear (P/R/N/D/S/1/2)"
    },
    0x1DC: {
        "name": "EPS: Steering Wheel Angle",
        "system": "Steering",
        "expected_hz": 100,
        "desc": "B0-B1: Steering angle signed, B2-B3: Steering angular rate"
    },
    0x245: {
        "name": "EPS: Assist Torque & Status",
        "system": "Steering",
        "expected_hz": 50,
        "desc": "Driver torque input & EPS motor assist command"
    },
    0x305: {
        "name": "VSA: Vehicle Speed & ABS",
        "system": "Braking/Stability",
        "expected_hz": 50,
        "desc": "VSA reference speed & ABS warning state"
    },
    0x309: {
        "name": "VSA: Individual Wheel Speeds",
        "system": "Braking/Stability",
        "expected_hz": 50,
        "desc": "B0-B1: FL, B2-B3: FR, B4-B5: RL, B6-B7: RR wheel speed"
    },
    0x324: {
        "name": "VSA: Yaw Rate & Lateral Accel",
        "system": "Stability",
        "expected_hz": 50,
        "desc": "Yaw rate sensor & G-sensor lateral acceleration"
    },
    0x405: {
        "name": "Meter: Cluster Status & Fuel",
        "system": "Gauge Cluster",
        "expected_hz": 10,
        "desc": "Odometer, fuel level, warning lights"
    },
    0x7E8: {
        "name": "OBD-II Diagnostic Response",
        "system": "Diagnostics",
        "expected_hz": 0,
        "desc": "Standard OBD-II ECU response to 0x7E0 request"
    }
}

class CanFrame:
    def __init__(self, can_id: int, data: bytes, timestamp: float, is_extended: bool = False):
        self.can_id = can_id
        self.data = data
        self.dlc = len(data)
        self.timestamp = timestamp
        self.is_extended = is_extended

class CanSnifferTracker:
    def __init__(self, log_csv_path: str = None, log_candump_path: str = None):
        self.records = {} # can_id -> dict of metadata
        self.filter_ids = set()
        self.muted_ids = set()
        self.paused = False
        self.lock = threading.Lock()
        self.log_csv_file = open(log_csv_path, "w", buffering=1) if log_csv_path else None
        self.log_candump_file = open(log_candump_path, "w", buffering=1) if log_candump_path else None
        self.total_frames = 0
        self.start_time = time.time()

        if self.log_csv_file:
            self.log_csv_file.write("timestamp,can_id_hex,dlc,data_hex,name\n")

    def process_frame(self, frame: CanFrame):
        with self.lock:
            self.total_frames += 1
            cid = frame.can_id

            # Filter & Mute logic
            if self.filter_ids and (cid not in self.filter_ids):
                return
            if cid in self.muted_ids:
                return

            now = frame.timestamp
            rec = self.records.get(cid)

            if rec is None:
                rec = {
                    "can_id": cid,
                    "is_extended": frame.is_extended,
                    "data": frame.data,
                    "prev_data": frame.data,
                    "count": 1,
                    "first_seen": now,
                    "last_seen": now,
                    "delta_ms": 0.0,
                    "rate_hz": 0.0,
                    "rate_window": [],
                    "byte_changed_time": [0.0] * 8, # Timestamp of last change for each byte
                }
                self.records[cid] = rec
            else:
                dt = (now - rec["last_seen"]) * 1000.0
                rec["delta_ms"] = dt
                rec["count"] += 1
                rec["last_seen"] = now

                # Rate calculation sliding window
                rec["rate_window"].append(now)
                rec["rate_window"] = [t for t in rec["rate_window"] if now - t <= 1.0]
                rec["rate_hz"] = len(rec["rate_window"])

                # Check changed bytes
                new_data = frame.data
                old_data = rec["data"]
                for i in range(min(len(new_data), len(old_data), 8)):
                    if new_data[i] != old_data[i]:
                        rec["byte_changed_time"][i] = now

                rec["prev_data"] = old_data
                rec["data"] = new_data

            # Log to CSV
            if self.log_csv_file:
                info = HONDA_FIT_CAN_DATABASE.get(cid, {})
                name = info.get("name", "Unknown")
                hex_data = " ".join(f"{b:02X}" for b in frame.data)
                self.log_csv_file.write(f"{now:.6f},0x{cid:03X},{frame.dlc},{hex_data},{name}\n")

            # Log to Candump (.log for SavvyCAN / Wireshark)
            if self.log_candump_file:
                hex_packed = "".join(f"{b:02X}" for b in frame.data)
                self.log_candump_file.write(f"({now:.6f}) can0 {cid:03X}#{hex_packed}\n")

    def decode_signals(self) -> dict:
        """Decode live Honda Fit 2nd Gen physical parameters."""
        res = {}
        with self.lock:
            # 0x158: RPM & Speed
            if 0x158 in self.records:
                d = self.records[0x158]["data"]
                if len(d) >= 6:
                    rpm = (d[0] << 8) | d[1]
                    speed = ((d[4] << 8) | d[5]) / 100.0
                    res["rpm"] = f"{rpm} RPM"
                    res["speed"] = f"{speed:.1f} km/h"

            # 0x17C: Throttle & APP
            if 0x17C in self.records:
                d = self.records[0x17C]["data"]
                if len(d) >= 5:
                    tps = (d[0] / 255.0) * 100.0
                    app = (d[4] / 255.0) * 100.0
                    res["tps"] = f"{tps:.1f} %"
                    res["pedal"] = f"{app:.1f} %"

            # 0x1A6: Brake & Gear
            if 0x1A6 in self.records:
                d = self.records[0x1A6]["data"]
                if len(d) >= 4:
                    brake_pressed = bool(d[2] & 0x20)
                    res["brake"] = f"{CLR_RED}PRESSED{CLR_RESET}" if brake_pressed else f"{CLR_GREEN}OFF{CLR_RESET}"
                    gear_byte = d[4] if len(d) > 4 else 0
                    gear_map = {0x00: "P", 0x10: "R", 0x20: "N", 0x30: "D", 0x40: "S", 0x50: "1", 0x60: "2"}
                    res["gear"] = gear_map.get(gear_byte & 0xF0, f"0x{gear_byte:02X}")

            # 0x1DC: Steering Angle
            if 0x1DC in self.records:
                d = self.records[0x1DC]["data"]
                if len(d) >= 2:
                    raw_steer = (d[0] << 8) | d[1]
                    if raw_steer & 0x8000:
                        raw_steer -= 0x10000
                    angle = raw_steer * 0.1
                    res["steering"] = f"{angle:+.1f} deg"

            # 0x309: Wheel speeds
            if 0x309 in self.records:
                d = self.records[0x309]["data"]
                if len(d) >= 8:
                    fl = ((d[0] << 8) | d[1]) / 100.0
                    fr = ((d[2] << 8) | d[3]) / 100.0
                    rl = ((d[4] << 8) | d[5]) / 100.0
                    rr = ((d[6] << 8) | d[7]) / 100.0
                    res["wheels"] = f"FL:{fl:.1f} FR:{fr:.1f} RL:{rl:.1f} RR:{rr:.1f} km/h"

        return res

    def close(self):
        if self.log_csv_file:
            self.log_csv_file.close()
        if self.log_candump_file:
            self.log_candump_file.close()

def render_dashboard(tracker: CanSnifferTracker, port_info: str, sim_mode: bool):
    """Draw the real-time ANSI terminal dashboard."""
    now = time.time()
    uptime = now - tracker.start_time

    with tracker.lock:
        items = list(tracker.records.values())

    # Sort by CAN ID ascending
    items.sort(key=lambda x: x["can_id"])

    lines = []
    # Clear screen and return cursor to home (0,0)
    lines.append("\033[H\033[2J")

    # Header
    banner = f"{CLR_BOLD}{CLR_CYAN}=== HONDA FIT (2nd GEN) REAL-TIME CAN BUS ANALYZER ==={CLR_RESET}"
    status = f"Source: {CLR_YELLOW}{'SIMULATOR' if sim_mode else port_info}{CLR_RESET} | Frames: {tracker.total_frames:,} | Uptime: {uptime:.1f}s | Active IDs: {len(items)}"
    lines.append(banner)
    lines.append(status)
    lines.append("-" * 115)

    # Live Decoded Signals Bar
    dec = tracker.decode_signals()
    sig_str = (
        f"{CLR_BOLD}LIVE SIGNALS:{CLR_RESET} "
        f"Speed: {CLR_GREEN}{dec.get('speed', '-- km/h')}{CLR_RESET} | "
        f"RPM: {CLR_YELLOW}{dec.get('rpm', '-- RPM')}{CLR_RESET} | "
        f"Throttle: {CLR_CYAN}{dec.get('pedal', '-- %')}{CLR_RESET} | "
        f"Brake: {dec.get('brake', '--')} | "
        f"Steering: {CLR_WHITE}{dec.get('steering', '-- deg')}{CLR_RESET}"
    )
    if "wheels" in dec:
        sig_str += f"\n              Wheels: {CLR_BLUE}{dec['wheels']}{CLR_RESET}"
    lines.append(sig_str)
    lines.append("=" * 115)

    # Table Header
    header = (
        f"{CLR_BOLD}{'CAN ID':<8} "
        f"{'NAME / SYSTEM':<32} "
        f"{'COUNT':>7} "
        f"{'RATE':>6} "
        f"{'DT(ms)':>6}  "
        f"{'PAYLOAD (HEX)':<28} "
        f"{'ASCII':<8}{CLR_RESET}"
    )
    lines.append(header)
    lines.append("-" * 115)

    if not items:
        lines.append(f"\n{CLR_YELLOW}>>> AGUARDANDO DADOS DO VEÍCULO... (Se nada aparecer em 3s, mude a chavinha do cabo de lado!){CLR_RESET}\n")

    # Table Body
    for rec in items:
        cid = rec["can_id"]
        info = HONDA_FIT_CAN_DATABASE.get(cid, {})
        name = info.get("name", "Unknown Module")[:31]

        # Format Payload with Changed-Byte Highlight
        payload_strs = []
        ascii_chars = []
        for i in range(8):
            if i < len(rec["data"]):
                val = rec["data"][i]
                # If byte changed within last 1.2 seconds, highlight it!
                if (now - rec["byte_changed_time"][i]) < 1.2:
                    payload_strs.append(f"{BG_HIGHLIGHT}{val:02X}{CLR_RESET}")
                else:
                    payload_strs.append(f"{val:02X}")
                ascii_chars.append(chr(val) if 32 <= val <= 126 else ".")
            else:
                payload_strs.append("  ")

        data_hex_col = " ".join(payload_strs)
        ascii_col = "".join(ascii_chars)

        ext_tag = "X" if rec.get("is_extended") else " "
        row = (
            f"0x{cid:03X}{ext_tag:<2} "
            f"{name:<32} "
            f"{rec['count']:>7} "
            f"{rec['rate_hz']:>5.0f}Hz "
            f"{rec['delta_ms']:>6.1f}  "
            f"{data_hex_col:<28} "
            f"{ascii_col}"
        )
        lines.append(row)

    lines.append("-" * 115)
    lines.append(
        f"{CLR_DIM}[HOTKEYS] Filter: press 'f' | Mute: 'm' | Clear Diffs: 'c' | Pause: 'p' | Quit: 'q' / Ctrl+C{CLR_RESET}"
    )

    # Print entire frame atomically to eliminate flickering
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()

def parse_incoming_line(line: str) -> CanFrame | None:
    """
    Parses both SLCAN ('t15881122334455667788') and formatted ESP32 table output:
    Example formatted: '[  12345 ms | + 10 ms]  ID: 0x158  DLC: 8  [ 1F 24 00 00 1E 88 00 00 ]'
    Example CSV: '12345,0x158,0,8,1F,24,00,00,1E,88,00,00'
    """
    line = line.strip()
    if not line:
        return None

    now = time.time()

    # 1. SLCAN standard frame: t<ID:3><DLC:1><DATA>
    if line.startswith('t') and len(line) >= 5:
        try:
            cid = int(line[1:4], 16)
            dlc = int(line[4])
            hex_data = line[5:5 + (dlc * 2)]
            data = bytes.fromhex(hex_data)
            return CanFrame(cid, data, now, is_extended=False)
        except Exception:
            pass

    # 2. SLCAN extended frame: T<ID:8><DLC:1><DATA>
    if line.startswith('T') and len(line) >= 10:
        try:
            cid = int(line[1:9], 16)
            dlc = int(line[9])
            hex_data = line[10:10 + (dlc * 2)]
            data = bytes.fromhex(hex_data)
            return CanFrame(cid, data, now, is_extended=True)
        except Exception:
            pass

    # 3. CSV stream format: TIME_MS,ID_HEX,EXT,DLC,B0,B1...
    if ',' in line:
        parts = line.split(',')
        if len(parts) >= 4 and parts[1].startswith('0x'):
            try:
                cid = int(parts[1], 16)
                is_ext = bool(int(parts[2]))
                dlc = int(parts[3])
                data_bytes = [int(p, 16) for p in parts[4:4+dlc] if p]
                return CanFrame(cid, bytes(data_bytes), now, is_extended=is_ext)
            except Exception:
                pass

    # 4. Formatted Table output: '... ID: 0x158 ... [ 1F 24 00 ... ]'
    id_match = re.search(r'ID:\s*0x([0-9A-Fa-f]+)', line)
    data_match = re.search(r'\[\s*([0-9A-Fa-f\s]{2,})\s*\]', line)
    if id_match and data_match:
        try:
            cid = int(id_match.group(1), 16)
            hex_str = data_match.group(1).replace(" ", "")
            data = bytes.fromhex(hex_str)
            is_ext = "EXT" in line
            return CanFrame(cid, data, now, is_extended=is_ext)
        except Exception:
            pass

    # 5. ELM327 Raw Monitor Format: '158 8 1F 24 ...' or '158 1F 24 ...'
    # Ignore ELM status strings
    upper = line.upper()
    if not any(k in upper for k in ["BUFFER", "STOPPED", "ERROR", "NO DATA", "BUS BUSY", "SEARCHING", "OK", ">", "ELM"]):
        tokens = line.split()
        if len(tokens) >= 2:
            first = tokens[0]
            if len(first) in (3, 4, 8) and all(c in "0123456789ABCDEFabcdef" for c in first):
                try:
                    cid = int(first, 16)
                    is_ext = len(first) > 3
                    if len(tokens[1]) == 1 and tokens[1].isdigit():
                        hex_bytes = "".join(tokens[2:])
                    else:
                        hex_bytes = "".join(tokens[1:])
                    data = bytes.fromhex(hex_bytes)
                    if 0 < len(data) <= 8:
                        return CanFrame(cid, data, now, is_extended=is_ext)
                except Exception:
                    pass

    return None

def run_simulation(tracker: CanSnifferTracker, stop_event: threading.Event):
    """Simulates realistic Honda Fit 2nd Gen CAN bus traffic."""
    speed_kmh = 65.0
    rpm = 2100
    steering_angle = 0.0
    brake_active = False
    step = 0

    while not stop_event.is_set():
        now = time.time()
        step += 1

        # Simulate vehicle dynamics
        speed_kmh += (0.05 if step % 200 < 100 else -0.05)
        rpm = int(speed_kmh * 32.0)
        brake_active = (step % 120 > 105) # Tap brake periodically
        steering_angle = 15.0 * (1.0 if step % 80 < 40 else -1.0)

        # 0x158 (100 Hz): RPM & Speed
        raw_rpm = int(rpm)
        raw_spd = int(speed_kmh * 100.0)
        d_158 = bytes([
            (raw_rpm >> 8) & 0xFF, raw_rpm & 0xFF,
            0x00, 0x00,
            (raw_spd >> 8) & 0xFF, raw_spd & 0xFF,
            0x00, 0x00
        ])
        tracker.process_frame(CanFrame(0x158, d_158, now))

        # 0x17C (50 Hz): Throttle Position
        if step % 2 == 0:
            pedal_byte = min(255, int(speed_kmh * 1.8))
            d_17c = bytes([pedal_byte, 0x10, 0x00, 0x00, pedal_byte, 0x00, 0x00, 0x00])
            tracker.process_frame(CanFrame(0x17C, d_17c, now))

        # 0x1A6 (50 Hz): Brake & Gear Position
        if step % 2 == 0:
            brake_byte = 0x20 if brake_active else 0x00
            gear_byte = 0x30 # Drive (D)
            d_1a6 = bytes([0x00, 0x00, brake_byte, 0x00, gear_byte, 0x00, 0x00, 0x00])
            tracker.process_frame(CanFrame(0x1A6, d_1a6, now))

        # 0x1DC (50 Hz): Steering Angle
        if step % 2 == 0:
            raw_steer = int(steering_angle * 10.0) & 0xFFFF
            d_1dc = bytes([(raw_steer >> 8) & 0xFF, raw_steer & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            tracker.process_frame(CanFrame(0x1DC, d_1dc, now))

        # 0x309 (25 Hz): 4-Wheel Speeds
        if step % 4 == 0:
            ws = int(speed_kmh * 100.0)
            d_309 = bytes([
                (ws >> 8) & 0xFF, ws & 0xFF,
                (ws >> 8) & 0xFF, ws & 0xFF,
                (ws >> 8) & 0xFF, ws & 0xFF,
                (ws >> 8) & 0xFF, ws & 0xFF,
            ])
            tracker.process_frame(CanFrame(0x309, d_309, now))

        # 0x405 (10 Hz): Gauge Cluster
        if step % 10 == 0:
            d_405 = bytes([0x12, 0x34, 0x56, 0x78, 0x00, 0x45, 0x00, 0x00])
            tracker.process_frame(CanFrame(0x405, d_405, now))

        time.sleep(0.01) # 100Hz tick

def init_elm327(ser, filter_id=None):
    """Initializes ELM327 OBD2 adapter for passive Honda F-CAN (500 kbps) sniffing."""
    print(f"\n{CLR_CYAN}[ELM327] Initializing OBD2 Adapter on {ser.port}...{CLR_RESET}")
    # Wake up & interrupt any previous ATMA
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    commands = [
        (b"ATWS\r", 0.8, "Warm Start Reset"),
        (b"ATE0\r", 0.15, "Echo OFF"),
        (b"ATL1\r", 0.15, "Linefeed ON"),
        (b"ATH1\r", 0.15, "Headers ON (Show CAN IDs)"),
        (b"ATSP6\r", 0.25, "Protocol: ISO 15765-4 CAN (11-bit / 500k)"),
        (b"ATCAF0\r", 0.15, "CAN Auto-Formatting OFF")
    ]

    if filter_id is not None:
        cmd_filter = f"ATCRA {filter_id:03X}\r".encode()
        commands.append((cmd_filter, 0.15, f"Hardware CAN Filter: ID 0x{filter_id:03X}"))

    for cmd, delay, desc in commands:
        ser.write(cmd)
        time.sleep(delay)
        resp = ser.read_all().decode("ascii", errors="ignore").strip().replace("\r", " ").replace("\n", " ")
        print(f"  {CLR_DIM}-> {cmd.strip().decode()}: {resp} ({desc}){CLR_RESET}")
    print(f"{CLR_GREEN}[ELM327] Adapter configured for Honda F-CAN 500 kbps!{CLR_RESET}\n")

def find_esp32_port():
    """Auto-detect connected ESP32, Raspberry Pi Pico, or ELM327 USB serial COM port."""
    if not SERIAL_AVAILABLE:
        return None
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.lower()
        if any(k in desc for k in ["ch340", "ch341", "cp210", "ftdi", "uart", "espressif", "usb-serial", "pico", "rp2040", "raspberry"]):
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    parser = argparse.ArgumentParser(description="Honda Fit (2nd Gen) CAN Bus Sniffer & Reverse Engineering Tool")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g., COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=None, help="USB Serial Baud Rate (default: 38400 for ELM, 115200 for ESP32)")
    parser.add_argument("--elm", action="store_true", help="Connect to ELM327 OBD2 USB cable (auto-configures 500k CAN)")
    parser.add_argument("--sim", action="store_true", help="Run with simulated Honda Fit 2nd Gen CAN traffic")
    parser.add_argument("--filter", type=str, default=None, help="Comma-separated hex IDs to filter (e.g. 158,1A6)")
    parser.add_argument("--mute", type=str, default=None, help="Comma-separated hex IDs to mute/hide")
    parser.add_argument("--save-csv", type=str, default=None, help="Output CSV log file path")
    parser.add_argument("--save-log", type=str, default=None, help="Output candump format .log file path (SavvyCAN)")
    args = parser.parse_args()

    # Generate default log files if not specified
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = args.save_csv or f"honda_fit_can_{timestamp_str}.csv"
    log_file = args.save_log or f"honda_fit_can_{timestamp_str}.log"

    tracker = CanSnifferTracker(log_csv_path=csv_file, log_candump_path=log_file)

    if args.filter:
        for f in args.filter.split(","):
            f = f.strip().lower()
            if f.startswith("0x"): f = f[2:]
            if f: tracker.filter_ids.add(int(f, 16))

    if args.mute:
        for m in args.mute.split(","):
            m = m.strip().lower()
            if m.startswith("0x"): m = m[2:]
            if m: tracker.muted_ids.add(int(m, 16))

    stop_event = threading.Event()

    # Handle Simulation mode
    if args.sim:
        sim_thread = threading.Thread(target=run_simulation, args=(tracker, stop_event), daemon=True)
        sim_thread.start()
        port_info = "SIMULATION"
    else:
        # Detect Serial Port
        port = args.port or find_esp32_port()
        if not port:
            print(f"{CLR_RED}[ERROR] No serial port detected!{CLR_RESET}")
            print("Connect your OBD2 USB cable or ESP32, or run in simulation mode:")
            print("  python tools/can_sniffer.py --sim")
            sys.exit(1)

        is_elm = args.elm or (args.port is None and "COM3" in str(port))
        default_baud = 38400 if is_elm else 115200
        baud = args.baud or default_baud

        port_info = f"{port} @ {baud}bps" + (" (ELM327)" if is_elm else "")
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            print(f"{CLR_RED}[ERROR] Failed to open serial port {port}: {e}{CLR_RESET}")
            sys.exit(1)

        if is_elm:
            fid = list(tracker.filter_ids)[0] if tracker.filter_ids else None
            init_elm327(ser, filter_id=fid)

        # Serial Reader Thread with robust CR/LF buffer processing & auto-restart on BUFFER FULL
        def serial_worker():
            buf = ""
            while not stop_event.is_set():
                try:
                    num_avail = ser.in_waiting or 1
                    chunk = ser.read(num_avail).decode('latin-1', errors='ignore')
                    if chunk:
                        buf += chunk
                        while '\r' in buf or '\n' in buf:
                            idx_cr = buf.find('\r')
                            idx_lf = buf.find('\n')
                            if idx_cr != -1 and idx_lf != -1:
                                idx = min(idx_cr, idx_lf)
                            elif idx_cr != -1:
                                idx = idx_cr
                            else:
                                idx = idx_lf
                            raw_line = buf[:idx].strip()
                            buf = buf[idx + 1:]
                            if raw_line:
                                uline = raw_line.upper()
                                if "BUFFER FULL" in uline or uline == ">" or "STOPPED" in uline:
                                    if is_elm:
                                        ser.write(b"ATMA\r")
                                else:
                                    if raw_line.startswith(">"):
                                        raw_line = raw_line[1:].strip()
                                    frame = parse_incoming_line(raw_line)
                                    if frame:
                                        tracker.process_frame(frame)
                except Exception:
                    time.sleep(0.01)

        reader_thread = threading.Thread(target=serial_worker, daemon=True)
        reader_thread.start()

        if is_elm:
            ser.reset_input_buffer()
            ser.write(b"ATMA\r")

    print(f"{CLR_GREEN}[OK] Sniffer running. Logging to {csv_file} and {log_file}{CLR_RESET}")
    time.sleep(0.5)

    try:
        # Refresh dashboard at 10 Hz
        while not stop_event.is_set():
            render_dashboard(tracker, port_info, args.sim)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if not args.sim:
            try:
                ser.write(b" \r\n")
                ser.close()
            except Exception:
                pass
        tracker.close()
        print(f"\n{CLR_GREEN}[OK] Stopped. Total captured frames: {tracker.total_frames:,}{CLR_RESET}")
        print(f"     CSV Log:     {os.path.abspath(csv_file)}")
        print(f"     Candump Log: {os.path.abspath(log_file)}\n")

if __name__ == "__main__":
    main()
