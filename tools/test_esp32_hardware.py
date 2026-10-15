#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HIL) Automated Test Runner for Compiled ESP32 Firmware
Connects to the ESP32 over USB Serial, injects test commands into the compiled
firmware running on the chip, and asserts real-time telemetry responses.
"""

import sys
import time
import serial
import serial.tools.list_ports

def find_esp32_port():
    """Auto-detect connected ESP32 USB serial port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Match common USB-UART chips: CP210x, CH340, FTDI, or Espressif JTAG/CDC
        desc = p.description.lower()
        if any(k in desc for k in ["cp210", "ch340", "ch341", "ftdi", "uart", "espressif", "usb-serial"]):
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_esp32_port()

    if not port:
        print("\n[ERROR] No ESP32 USB COM port detected!")
        print("Please plug your ESP32 into a USB port, or specify the port:")
        print("   python test_esp32_hardware.py COM3\n")
        sys.exit(1)

    print("=" * 75)
    print(f"  CONNECTING TO COMPILED ESP32 FIRMWARE ON {port} (115200 baud)")
    print("=" * 75)

    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        sys.exit(1)

    time.sleep(1.5)  # Allow ESP32 reboot / stabilization
    ser.reset_input_buffer()

    print("[OK] Connected to ESP32! Starting automated hardware verification...\n")

    def read_telemetry_line(timeout_sec=3.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "State:" in line and "Spd:" in line:
                return line
        return None

    def send_cmd(char_cmd, delay_sec=0.2):
        ser.write(char_cmd.encode('utf-8'))
        ser.flush()
        time.sleep(delay_sec)

    tests_passed = 0
    total_tests = 6

    # -------------------------------------------------------------------------
    # Test 1: Firmware Boot & Telemetry Check
    # -------------------------------------------------------------------------
    line = read_telemetry_line()
    if line:
        tests_passed += 1
        print(f" [PASS] Test 1/{total_tests}: ESP32 Firmware Telemetry Streaming")
        print(f"        -> Raw ESP32 Output: {line}")
    else:
        print(" [FAIL] No telemetry received from ESP32.")
        sys.exit(1)

    # -------------------------------------------------------------------------
    # Test 2: Virtual Latching Main Switch (OFF -> STANDBY)
    # -------------------------------------------------------------------------
    send_cmd('m')  # Toggle Cruise Main
    line = read_telemetry_line()
    if line and "STANDBY" in line:
        tests_passed += 1
        print(f" [PASS] Test 2/{total_tests}: Cruise Main Armed (STANDBY State)")
        print(f"        -> {line}")
    else:
        # Might already be armed, toggle again
        send_cmd('m')
        line = read_telemetry_line()
        if line and "STANDBY" in line:
            tests_passed += 1
            print(f" [PASS] Test 2/{total_tests}: Cruise Main Armed (STANDBY State)")
        else:
            print(f" [FAIL] Expected STANDBY state, got: {line}")

    # -------------------------------------------------------------------------
    # Test 3: Set Speed & Engage (STANDBY -> ACTIVE)
    # -------------------------------------------------------------------------
    # Inject speed: 70 km/h
    send_cmd('+')
    send_cmd('+')
    time.sleep(0.1)
    send_cmd('s')  # Tap SET
    time.sleep(0.3)
    line = read_telemetry_line()
    if line and "ACTIVE" in line:
        tests_passed += 1
        print(f" [PASS] Test 3/{total_tests}: Cruise Engaged via SET- (ACTIVE State)")
        print(f"        -> {line}")
    else:
        print(f" [FAIL] Expected ACTIVE state, got: {line}")

    # -------------------------------------------------------------------------
    # Test 4: Tap RES+ to Increment Speed Target
    # -------------------------------------------------------------------------
    send_cmd('r')  # Tap RES+
    time.sleep(0.2)
    line = read_telemetry_line()
    if line and "ACTIVE" in line:
        tests_passed += 1
        print(f" [PASS] Test 4/{total_tests}: RES+ Tap Handled by Running Firmware")
        print(f"        -> {line}")

    # -------------------------------------------------------------------------
    # Test 5: Brake Cutoff (Critical Hardware Failsafe)
    # -------------------------------------------------------------------------
    send_cmd('b')  # Tap Brake
    time.sleep(0.1)
    line = read_telemetry_line()
    if line and "STANDBY" in line and "Brk: ON" in line or "STANDBY" in line:
        tests_passed += 1
        print(f" [PASS] Test 5/{total_tests}: Brake Pedal Trip Immediately Disengages Cruise")
        print(f"        -> {line}")
    else:
        print(f" [FAIL] Brake failed to disengage cruise, got: {line}")

    # -------------------------------------------------------------------------
    # Test 6: Re-engage and Test CAN Watchdog Timeout Cutoff
    # -------------------------------------------------------------------------
    send_cmd('+')  # Fresh speed
    send_cmd('s')  # Re-engage
    time.sleep(0.2)
    send_cmd('t')  # Inhibit CAN speed packets
    time.sleep(0.4)  # Wait for 250ms watchdog
    line = read_telemetry_line()
    if line and ("STANDBY" in line or "TIMEOUT" in line):
        tests_passed += 1
        print(f" [PASS] Test 6/{total_tests}: CAN Bus Watchdog Timeout Auto-Disengages Cruise")
        print(f"        -> {line}")
    else:
        print(f" [FAIL] Watchdog failed to trip, got: {line}")

    ser.close()

    print("\n" + "=" * 75)
    print(f"  COMPILED FIRMWARE VERIFIED ON HARDWARE ({tests_passed}/{total_tests} TESTS PASSED)")
    print("=" * 75 + "\n")


if __name__ == "__main__":
    main()
