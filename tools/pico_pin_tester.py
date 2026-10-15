"""
Raspberry Pi Pico MCP2515 Hardware Pin & Wire Diagnostic Tool
=============================================================
Run this in Thonny to immediately identify the exact wiring problem.
"""

from machine import Pin, SoftSPI
import time

PIN_CS   = 17 # Physical Pin 22
PIN_SCK  = 18 # Physical Pin 24
PIN_MOSI = 19 # Physical Pin 25
PIN_MISO = 16 # Physical Pin 21

print("=" * 60)
print("     RASPBERRY PI PICO MCP2515 HARDWARE DIAGNOSTIC      ")
print("=" * 60)

# TEST 1: Check if any SPI pin is shorted to Ground or VCC
print("\n[TEST 1] Checking GPIO Pin Health (Short-Circuit Test)...")
pins = [
    ("GP16 (MISO / SO)", PIN_MISO),
    ("GP17 (CS)", PIN_CS),
    ("GP18 (SCK)", PIN_SCK),
    ("GP19 (MOSI / SI)", PIN_MOSI),
]

for name, p_num in pins:
    p = Pin(p_num, Pin.IN, Pin.PULL_UP)
    time.sleep_ms(5)
    val_up = p.value()
    
    p = Pin(p_num, Pin.IN, Pin.PULL_DOWN)
    time.sleep_ms(5)
    val_down = p.value()
    
    if val_up == 0 and val_down == 0:
        print(f"  [!] {name}: STUCK TO GND (0V)! Pin is grounded or shorted.")
    elif val_up == 1 and val_down == 1:
        print(f"  [*] {name}: Held HIGH (3.3V/5V) or externally pulled up.")
    else:
        print(f"  [OK] {name}: Normal floating pin (PULLUP=1, PULLDOWN=0).")

# TEST 2: SPI Loopback Check (Tests if SPI controller can transmit/receive)
print("\n[TEST 2] SPI Bus Pulse Test...")
spi = SoftSPI(baudrate=1000000, polarity=0, phase=0, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
cs = Pin(PIN_CS, Pin.OUT, value=1)

# Toggle CS and clock
cs.value(0)
tx = bytearray([0xC0]) # Reset command
spi.write(tx)
cs.value(1)
time.sleep_ms(10)

# Try reading CANSTAT (0x0E) and CANCTRL (0x0F)
cs.value(0)
tx = bytearray([0x03, 0x0E, 0x00, 0x00])
rx = bytearray(4)
spi.write_readinto(tx, rx)
cs.value(1)

canstat = rx[2]
canctrl = rx[3]

print(f"  Raw SPI response bytes: {[hex(b) for b in rx]}")
print(f"  CANSTAT Register: 0x{canstat:02X} (Expected: 0x80)")
print(f"  CANCTRL Register: 0x{canctrl:02X} (Expected: 0x80 or 0x87)")

if canstat == 0x80:
    print("\n>>> [SUCCESS!] MCP2515 is ALIVE and in CONFIG mode! <<<")
elif canstat == 0x00 and canctrl == 0x00:
    print("\n>>> [RESULT: ALL ZEROS (0x00)] <<<")
    print("MISO line is not receiving any pulses from the MCP2515.")
elif canstat == 0xFF:
    print("\n>>> [RESULT: ALL ONES (0xFF)] <<<")
    print("MISO line is high, MCP2515 is unpowered or CS not selecting.")

print("\n" + "=" * 60)
