"""
Deep Hardware SPI Debugger for MCP2515 on Raspberry Pi Pico
===========================================================
Tests SPI Mode 0, Mode 1, clock frequencies, and all control registers.
"""

from machine import Pin, SPI, SoftSPI
import time

PIN_CS   = 17 # GP17 (Physical Pin 22)
PIN_SCK  = 18 # GP18 (Physical Pin 24)
PIN_MOSI = 19 # GP19 (Physical Pin 25)
PIN_MISO = 16 # GP16 (Physical Pin 21)

print("=" * 60)
print("     MCP2515 DEEP HARDWARE REGISTER PROBE      ")
print("=" * 60)

cs = Pin(PIN_CS, Pin.OUT, value=1)

def test_spi_config(name, spi_obj):
    print(f"\n--- Testing: {name} ---")
    
    # 1. Reset chip
    cs.value(0)
    spi_obj.write(bytearray([0xC0])) # Reset command
    cs.value(1)
    time.sleep_ms(20)
    
    # 2. Read CANSTAT (0x0E), CANCTRL (0x0F), and CANINTF (0x2C)
    regs = [0x0E, 0x0F, 0x2C, 0x0C]
    results = {}
    for r in regs:
        cs.value(0)
        tx = bytearray([0x03, r, 0x00])
        rx = bytearray(3)
        spi_obj.write_readinto(tx, rx)
        cs.value(1)
        results[r] = rx[2]
        
    stat = results[0x0E]
    ctrl = results[0x0F]
    intf = results[0x2C]
    bfp  = results[0x0C]
    
    print(f"  CANSTAT (0x0E): 0x{stat:02X} (Expected: 0x80)")
    print(f"  CANCTRL (0x0F): 0x{ctrl:02X} (Expected: 0x87 or 0x80)")
    print(f"  CANINTF (0x2C): 0x{intf:02X} (Expected: 0x00)")
    
    if stat == 0x80 or ctrl in (0x80, 0x87):
        print(f"  >>> SUCCESS with {name}! MCP2515 IS RESPONDING! <<<")
        return True
    return False

success = False

# Config 1: SoftSPI Mode 0 (100 kHz)
try:
    s1 = SoftSPI(baudrate=100000, polarity=0, phase=0, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
    if test_spi_config("SoftSPI Mode 0 (100 kHz)", s1): success = True
except Exception as e:
    print("  SoftSPI Mode 0 Error:", e)

# Config 2: SoftSPI Mode 1 (100 kHz)
if not success:
    try:
        s2 = SoftSPI(baudrate=100000, polarity=1, phase=1, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
        if test_spi_config("SoftSPI Mode 1 (100 kHz)", s2): success = True
    except Exception as e:
        print("  SoftSPI Mode 1 Error:", e)

# Config 3: Hardware SPI0 Mode 0 (500 kHz)
if not success:
    try:
        h1 = SPI(0, baudrate=500000, polarity=0, phase=0, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
        if test_spi_config("Hardware SPI0 Mode 0 (500 kHz)", h1): success = True
    except Exception as e:
        print("  Hardware SPI0 Error:", e)

# Config 4: Hardware SPI0 Mode 1 (500 kHz)
if not success:
    try:
        h2 = SPI(0, baudrate=500000, polarity=1, phase=1, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))
        if test_spi_config("Hardware SPI0 Mode 1 (500 kHz)", h2): success = True
    except Exception as e:
        print("  Hardware SPI0 Mode 1 Error:", e)

print("\n" + "=" * 60)
if not success:
    print("CONCLUSION:")
    print("If all tests return 0xFF:")
    print("1. Check CS wire: Is GP17 securely connected to the 'CS' pin on the MCP2515?")
    print("2. Check SCK wire: Is GP18 securely connected to the 'SCK' pin on the MCP2515?")
    print("3. Check Crystal: Is the silver 8MHz oval crystal firmly soldered?")
print("=" * 60)
