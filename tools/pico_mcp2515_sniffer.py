"""
MCP2515 MONITOR COM SOFTSPI NATIVO (PRECISÃO DE HARDWARE)
Elimina o jitter do Python usando o motor SoftSPI C do MicroPython.
Pinos: CS=GP17 (22), SCK=GP18 (24), SI=GP19 (25), SO=GP21 (27)
"""
from machine import Pin, SoftSPI
import time

PIN_CS   = 17 # Furo 22 (GP17)
PIN_SCK  = 18 # Furo 24 (GP18)
PIN_MOSI = 19 # Furo 25 (GP19) - SI
PIN_MISO = 21 # Furo 27 (GP21) - SO

cs = Pin(PIN_CS, Pin.OUT, value=1)

# SoftSPI nativo compilado em C (Modo 0,0 a 100 kHz)
spi = SoftSPI(
    baudrate=100000,
    polarity=0,
    phase=0,
    sck=Pin(PIN_SCK),
    mosi=Pin(PIN_MOSI),
    miso=Pin(PIN_MISO)
)

def mcp_reset():
    cs.value(0)
    spi.write(bytearray([0xC0])) # Reset SPI
    cs.value(1)
    time.sleep_ms(20)

def mcp_read(reg):
    cs.value(0)
    tx = bytearray([0x03, reg, 0x00])
    rx = bytearray(3)
    spi.write_readinto(tx, rx)
    cs.value(1)
    return rx[2]

def mcp_write(reg, val):
    cs.value(0)
    spi.write(bytearray([0x02, reg, val]))
    cs.value(1)

print("=" * 60)
print("     MCP2515 MONITOR: SOFTSPI NATIVO (100 kHz)     ")
print("=" * 60)

mcp_reset()

count = 0
ok_streak = 0

while True:
    count += 1
    stat = mcp_read(0x0E) # CANSTAT (esperado 0x80)
    ctrl = mcp_read(0x0F) # CANCTRL (esperado 0x87 ou 0x80)
    
    if stat == 0x80:
        ok_streak += 1
        print(f"[{count:03d}] 🎉 SUCESSO TOTAL! CANSTAT=0x{stat:02X} | CANCTRL=0x{ctrl:02X} (Streak: {ok_streak})")
    else:
        ok_streak = 0
        status_msg = "Preso em 1" if stat == 0xFF else ("Preso em 0" if stat == 0x00 else "Transição")
        print(f"[{count:03d}] CANSTAT=0x{stat:02X} | CANCTRL=0x{ctrl:02X} -> {status_msg}")
        
    time.sleep_ms(300)
