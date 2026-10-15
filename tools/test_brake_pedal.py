#!/usr/bin/env python3
"""
MONITOR DO SENSOR DE FREIO VIA CAN BUS (HONDA F-CAN)
Mede o status do pedal de freio em tempo real via IDs 0x17C e 0x1A4.
"""
import serial
import time
import sys

def main():
    target_id = sys.argv[1].upper() if len(sys.argv) > 1 else "17C"
    if target_id.startswith("0X"):
        target_id = target_id[2:]
        
    print(f"\n==================================================================")
    print(f"  MONITORANDO STATUS DO PEDAL DE FREIO NO CAN ID 0x{target_id}")
    print(f"  Pressione e solte o PEDAL DE FREIO...")
    print(f"==================================================================\n")
    
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"ATWS\r")
    time.sleep(0.8)
    ser.reset_input_buffer()
    
    commands = [
        b"ATE0\r",
        b"ATL1\r",
        b"ATH1\r",
        b"ATSP6\r",
        b"ATCAF0\r",
        f"ATCRA {target_id}\r".encode(),
        b"ATMA\r"
    ]
    for c in commands:
        ser.write(c)
        time.sleep(0.08)
        ser.reset_input_buffer()
        
    last_core = None
    buf = ""
    frame_count = 0
    
    try:
        while True:
            n = ser.in_waiting
            if n:
                buf += ser.read(n).decode('latin-1', errors='ignore')
                while '\r' in buf or '\n' in buf:
                    idx = min([i for i in [buf.find('\r'), buf.find('\n')] if i != -1])
                    line = buf[:idx].strip()
                    buf = buf[idx+1:]
                    
                    if line.startswith(target_id):
                        frame_count += 1
                        tokens = line.split()
                        if len(tokens) >= 2:
                            bytes_list = tokens[1:]
                            # Ignora o último byte (checksum/counter)
                            core = tuple(bytes_list[:-1])
                            
                            if core != last_core:
                                old = last_core
                                last_core = core
                                t_str = time.strftime('%H:%M:%S')
                                
                                if old is not None:
                                    diffs = [f"B{i}: {old[i]}->{core[i]}" for i in range(min(len(old), len(core))) if old[i] != core[i]]
                                    print(f"[{t_str}] 🚨 FREIO/MUDANCA DETECTADA: {', '.join(diffs)}")
                                else:
                                    print(f"[{t_str}] Estado Inicial: {' '.join(core)}")
                                print(f"   Payload: {' '.join(bytes_list)}")
                                print("-" * 55)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nMonitor encerrado.")
    finally:
        ser.write(b" \r")
        ser.close()

if __name__ == "__main__":
    main()
