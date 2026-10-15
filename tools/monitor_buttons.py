#!/usr/bin/env python3
"""
MONITOR FILTRADO DE BOTÕES (IGNORA COUNTER/CHECKSUM AUTOMOTIVO)
Mostra apenas mudanças reais de estado dos botões.
"""
import serial
import time

TARGET_IDS = [0x164, 0x1A4, 0x188, 0x294, 0x405, 0x13F]

def main():
    print("Conectando ao adaptador no COM3...")
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"ATWS\r")
    time.sleep(0.8)
    ser.reset_input_buffer()
    
    for c in [b"ATE0\r", b"ATL1\r", b"ATH1\r", b"ATSP6\r", b"ATCAF0\r"]:
        ser.write(c)
        time.sleep(0.15)
        ser.reset_input_buffer()
        
    ser.write(b"ATMA\r")
    print("\n" + "=" * 65)
    print("  MONITOR INTELIGENTE: COUNTER/CHECKSUM IGNORADOS!         ")
    print("  A tela ficará QUIETA até você apertar algum botão!       ")
    print("=" * 65)
    
    last_state = {}
    buf = ""
    
    try:
        while True:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n).decode('latin-1', errors='ignore')
                buf += chunk
                while '\r' in buf or '\n' in buf:
                    idx = min([i for i in [buf.find('\r'), buf.find('\n')] if i != -1])
                    line = buf[:idx].strip()
                    buf = buf[idx+1:]
                    if line:
                        if "BUFFER" in line.upper() or line == ">":
                            ser.write(b"ATMA\r")
                            continue
                        if line.startswith(">"): line = line[1:].strip()
                        tokens = line.split()
                        if len(tokens) >= 2:
                            first = tokens[0]
                            if len(first) in (3, 4) and all(c in "0123456789ABCDEFabcdef" for c in first):
                                try:
                                    cid = int(first, 16)
                                    if cid in TARGET_IDS:
                                        if len(tokens[1]) == 1 and tokens[1].isdigit():
                                            bytes_list = tokens[2:]
                                        else:
                                            bytes_list = tokens[1:]
                                        
                                        # Remove o último byte (Rolling Counter / Checksum que muda sempre)
                                        if len(bytes_list) > 1:
                                            data_core = tuple(bytes_list[:-1])
                                        else:
                                            data_core = tuple(bytes_list)
                                            
                                        old_core = last_state.get(cid)
                                        if old_core is not None and data_core != old_core:
                                            print(f"[{time.strftime('%H:%M:%S')}] ID 0x{cid:03X} MUDOU!")
                                            print(f"   Antes: {' '.join(old_core)}")
                                            print(f"   Agora: {' '.join(data_core)}")
                                            print("-" * 45)
                                        last_state[cid] = data_core
                                except: pass
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b" \r")
        ser.close()

if __name__ == "__main__":
    main()
