import serial
import time
import sys
import io

# Garante suporte a UTF-8 no Windows
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def main():
    target_id = sys.argv[1].upper() if len(sys.argv) > 1 else "164"
    if target_id.startswith("0X"):
        target_id = target_id[2:]
        
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    
    print(f"\n==================================================================")
    print(f"  CAPTURANDO ID 0x{target_id} POR {duration:.0f} SEGUNDOS")
    print(f"  APERTE: SET (segure 2s), DEPOIS RES (segure 2s), DEPOIS CANCEL!")
    print(f"==================================================================\n")
    
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"ATWS\r")
    time.sleep(0.8)
    ser.reset_input_buffer()
    
    for cmd in [b"ATE0\r", b"ATL1\r", b"ATH1\r", b"ATSP6\r", b"ATCAF0\r", f"ATCRA {target_id}\r".encode(), b"ATMA\r"]:
        ser.write(cmd)
        time.sleep(0.08)
        
    t_end = time.time() + duration
    last_b0 = None
    buf = ""
    events = []
    
    while time.time() < t_end:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n).decode('latin-1', errors='ignore')
            buf += chunk
            while '\r' in buf or '\n' in buf:
                idx = min([i for i in [buf.find('\r'), buf.find('\n')] if i != -1])
                line = buf[:idx].strip()
                buf = buf[idx+1:]
                
                if line.startswith(target_id):
                    tokens = line.split()
                    if len(tokens) >= 2:
                        b0 = tokens[1].upper()
                        if b0 != last_b0:
                            t_str = time.strftime('%H:%M:%S')
                            b0_int = int(b0, 16)
                            event_msg = f"[{t_str}] >>> B0 MUDOU: {last_b0 or 'INICIAL'} -> {b0} (bin: {b0_int:08b}) | {line}"
                            print(event_msg)
                            events.append(event_msg)
                            last_b0 = b0
        time.sleep(0.005)
        
    ser.write(b" \r")
    ser.close()
    print(f"\nCaptura finalizada! Total de eventos detectados em B0: {len(events)}")

if __name__ == "__main__":
    main()
