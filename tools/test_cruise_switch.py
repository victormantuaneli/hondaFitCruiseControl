#!/usr/bin/env python3
"""
MONITOR MULTI-ID DE ALTA PRECISÃO PARA BOTÕES DO HONDA FIT
Filtra por hardware no chip ELM327 qualquer ID desejado:
Exemplos de uso:
  python tools/test_cruise_switch.py 294   (Padrão: SCM_BUTTONS Honda)
  python tools/test_cruise_switch.py 1A4   (Transmissão/PCM)
  python tools/test_cruise_switch.py 188
  python tools/test_cruise_switch.py 405   (Painel de Instrumentos)
"""
import serial
import time
import sys

def main():
    target_id_str = sys.argv[1].upper() if len(sys.argv) > 1 else "164"
    if target_id_str.startswith("0X"):
        target_id_str = target_id_str[2:]
        
    print(f"Conectando ao adaptador no COM3 para monitorar ID 0x{target_id_str}...")
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    
    # Abort e warm reset
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"ATWS\r")
    time.sleep(0.8)
    ser.reset_input_buffer()
    
    # Filtro de hardware direto no chip ELM327
    commands = [
        b"ATE0\r",
        b"ATL1\r",
        b"ATH1\r",
        b"ATSP6\r",
        b"ATCAF0\r",
        f"ATCRA {target_id_str}\r".encode(),
        b"ATMA\r"
    ]
    for c in commands:
        ser.write(c)
        time.sleep(0.08)
        
    print("\n" + "=" * 65)
    print(f"  MONITORANDO OFICIALMENTE O CAN ID 0x{target_id_str} (PAINEL HONDA)")
    print("  Decodificador Oficial Ativo:")
    print("    - CRUISE MAIN: Bit 5 (0x20) [Pino 22 do Painel]")
    print("    - RES / ACCEL: Bit 7 (0x80) [Pino 19 do Painel]")
    print("    - SET / DECEL: Bit 6 (0x40) [Pino 20 do Painel]")
    print("    - CANCEL:      Bit 7 + 6 (0xC0)")
    print("=" * 65 + "\n")
    
    last_payload = None
    buf = ""
    frame_count = 0
    last_rx_time = time.time()
    last_heartbeat = time.time()
    
    try:
        while True:
            n = ser.in_waiting
            now = time.time()
            
            if n:
                chunk = ser.read(n).decode('latin-1', errors='ignore')
                buf += chunk
                last_rx_time = now
                
                while '\r' in buf or '\n' in buf:
                    idx = min([i for i in [buf.find('\r'), buf.find('\n')] if i != -1])
                    line = buf[:idx].strip()
                    buf = buf[idx+1:]
                    
                    if line:
                        if "BUFFER" in line.upper():
                            time.sleep(0.05)
                            ser.reset_input_buffer()
                            ser.write(b" \r")
                            time.sleep(0.05)
                            ser.write(b"ATMA\r")
                            continue
                        if line == ">":
                            ser.write(b"ATMA\r")
                            continue
                            
                        if line.startswith(">"): line = line[1:].strip()
                        tokens = line.split()
                        
                        if len(tokens) >= 2 and tokens[0].upper() == target_id_str:
                            frame_count += 1
                            bytes_list = [t.upper() for t in tokens[1:]]
                            b0_hex = bytes_list[0]
                            
                            if b0_hex != last_payload:
                                old_b0 = last_payload
                                last_payload = b0_hex
                                
                                b0_int = int(b0_hex, 16)
                                print(f"\n[{time.strftime('%H:%M:%S')}] >>> B0: {old_b0 or 'INICIAL'} -> {b0_hex} (bin: {b0_int:08b})")
                                
                                # Decodificação exata do ID 0x164
                                main_on = bool(b0_int & 0x20)  # Bit 5
                                res_on  = bool(b0_int & 0x80)  # Bit 7
                                set_on  = bool(b0_int & 0x40)  # Bit 6
                                
                                if not main_on:
                                    status_str = "⚪ CRUISE MAIN DESLIGADO"
                                else:
                                    if res_on and set_on:
                                        status_str = "🔴 CANCEL PRESSIONADO (RES + SET)"
                                    elif res_on:
                                        status_str = "🔴 RES / ACCEL PRESSIONADO"
                                    elif set_on:
                                        status_str = "🔴 SET / DECEL PRESSIONADO"
                                    else:
                                        status_str = "🟢 CRUISE MAIN LIGADO (AGUARDANDO COMANDO)"
                                
                                print(f"   👉 STATUS: {status_str}")
                                print(f"   Payload Completo: {' '.join(bytes_list)}")
                                print("-" * 55)
            
            # Watchdog: se não receber dados por 1.5s, reseta o monitor
            if now - last_rx_time > 1.5:
                ser.write(b" \r")
                time.sleep(0.05)
                ser.reset_input_buffer()
                ser.write(b"ATMA\r")
                last_rx_time = now
            
            # Heartbeat discreto a cada 1s mostrando frames recebidos e B0 atual
            if now - last_heartbeat >= 1.0:
                cur_b0 = last_payload if last_payload else "--"
                sys.stdout.write(f"\r  [ATIVO] Frames 0x{target_id_str}: {frame_count:5d} | B0 atual: {cur_b0}   ")
                sys.stdout.flush()
                last_heartbeat = now
                
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b" \r")
        ser.close()
        print("\nMonitor encerrado.")

if __name__ == "__main__":
    main()
