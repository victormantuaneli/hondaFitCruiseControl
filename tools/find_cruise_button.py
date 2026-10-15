#!/usr/bin/env python3
"""
ISOLADOR DE BOTÃO DE CRUISE CONTROL (HONDA FIT)
Compara os bytes do barramento CAN com o botão SOLTO vs APERTADO.
"""
import serial
import time
from collections import defaultdict

def sample_can(ser, duration_sec, prompt_msg):
    print("\n" + "=" * 65)
    print(f"👉 {prompt_msg}")
    print("=" * 65)
    
    # Contagem regressiva
    for i in range(3, 0, -1):
        print(f"Iniciando captura em {i}...", end="\r")
        time.sleep(1.0)
    print("Gravando dados... MANTENHA A AÇÃO!                  ")
    
    ser.reset_input_buffer()
    ser.write(b"ATMA\r")
    
    t0 = time.time()
    frames = defaultdict(list)
    buf = ""
    
    while time.time() - t0 < duration_sec:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n).decode('latin-1', errors='ignore')
            buf += chunk
            while '\r' in buf or '\n' in buf:
                idx = min([i for i in [buf.find('\r'), buf.find('\n')] if i != -1])
                line = buf[:idx].strip()
                buf = buf[idx+1:]
                if line and not any(k in line.upper() for k in ["ATMA", "BUFFER", "STOPPED", "OK", ">"]):
                    if line.startswith(">"): line = line[1:].strip()
                    tokens = line.split()
                    if len(tokens) >= 2:
                        first = tokens[0]
                        if len(first) in (3, 4) and all(c in "0123456789ABCDEFabcdef" for c in first):
                            try:
                                cid = int(first, 16)
                                if len(tokens[1]) == 1 and tokens[1].isdigit():
                                    data = "".join(tokens[2:])
                                else:
                                    data = "".join(tokens[1:])
                                if data:
                                    frames[cid].append(data)
                            except: pass
        time.sleep(0.005)
        
    ser.write(b" \r")
    time.sleep(0.2)
    ser.reset_input_buffer()
    print("✅ Captura concluída!")
    return frames

def main():
    print("Conectando ao cabo OBD2 no COM3...")
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    
    # Reset e config
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
        
    print("\n[CALIBRAÇÃO DO BOTÃO DO CRUISE]")
    print("Vamos fazer 2 capturas de 3 segundos:")
    print("1. Com todos os botões SOLTOS.")
    print("2. Com o botão de CRUISE APERTADO/LIGADO.")
    
    # Passo 1: Solto
    input("\nPressione ENTER para capturar com os BOTÕES SOLTOS...")
    idle_frames = sample_can(ser, 3.5, "DEIXE TODOS OS BOTÕES DO CRUISE SOLTOS/DESLIGADOS")
    
    # Passo 2: Apertado
    input("\nPressione ENTER para preparar a captura com o BOTÃO APERTADO...")
    pressed_frames = sample_can(ser, 3.5, "APERTE E SEGURE O BOTÃO DO CRUISE (MAIN ou SET) AGORA!")
    
    ser.close()
    
    print("\n" + "=" * 65)
    print("             RESULTADO DA ANÁLISE DIFERENCIAL              ")
    print("=" * 65)
    
    candidates = []
    
    # Analisa IDs comuns entre as duas capturas
    all_cids = sorted(set(idle_frames.keys()) | set(pressed_frames.keys()))
    
    for cid in all_cids:
        f_idle = set(idle_frames.get(cid, []))
        f_press = set(pressed_frames.get(cid, []))
        
        # Payloads que apareceram quando apertou mas NUNCA apareceram quando solto
        new_payloads = f_press - f_idle
        
        if new_payloads:
            candidates.append((cid, list(f_idle)[:1], list(new_payloads)[:3]))
            
    if not candidates:
        print("❌ Nenhuma mudança de byte foi detectada entre solto e apertado.")
        print("Possíveis causas:")
        print("1. O botão não foi mantido pressionado durante a segunda captura.")
        print("2. O painel não repassa esse botão para a rede F-CAN de 500k.")
    else:
        print(f"🎉 FORAM ENCONTRADOS {len(candidates)} CANDIDATOS COM BYTES ALTERADOS!\n")
        for cid, idles, presseds in candidates:
            print(f"📍 CAN ID: 0x{cid:03X}")
            print(f"   Payload Normal (Solto):    {idles[0] if idles else 'Nenhum'}")
            print(f"   Payload Ativo (Apertado):  {presseds[0]}")
            print("-" * 50)

if __name__ == "__main__":
    main()
