#!/usr/bin/env python3
"""
INJETOR DE FRAMES CAN PARA DESCOBERTA DA LUZ "CRUISE CONTROL" (SET)
Envia rajadas de pacotes CAN para os IDs candidatos e testa os bits no painel.
"""
import serial
import time
import sys

CANDIDATES = [
    {
        "name": "0x17C - Confirmacao Definitiva: Bit 3 (0x08) - CRUISE CONTROL",
        "id": "17C",
        "payloads": [
            ("00 00 02 BD 08 00 00 28", "FASE 1: LIGA A LUZ POR 4 SEGUNDOS (Byte 4 = 0x08)"),
            ("00 00 02 BD 00 00 00 28", "FASE 2: DESLIGA A LUZ POR 2 SEGUNDOS (Byte 4 = 0x00)"),
            ("00 00 02 BD 08 00 00 28", "FASE 3: LIGA A LUZ NOVAMENTE POR 4 SEGUNDOS (Byte 4 = 0x08)"),
            ("00 00 02 BD 00 00 00 28", "FASE 4: DESLIGA A LUZ (Byte 4 = 0x00)"),
        ]
    },
    {
        "name": "0x1A4 - PCM / VSA / Cruise Dynamic Control",
        "id": "1A4",
        "payloads": [
            ("01 00 00 00 00 00 00 27", "Byte 0 = 0x01"),
            ("02 00 00 00 00 00 00 27", "Byte 0 = 0x02"),
            ("04 00 00 00 00 00 00 27", "Byte 0 = 0x04"),
            ("08 00 00 00 00 00 00 27", "Byte 0 = 0x08"),
            ("10 00 00 00 00 00 00 27", "Byte 0 = 0x10"),
            ("00 01 00 00 00 00 00 27", "Byte 1 = 0x01"),
            ("00 02 00 00 00 00 00 27", "Byte 1 = 0x02"),
            ("00 00 01 00 00 00 00 27", "Byte 2 = 0x01"),
            ("00 00 00 01 00 00 00 27", "Byte 3 = 0x01"),
        ]
    },
    {
        "name": "0x188 - Instrument / Gateway Lamp Status",
        "id": "188",
        "payloads": [
            ("01 00 00 01 00 24", "Byte 0 = 0x01"),
            ("02 00 00 01 00 24", "Byte 0 = 0x02"),
            ("04 00 00 01 00 24", "Byte 0 = 0x04"),
            ("08 00 00 01 00 24", "Byte 0 = 0x08"),
            ("00 01 00 01 00 24", "Byte 1 = 0x01"),
            ("00 02 00 01 00 24", "Byte 1 = 0x02"),
            ("00 04 00 01 00 24", "Byte 1 = 0x04"),
            ("00 00 01 01 00 24", "Byte 2 = 0x01"),
            ("00 00 02 01 00 24", "Byte 2 = 0x02"),
            ("00 00 04 01 00 24", "Byte 2 = 0x04"),
        ]
    },
    {
        "name": "0x3D7 - PCM Indicators",
        "id": "3D7",
        "payloads": [
            ("B1 00 00 00 00 00 00 51", "Byte 0 = 0xB1 (Bit 0 ativo)"),
            ("B2 00 00 00 00 00 00 51", "Byte 0 = 0xB2 (Bit 1 ativo)"),
            ("B4 00 00 00 00 00 00 51", "Byte 0 = 0xB4 (Bit 2 ativo)"),
            ("B8 00 00 00 00 00 00 51", "Byte 0 = 0xB8 (Bit 3 ativo)"),
            ("B0 01 00 00 00 00 00 51", "Byte 1 = 0x01"),
            ("B0 02 00 00 00 00 00 51", "Byte 1 = 0x02"),
            ("B0 04 00 00 00 00 00 51", "Byte 1 = 0x04"),
            ("B0 00 01 00 00 00 00 51", "Byte 2 = 0x01"),
            ("B0 00 02 00 00 00 00 51", "Byte 2 = 0x02"),
            ("B0 00 04 00 00 00 00 51", "Byte 2 = 0x04"),
            ("B0 00 00 01 00 00 00 51", "Byte 3 = 0x01"),
        ]
    },
    {
        "name": "0x324 - Cruise Set Speed / State",
        "id": "324",
        "payloads": [
            ("7F 54 11 27 46 00 00 14", "Byte 4 = 0x46 (70 km/h Set Speed)"),
            ("7F 54 11 27 01 00 00 14", "Byte 4 = 0x01 (Cruise Flag)"),
            ("7F 54 11 27 80 00 00 14", "Byte 4 = 0x80 (Bit 7)"),
            ("7F 54 11 27 00 46 00 14", "Byte 5 = 0x46 (Set Speed)"),
            ("00 00 00 00 46 00 00 00", "Simulação pura Civic/Fit Cruise State"),
            ("00 00 00 01 46 00 00 00", "Cruise On + 70 km/h Set Speed"),
            ("01 00 00 00 46 00 00 00", "Byte 0 = 0x01 + 70 km/h"),
            ("FF FF FF FF FF FF FF FF", "Broadcast All HIGH"),
        ]
    },
    {
        "name": "0x305 - Cluster Indicator / Status",
        "id": "305",
        "payloads": [
            ("81 26", "Byte 0 = 0x81 (Bit 0)"),
            ("82 26", "Byte 0 = 0x82 (Bit 1)"),
            ("84 26", "Byte 0 = 0x84 (Bit 2)"),
            ("88 26", "Byte 0 = 0x88 (Bit 3)"),
            ("90 26", "Byte 0 = 0x90 (Bit 4)"),
            ("A0 26", "Byte 0 = 0xA0 (Bit 5)"),
            ("C0 26", "Byte 0 = 0xC0 (Bit 6)"),
            ("80 27", "Byte 1 = 0x27 (Bit 0)"),
            ("80 28", "Byte 1 = 0x28"),
            ("80 36", "Byte 1 = 0x36"),
            ("FF FF", "Todos os bits ligados"),
        ]
    }
]

def init_ser():
    ser = serial.Serial('COM3', 38400, timeout=0.1)
    ser.write(b" \r\n \r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"ATZ\r")
    time.sleep(0.8)
    ser.reset_input_buffer()
    for cmd in [b"ATE0\r", b"ATL0\r", b"ATSP6\r", b"ATAL\r", b"ATCAF0\r", b"ATR0\r"]:
        ser.write(cmd)
        time.sleep(0.08)
        ser.reset_input_buffer()
    return ser

def make_honda_frame(can_id_int, payload_7bytes, counter):
    s = sum((b & 0xF) + (b >> 4) for b in payload_7bytes)
    s += counter
    s += sum((can_id_int >> (4*i)) & 0xF for i in range(3))
    checksum = (8 - s) & 0xF
    b7 = (counter << 4) | checksum
    full_payload = payload_7bytes + bytes([b7])
    return full_payload.hex(' ').upper()

def send_burst(ser, can_id_str, payload_template, duration_sec=3.0):
    can_id_int = int(can_id_str, 16)
    ser.write(f"ATSH {can_id_str}\r".encode())
    time.sleep(0.05)
    ser.reset_input_buffer()
    
    # Se o payload tiver 7 ou 8 bytes, extrai os primeiros 7 bytes para calcular o checksum dinâmico
    raw_bytes = bytes.fromhex(payload_template)
    payload_7 = raw_bytes[:7]
    
    t_end = time.time() + duration_sec
    count = 0
    counter = 0
    while time.time() < t_end:
        # Se o frame for de 8 bytes com checksum (maioria dos IDs de controle da Honda)
        if len(raw_bytes) >= 7:
            frame_str = make_honda_frame(can_id_int, payload_7, counter)
        else:
            frame_str = payload_template
            
        ser.write(frame_str.encode() + b"\r")
        count += 1
        counter = (counter + 1) & 0x03 # Rolling counter: 0, 1, 2, 3
        time.sleep(0.015) # ~65 Hz
        
    ser.reset_input_buffer()
    return count

def main():
    target_filter = sys.argv[1].upper() if len(sys.argv) > 1 else None
    if target_filter and target_filter.startswith("0X"):
        target_filter = target_filter[2:]
        
    print("=" * 65)
    print("   INJETOR DE TESTE DA LUZ 'CRUISE CONTROL' (SET)")
    print("   Certifique-se de que o CRUISE MAIN esta LIGADO no painel!")
    print("=" * 65)
    
    selected_cats = CANDIDATES
    if target_filter and target_filter != "ALL":
        selected_cats = [c for c in CANDIDATES if c["id"].upper() == target_filter]
        if not selected_cats:
            print(f"Erro: ID 0x{target_filter} nao esta na lista de candidatos.")
            print(f"Disponiveis: {[c['id'] for c in CANDIDATES]}")
            return
            
    ser = init_ser()
    print("\nAdaptador COM3 configurado para injecao raw em 500 kbps!\n")
    
    test_idx = 1
    total_tests = sum(len(c["payloads"]) for c in selected_cats)
    
    try:
        for cat in selected_cats:
            print("\n" + "=" * 65)
            print(f"CATEGORIA: {cat['name']}")
            print("=" * 65)
            
            for payload, desc in cat["payloads"]:
                print(f"\n[{test_idx}/{total_tests}] Testando ID 0x{cat['id']}: {desc}")
                print(f"       Payload: {payload}")
                print("       --> ENVIANDO RAJADA DE 3 SEGUNDOS (OLHE PARA O PAINEL!)...")
                
                frames = send_burst(ser, cat['id'], payload, duration_sec=3.0)
                print(f"       --> {frames} frames enviados!")
                test_idx += 1
                time.sleep(1.0)
                
    except KeyboardInterrupt:
        print("\nTeste interrompido pelo usuario.")
    finally:
        ser.write(b" \r")
        ser.close()
        print("\nInjetor finalizado.")

if __name__ == "__main__":
    main()
