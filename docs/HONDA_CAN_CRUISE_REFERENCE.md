# Referência Técnica: Piloto Automático via CAN Bus - Honda Fit GE (2ª Geração)

Este documento registra a engenharia reversa completa dos comandos do piloto automático do Honda Fit (GE6 / GE8, 2009–2014), incluindo a pinagem do painel de instrumentos (GCM) e a decodificação da rede F-CAN.

---

## 1. Conexões Físicas no Painel de Instrumentos (Gauge Control Module - GCM)

Os botões do piloto automático no volante da 2ª geração do Honda Fit conectam-se diretamente ao chicote do **Painel de Instrumentos** (através do carretel da buzina / cinta do airbag):

| Pino no Painel (GCM) | Cor Original (Chicote) | Função | Tipo de Sinal |
| :---: | :---: | :--- | :--- |
| **Comum** | Verde/Amarelo (`GRN/YEL`) | **GND (Terra)** | Referência comum do volante |
| **Pino 22** | Rosa (`PNK`) ou Verde Claro (`LT GRN`) | **CRUISE MAIN** | Chave com trava mecânica (Latching) para GND |
| **Pino 19** | Azul (`BLU`) ou Verde Claro/Preto (`LT GRN/BLK`) | **RES / ACCEL** | Botão momentâneo para GND |
| **Pino 20** | Vermelho (`RED`) ou Amarelo/Vermelho (`YEL/RED`) | **SET / DECEL** | Botão momentâneo para GND |
| **Pinos 19 + 20** | *(Diodos internos no volante)* | **CANCEL** | Aterra os pinos 19 e 20 em paralelo |

> **Nota de Circuito:** O painel possui resistores internos de pull-up para 5V. Ao pressionar qualquer tecla, o circuito correspondente é puxado para nível lógico baixo (GND).

---

## 2. Decodificação CAN Bus (Rede F-CAN @ 500 kbps)

O microcontrolador interno do painel processa as leituras analógicas/digitais dos pinos 19, 20 e 22 e faz o broadcast contínuo na rede **F-CAN** no identificador **`0x164`**.

* **CAN ID:** `0x164` (Standard 11-bit)
* **Taxa de Transmissão:** 500 kbps
* **Tamanho do Payload:** 8 Bytes (DLC 8)
* **Frequência de Envio:** ~50 Hz (a cada 20 ms)
* **Byte Responsável pelos Botões:** **Byte 0 (B0)**

---

## 3. Tabela Verdade de Bits do Byte 0 (`0x164` B0)

Os bits do Byte 0 são mapeados da seguinte forma:

$$\text{B0} = [\text{Bit 7: RES}] \quad [\text{Bit 6: SET}] \quad [\text{Bit 5: MAIN}] \quad [\text{Bit 4}] \quad [\text{Bit 3}] \quad [\text{Bit 2: BASE}] \quad [\text{Bit 1}] \quad [\text{Bit 0}]$$

| Tecla / Estado do Sistema | B0 Hex | B0 Binário | Bits Ativos | Payload Exemplo Capturado |
| :--- | :---: | :---: | :---: | :--- |
| **Cruise Desligado (Idle)** | `0x04` | `0000 0100` | Bit 2 | `04 00 40 5A 45 00 00 1C` |
| **CRUISE MAIN Ligado** | `0x24` | `0010 0100` | Bit 5 + Bit 2 | `24 00 40 5A 45 00 00 1A` |
| **RES / ACCEL Pressionado** | `0xA4` | `1010 0100` | **Bit 7** + Bit 5 + Bit 2 | `A4 00 40 58 45 00 00 23` |
| **SET / DECEL Pressionado** | `0x64` | `0110 0100` | **Bit 6** + Bit 5 + Bit 2 | `64 00 40 5A 45 00 00 16` |
| **CANCEL Pressionado** | `0xE4` | `1110 0100` | **Bit 7 + Bit 6** + Bit 5 + Bit 2 | `E4 00 40 46 63 10 65 39` |
| **RES Pressionado (Main OFF)** | `0x84` | `1000 0100` | Bit 7 + Bit 2 | `84 00 40 58 49 00 00 12` |

### Máscaras de Bits em C / C++ (ESP32):

```cpp
// Extração direta no driver TWAI / MCP2515
uint8_t b0 = message.data[0];

bool cruiseMainActive = (b0 & 0x20) != 0; // Bit 5
bool resPressed       = (b0 & 0x80) != 0; // Bit 7
bool setPressed       = (b0 & 0x40) != 0; // Bit 6
bool cancelPressed    = (resPressed && setPressed); // Bits 7 e 6 ativos juntos (0xC0)
```

---

## 4. Log Histórico de Validação (Sessão de Sniffing ao Vivo)

Durante o teste no veículo com mais de 53.000 frames capturados, a transição ocorreu com 100% de confiabilidade:

```text
[13:48:20] >>> B0: 04 -> 24 (bin: 00100100) -> CRUISE MAIN LIGADO (LED verde aceso)
[13:48:28] >>> B0: 24 -> A4 (bin: 10100100) -> RES / ACCEL PRESSIONADO
[13:48:28] >>> B0: A4 -> 24 (bin: 00100100) -> SOLTO
[13:48:43] >>> B0: 24 -> 64 (bin: 01100100) -> SET / DECEL PRESSIONADO
[13:48:44] >>> B0: 64 -> 24 (bin: 00100100) -> SOLTO
[13:45:47] >>> B0: 20 -> E0 (bin: 11100000) -> CANCEL PRESSIONADO (RES + SET)
[13:45:47] >>> B0: E0 -> 20 (bin: 00100000) -> SOLTO
[13:48:49] >>> B0: 24 -> 04 (bin: 00000100) -> CRUISE MAIN DESLIGADO
```

---

## 5. Indicador "CRUISE CONTROL" (Luz Verde de SET no Painel) - COMPROVADO! 🏆

A luz indicadora **`CRUISE CONTROL`** (que avisa que o piloto automático está engatado e mantendo a velocidade) é controlada exclusivamente por mensagem digital na rede **F-CAN**:

* **CAN ID:** **`0x17C`** (`POWERTRAIN_DATA`)
* **Byte Responsável:** **Byte 4**
* **Bit Exato:** **Bit 3 (`0x08` / `0000 1000`)**
* **Frequência Original:** 100 Hz (a cada 10 ms)

### Tabela de Controle do Byte 4 do `0x17C`:

| Estado da Lâmpada no Painel | Byte 4 Hex | Byte 4 Binário | Máscara de Bits |
| :--- | :---: | :---: | :--- |
| **CRUISE CONTROL Apagado** | `0x00` | `0000 0000` | Repouso normal da ECU |
| **CRUISE CONTROL ACESO** | `0x08` | `0000 1000` | **Bit 3 (`0x08`)** |

> [!IMPORTANT]
> **Por que pisca rápido quando injetado via ELM327?**
> A ECU original do veículo transmite o `0x17C` a 100 Hz com `0x00`. Como o adaptador ELM327 via serial (38400 baud) só consegue enviar a ~60 Hz, o painel recebe alternadamente os pacotes da ECU (`0x00`) e do injetor (`0x08`), fazendo o LED piscar na frequência da intercalação. No ESP32 via hardware nativo (TWAI / MCP2515), ou via interceptor/gateway CAN, o frame é mantido contínuo e a lâmpada fica 100% acesa sólida!

---

## 6. Algoritmo de Checksum e Rolling Counter da Honda F-CAN (Byte 7)

O painel de instrumentos do Honda Fit valida rigorosamente a integridade de segurança de todos os pacotes através do **Byte 7**:

* **Rolling Counter (Bits 4-7 / Upper Nibble):** Varia ciclicamente de `0` a `3` (`0 -> 1 -> 2 -> 3 -> 0...`).
* **Checksum (Bits 0-3 / Lower Nibble):** Soma invertida de todos os nibbles do frame com constante $K = 8$:

$$\text{Checksum} = \left(8 - \sum \text{nibbles}\right) \pmod{16}$$

### Implementação em C / C++ (ESP32):

```cpp
void sendHondaCruiseStatus(bool cruiseEngaged) {
    static uint8_t counter = 0;
    
    twai_message_t msg;
    msg.identifier = 0x17C;
    msg.data_length_code = 8;
    msg.flags = TWAI_MSG_FLAG_NONE;
    
    // Bytes 0 a 6 (Trem de Força)
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x02;
    msg.data[3] = 0xBD;
    msg.data[4] = cruiseEngaged ? 0x08 : 0x00; // Bit 3 comanda a luz CRUISE CONTROL
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    
    // Cálculo do Checksum Oficial Honda (Constante 8)
    uint32_t s = 0;
    uint32_t addr = msg.identifier;
    while (addr > 0) {
        s += (addr & 0x0F);
        addr >>= 4;
    }
    for (int i = 0; i < 7; i++) {
        s += (msg.data[i] & 0x0F) + (msg.data[i] >> 4);
    }
    s += counter;
    
    uint8_t checksum = (8 - s) & 0x0F;
    msg.data[7] = (counter << 4) | checksum;
    
    twai_transmit(&msg, 0);
    counter = (counter + 1) & 0x03; // Incrementa counter 0..3
}
```

---

## 7. Status do Pedal de Freio via CAN Bus (ID `0x17C` Byte 4 Bit 0) - COMPROVADO! 🛑

O sensor/interruptor do pedal de freio do carro já é lido nativamente pela ECU e transmitido a cada 10 ms no mesmo frame de trem de força:

* **CAN ID:** **`0x17C`** (`POWERTRAIN_DATA`)
* **Byte Responsável:** **Byte 4**
* **Bit Exato:** **Bit 0 (`0x01` / `0000 0001`)**
* **Taxa de Atualização:** 100 Hz (10 ms de latência máxima)

### Tabela Verdade do Freio (`0x17C` Byte 4):

| Estado do Pedal de Freio | Byte 4 Hex | Byte 4 Binário | Máscara Bit 0 (`& 0x01`) |
| :--- | :---: | :---: | :---: |
| **Freio Solto (Pedal Livre)** | `0x10` / `0x00` | `0001 0000` | `0` (Falso / Desativado) |
| **Freio Pressionado (Pé no Pedal)** | `0x11` / `0x01` | `0001 0001` | `1` (Verdadeiro / **DESENGATAR CRUISE**) |

### Log Real de Captura no Veículo:
```text
[14:28:12] Estado Inicial: 00 00 00 00 10 00 00 21 (Freio Solto)
[14:28:12] 🚨 FREIO DETECTADO: B4: 10 -> 11 (PÉ NO FREIO!)
[14:28:13] 🚨 FREIO SOLTO:      B4: 11 -> 10 (SOLTOU)
[14:28:15] 🚨 FREIO DETECTADO: B4: 10 -> 11 (PÉ NO FREIO!)
[14:28:16] 🚨 FREIO SOLTO:      B4: 11 -> 10 (SOLTOU)
```

### Leitura Instantânea em C/C++ no ESP32:

```cpp
// Processamento do frame 0x17C no loop CAN:
if (message.identifier == 0x17C && message.data_length_code >= 5) {
    uint8_t b4 = message.data[4];
    bool brakePressed = (b4 & 0x01) != 0; // Bit 0
    
    if (brakePressed && cruiseState == CRUISE_ACTIVE) {
        disengageCruise("Brake pedal pressed via CAN");
    }
}
```

---

## 8. Arquitetura 100% CAN Bus do Piloto Automático

Graças a essas descobertas, a instalação do ESP32 no Honda Fit GE ficou **extremamente limpa**:

1. **Velocidade do Carro (VSS):** Lida via CAN ID **`0x158`** (Bytes 4 e 5).
2. **Botões do Volante (SET, RES, CANCEL, MAIN):** Lidos via CAN ID **`0x164`** (Byte 0).
3. **Pedal de Freio:** Lido via CAN ID **`0x17C`** (Byte 4 Bit 0).
4. **Luz CRUISE CONTROL (SET) no Painel:** Comandada via CAN ID **`0x17C`** (Byte 4 Bit 3).
5. **Zero fios extras:** Apenas os 2 fios do barramento **F-CAN** (`CAN_H` e `CAN_L`) ligam o ESP32 a todos os controles e mostradores do carro!

---

## 9. Controle do Acelerador (APP) e Ultrapassagem (Modo A)

### Arquitetura de Interceptação (Failsafe)
* **Relé de Sinal DPDT:**
  * **Desenergizado (NF):** Conecta o pedal físico original diretamente à ECU. Garante 100% de segurança mecânica se o ESP32 for desligado.
  * **Energizado (NA):** Conecta as saídas do DAC **MCP4728** à ECU quando o Cruise está ativo.
* **Correlação dos Sensores Honda DBW (Proporção Obrigatória 2:1):**
  * $\text{APP1} = 0.85\text{V (repouso)} \to 4.50\text{V (WOT)}$
  * $\text{APP2} = 0.425\text{V (repouso)} \to 2.25\text{V (WOT)}$
  * **Regra:** $\text{APP2} = \frac{\text{APP1}}{2}$ com tolerância de $\pm 0.2\text{V}$ para evitar o erro **P2138** (Limp Mode).

### Dinâmica da Ultrapassagem (Modo A - Pass-Through Eletrônico Contínuo)
1. **Monitoramento do Pé do Motorista:** O sinal do pedal físico original (APP1) é lido continuamente a 100 Hz pelo pino **GPIO 36 (ADC)** através de um divisor de tensão 10k/10k, **antes dos contatos do relé**.
2. **Prioridade Máxima ao Motorista:**
   $$\text{Aceleração DAC} = \max(\text{PID}_{\text{Cruise}}, \text{Pedal}_{\text{Motorista}})$$
3. **Sem Bater Relé:** Durante toda a ultrapassagem, o relé **permanece energizado**. O DAC MCP4728 simplesmente acompanha a pressão do pé do motorista em tempo real, permitindo aceleração até 100% (WOT).
4. **Retomada Suave (Anti-Windup):** Durante o excesso de velocidade da ultrapassagem, o termo integral do PID é congelado para evitar efeito chicote. Ao soltar o pé, o carro desacelera suavemente até a velocidade memorizada e o PID reassume o controle.
5. **Travamento de Nova Velocidade:** Se o motorista der um toque na tecla **SET** durante a ultrapassagem, a nova velocidade atingida passa a ser a nova velocidade de cruzeiro memorizada.
6. **Desarme Imediato:** Qualquer toque no **pedal de freio** (detectado via CAN `0x17C` Bit 0) ou toque na tecla **CANCEL** desarma o relé instantaneamente (< 5ms).


