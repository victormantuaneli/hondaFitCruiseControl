# ESP32 Honda Cruise Control - Wiring & Architecture Guide

This document describes the complete electrical, software, and safety architecture for building a Cruise Control system using an **ESP32** and an **MCP4728 12-bit DAC** interfaced to a Honda Drive-By-Wire (DBW) system.

---

## 1. System Overview & Safety Architecture

```
+--------------------------------------------------------------------------+
|                                 VEHICLE                                  |
|                                                                          |
|   +-----------------------+                    +--------------------+    |
|   | Steering Wheel Switch |                    |   Honda F-CAN      |    |
|   | (SET-, RES+, CANCEL)  |                    | (CAN H / CAN L)    |    |
|   +-----------+-----------+                    +---------+----------+    |
|               | (3 Wires: Ground, SET, RES)              |               |
|               v                                          v               |
|         [Discrete GPIOs]                         [CAN Transceiver]       |
|         (Built-in Pullups)                       (SN65HVD230 / VP230)    |
|               |                                          |               |
|               +------------------+  +--------------------+               |
|                                  |  |                                    |
|                                  v  v                                    |
|                             +-------------+                              |
|                             |    ESP32    |                              |
|                             |  Controller |                              |
|                             +------+------+                              |
|                                    |                                     |
|               +--------------------+---------------------+               |
|               | (I2C)                                    | (Relay Control|
|               v                                          v   Active HIGH)|
|          +---------+                               +-------------+       |
|          | MCP4728 |                               | 4-Pin Brake |       |
|          |  12-Bit |                               | Switch Inter|       |
|          |   DAC   |                               +------+------+       |
|          +----+----+                                      |              |
|               | (APP1 & APP2                              v              |
|               |  Spoofed Voltages)                 +-------------+       |
|               +───────────────────┐                | DPDT Signal |       |
|                                   │  NO Contacts   |    Relay    |       |
|                                   └───────────────>|  (Failsafe) |       |
|  +--------------------+                            |             |       |
|  | Physical Gas Pedal |──────── NC Contacts ──────>|             |──────> ECM/PCM
|  |   (APP1 & APP2)    |  (Default De-energized)    +-------------+        (Pedal In)
|  +--------------------+                                                  |
+--------------------------------------------------------------------------+
```

---

## 2. Steering Wheel Switch Wiring & CAN Bus Decoding (Gauge Cluster Integration)

No Honda Fit GE (2ª geração), as teclas do piloto automático do volante são ligadas diretamente no conector do **Painel de Instrumentos (Gauge Control Module - GCM)**:

### Chicote Físico no Painel (GCM)
1. **Comum (GND)**: Terra dos botões do volante (Pino 1 do conector do volante / Carretel da buzina).
2. **Pino 22 do Painel (CRUISE MAIN)**: Sinal do botão principal do piloto automático (latching / interruptor de trava).
3. **Pino 19 do Painel (RES / ACCEL)**: Sinal de retomada / aceleração.
4. **Pino 20 do Painel (SET / DECEL)**: Sinal de ajuste / desaceleração.
5. *(Botão CANCEL no volante aterra simultaneamente os pinos 19 e 20 através dos diodos internos do volante).*

---

### Decodificação via CAN Bus (F-CAN ID `0x164` Byte 0)

O Painel de Instrumentos faz a conversão de hardware dos pinos 19, 20 e 22 e transmite o estado de todos os botões diretamente na rede F-CAN através do **ID `0x164` Byte 0**. 

**Vantagem Crítica:** O ESP32 **não precisa de fios adicionais para os botões**. Ele lê todas as teclas através do transceptor CAN (SN65HVD230 / VP230) já conectado aos pinos CAN H e CAN L!

#### Tabela Verdade Oficial Comprovada na CAN (`0x164` Byte 0):

| Estado / Tecla Pressionada | Hex B0 | Binário B0 | Máscara de Bits | Ação no Sistema |
| :--- | :---: | :---: | :---: | :--- |
| **Cruise Desligado (Idle)** | `0x04` | `0000 0100` | `Bit 2` | Sistema desarmado / desligado |
| **CRUISE MAIN Ligado** | `0x24` | `0010 0100` | `Bit 5 (0x20)` | **Sistema Armado (STANDBY)** |
| **RES / ACCEL** | `0xA4` | `1010 0100` | `Bit 7 (0x80) + Bit 5` | **Retomar velocidade / Incrementar** |
| **SET / DECEL** | `0x64` | `0110 0100` | `Bit 6 (0x40) + Bit 5` | **Definir velocidade atual / Decrementar** |
| **CANCEL** | `0xE4` | `1110 0100` | `Bit 7 + Bit 6 (0xC0) + Bit 5` | **Desengatar piloto automático** |

> [!NOTE]
> O painel só processa e despacha os comandos de `RES`, `SET` e `CANCEL` na CAN quando o **CRUISE MAIN está ligado** (Bit 5 ativo, LED verde aceso no painel). Se o Main estiver desligado, o painel mantém o estado base `0x04`.

---

## 3. Other Peripherals & Connections

### A. Honda 4-Pin Brake Switch (Critical Hardware Interlock)
* **NC Contact (Pins 1 & 2)**:
  * Connect between **GND** and **ESP32 GPIO 18** (internal pull-up).
  * Released = LOW; Depressed = HIGH (switch opens).
* **NO Contact (Pins 3 & 4)**:
  * Connect between **GND** and **ESP32 GPIO 19** (internal pull-up).
  * Released = HIGH; Depressed = LOW (switch closes).
* **Hardware Relay Interlock:**
  * Route the ground return or supply of the DPDT relay coil through this brake switch so pressing the brake physically de-energizes the relay within **5ms**, completely independent of code execution.

### B. CAN Bus Interface (Honda F-CAN @ 500 kbps)
Using an **SN65HVD230** or **VP230** 3.3V CAN transceiver:
* `CTX` (TXD) $\to$ ESP32 **GPIO 5**
* `CRX` (RXD) $\to$ ESP32 **GPIO 4**
* `CANH` $\to$ Vehicle **F-CAN H** (White wire on ECM A3)
* `CANL` $\to$ Vehicle **F-CAN L** (Red wire on ECM A4)

### C. MCP4728 12-Bit DAC & DPDT Safety Relay
* `SDA` $\to$ ESP32 **GPIO 21**
* `SCL` $\to$ ESP32 **GPIO 22**
* `VOUTA` (APP1 output) $\to$ DPDT Relay NO1
* `VOUTB` (APP2 output) $\to$ DPDT Relay NO2
* `COM1` $\to$ ECM APP Sensor 1 input
* `COM2` $\to$ ECM APP Sensor 2 input
* `NC1` $\to$ Physical gas pedal APP Sensor 1
* `NC2` $\to$ Physical gas pedal APP Sensor 2
* `Relay Coil Drive` $\to$ NPN transistor / MOSFET driven by ESP32 **GPIO 25**.
