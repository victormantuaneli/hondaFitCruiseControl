# ESP32 Honda Cruise Control (Drive-By-Wire)

Sistema de Piloto Automático (Cruise Control) de alto desempenho desenvolvido para veículos Honda com acelerador eletrônico (Drive-By-Wire - DBW), com foco no **Honda Fit (2ª Geração - GE6 / GE8, 2009–2014)** e modelos compatíveis (Civic, City).

O projeto utiliza um microcontrolador **ESP32**, um **DAC MCP4728 de 12 bits** (I2C), um transceptor **CAN Bus (SN65HVD230)** integrado à rede **Honda F-CAN (500 kbps)** e um **relé de segurança DPDT**.

---

## 🚀 Principais Funcionalidades

* **Integração Nativa com a Rede F-CAN (500 kbps):**
  * Leitura em tempo real de **Velocidade do Veículo** e **RPM do Motor** via ID `0x158`.
  * Leitura digital dos botões do volante (**CRUISE MAIN**, **SET / DECEL**, **RES / ACCEL**, **CANCEL**) transmitidos pelo Painel de Instrumentos via ID `0x164`. Não requer passagem de fios adicionais para o volante!
  * Leitura instantânea do status do **Pedal de Freio** via ID `0x17C`.
  * Injeção do indicador luminoso verde **`CRUISE CONTROL`** no painel de instrumentos via ID `0x17C`.
* **Emulação Precisa do Pedal de Acelerador (DBW):**
  * Geração das curvas analógicas **APP1** (0.85V – 4.50V) e **APP2** (0.42V – 2.25V) com razão estrita de 2:1 através do DAC I2C MCP4728 de 12 bits, evitando qualquer erro de injeção (DTC P2138 / Limp Mode).
* **Transição Suave sem Tranco ao Engatar (Zero Glitch Relay):**
  * No momento em que a tecla `SET` é pressionada, o ESP32 lê a posição exata do pé do motorista, pré-carrega o DAC com a mesma tensão atual e só então atraca o relé DPDT. A transição é 100% invisível para a ECU.
* **Driver Override Progressivo com Zero Zona Morta:**
  * Auto-calibração no boot da tensão de repouso do pedal físico.
  * Resposta progressiva instantânea ao encostar o pé no pedal físico durante o cruise para ultrapassagens rápidas, retornando suavemente ao controle PID quando o pedal é solto.
* **Arquitetura de Segurança Failsafe (Hardware Interlock):**
  * Relé DPDT normalmente desenergizado: em caso de falha, corte de energia ou travamento, o pedal físico original fica conectado diretamente à ECU do veículo.
  * Desarme imediato por: pedal de freio (CAN ou discreto), botão CANCEL, desarme da chave MAIN, velocidade fora da faixa (mínima de 25 km/h, máxima de 140 km/h) ou timeout de comunicação CAN (1000ms).

---

## 🛠️ Arquitetura de Hardware e Pinagem

```text
+--------------------------------------------------------------------------+
|                                 VEÍCULO                                  |
|                                                                          |
|   +-----------------------+                    +--------------------+    |
|   | Botões no Volante     |                    |   Honda F-CAN      |    |
|   | (SET-, RES+, CANCEL)  |                    | (CAN H / CAN L)    |    |
|   +-----------+-----------+                    +---------+----------+    |
|               | (3 Fios -> Painel)                       |               |
|               v                                          v               |
|       [Painel de Instrumentos]                 [Transceptor CAN]         |
|         (Broadcast F-CAN)                     (SN65HVD230 / VP230)       |
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
|               | (I2C)                                    | (Relé DPDT    |
|               v                                          v   GPIO 25)    |
|          +---------+                               +-------------+       |
|          | MCP4728 |                               |  Relé DPDT  |       |
|          |  12-Bit |                               |  de Sinal   |       |
|          |   DAC   |                               +------+------+       |
|          +----+----+                                      |              |
|               | (APP1 & APP2                              v              |
|               |  Sinais Simulados)                 +-------------+       |
|               +───────────────────┐                | Contatos NO |       |
|                                   │                |             |       |
|                                   └───────────────>|  (Failsafe) |       |
|  +--------------------+                            |             |       |
|  | Pedal Físico (Carro|──────── Contatos NC ──────>|             |──────> ECM / Injeção
|  |   (APP1 & APP2)    |  (Desenergizado = Original)|             |        (Entrada Pedal)
|  +--------------------+                            +-------------+
+--------------------------------------------------------------------------+
```

### Tabela de Conexões do ESP32

| Periférico | Função | Pino ESP32 | Observações |
| :--- | :--- | :---: | :--- |
| **Transceptor CAN** | `CTX` (TXD) | **GPIO 5** | Driver TWAI nativo do ESP32 |
| **Transceptor CAN** | `CRX` (RXD) | **GPIO 4** | Driver TWAI nativo do ESP32 |
| **CAN Bus H/L** | F-CAN H / L | — | Conectado ao pino ECM A3 (Branco) e A4 (Vermelho) |
| **DAC MCP4728** | I2C `SDA` | **GPIO 21** | Barramento I2C (100 kHz) |
| **DAC MCP4728** | I2C `SCL` | **GPIO 22** | Barramento I2C (100 kHz) |
| **DAC MCP4728** | `VOUTA` (Canal A) | — | Sinal simulado **APP2** $\to$ Relé NO2 |
| **DAC MCP4728** | `VOUTB` (Canal B) | — | Sinal simulado **APP1** $\to$ Relé NO1 |
| **Relé DPDT** | Acionamento | **GPIO 25** | Saída para transistor NPN/MOSFET (HIGH = Atracado) |
| **Leitura Pedal** | Leitura Analógica | **GPIO 36 (VP)**| Monitora o pedal físico antes do relé para Override |
| **LED Status** | Indicador Cruise Ativo | **GPIO 26** | LED piloto automático controlando |
| **LED Status** | Indicador Cruise Main | **GPIO 27** | LED sistema armado (Main ON) |

---

## 📡 Engenharia Reversa da Rede F-CAN (Honda)

### 1. Botões do Volante no Painel (`ID 0x164` - 50 Hz)
O painel de instrumentos processa os botões do volante e despacha no Byte 0 do ID `0x164`:

| Estado / Tecla | Byte 0 (Hex) | Binário | Ação no Sistema |
| :--- | :---: | :---: | :--- |
| **Cruise Main Desligado** | `0x04` | `0000 0100` | Sistema desarmado (`OFF`) |
| **CRUISE MAIN Ligado** | `0x24` | `0010 0100` | Sistema armado (`STANDBY`) |
| **SET / DECEL** | `0x64` | `0110 0100` | Engata ou decrementa velocidade (-1 km/h) |
| **RES / ACCEL** | `0xA4` | `1010 0100` | Retoma ou incrementa velocidade (+1 km/h) |
| **CANCEL** | `0xE4` | `1110 0100` | Desarma o piloto automático |

### 2. Velocidade e RPM (`ID 0x158` - 100 Hz)
* **RPM do Motor:** `(Byte 0 << 8) | Byte 1`
* **Velocidade:** `((Byte 4 << 8) | Byte 5) / 100.0` (km/h)

### 3. Luz Verde "CRUISE CONTROL" no Painel (`ID 0x17C` - 100 Hz)
* **Status do Freio:** `(Byte 4 & 0x01)` (1 = freio pressionado, 0 = solto).
* **Luz de Cruise:** `Byte 4, Bit 3 (0x08)`. Quando ativo, o ESP32 injeta o bit 3 para manter o LED verde de controle aceso no cluster.

---

## 🛡️ Tabela de Motivos de Desarme (Diagnóstico Serial)

Sempre que o piloto automático desarmar, o console serial exibirá em destaque:
```text
>>> [CRUISE DESATIVOU] Motivo: [MOTIVO] (código X) | Vel: Y km/h <<<
```

| Código | Identificador | Condição de Disparo |
| :---: | :--- | :--- |
| **1** | **`BRAKE_PRESSED`** | Sinal digital de freio via CAN (0x17C) ou interruptor físico acionado |
| **3** | **`CANCEL_BUTTON`** | Tecla CANCEL acionada no volante (Bits 6 + 7 no 0x164) |
| **4** | **`MAIN_SWITCH_OFF`** | Chave principal do piloto automático desligada |
| **5** | **`SPEED_TOO_LOW`** | Velocidade caiu para mais de 5 km/h abaixo da mínima (abaixo de 20 km/h) |
| **6** | **`SPEED_TOO_HIGH`**| Velocidade ultrapassou 150 km/h |
| **7** | **`CAN_TIMEOUT`** | Ausência de pacotes de velocidade na CAN por mais de 1000 ms |

---

## 💻 Painel de Telemetria Serial em Tempo Real

Durante o funcionamento, o monitor serial atualiza a cada 200ms uma linha de telemetria estática no console:

```text
State: ACTIVE  | Spd: 70.0/70.0 km/h | Thr:18.5% | Ped: 0.0%(0.22V) | APP: 0.43/0.85V | DAC:OK | CAN:OK  | Brk:OFF | Lamp:ON
```

* **State:** Estado da máquina (`OFF`, `STANDBY`, `ACTIVE`, `OVERRIDE`).
* **Spd:** Velocidade Atual / Velocidade Alvo (km/h).
* **Thr:** Abertura da borboleta calculada pelo PID (%).
* **Ped:** Posição do pedal físico do motorista e tensão analógica lida ao vivo.
* **APP:** Tensões físicas entregues nos canais de saída do DAC (APP2 / APP1).
* **DAC / CAN / Brk / Lamp:** Status de integridade dos periféricos em tempo real.

---

## ⚙️ Como Compilar e Gravar (PlatformIO)

Requisitos: [PlatformIO Core](https://platformio.org/) ou extensão PlatformIO no VS Code.

### Compilação:
```powershell
pio run
```

### Gravação no ESP32:
```powershell
pio run --target upload --upload-port COM8
```

### Monitor Serial Interativo (115200 baud):
```powershell
pio device monitor -p COM8 -b 115200
```

#### Comandos Interativos de Teste no Monitor Serial:
* `m`: Alternar chave Cruise Main virtual
* `s`: Pressionar botão SET / DECEL
* `r`: Pressionar botão RES / ACCEL
* `c`: Pressionar botão CANCEL
* `b`: Simular pulso de pedal de freio (500ms)
* `+` / `-`: Incrementar / decrementar velocidade simulada em bancada
* `i`: Executar diagnóstico I2C e scan de barramento do MCP4728
* `x`: Alternar modo do sniffer CAN (Desligado / Filtrado / Todos os frames Raw)

---

## 📁 Estrutura do Repositório

```text
├── docs/                      # Documentação técnica e logs de engenharia reversa
│   ├── HONDA_CAN_CRUISE_REFERENCE.md  # Mapeamento completo dos IDs CAN da Honda
│   ├── WIRING_AND_ARCHITECTURE.md     # Esquemas de ligação elétrica detalhados
│   └── CAN_SNIFFER_GUIDE.md           # Guia de captura com Raspberry Pi Pico e sniffer
├── include/                   # Cabeçalhos do sistema
│   ├── config.h               # Parâmetros de calibração, pinos e constantes
│   ├── cruise_controller.h    # Máquina de estados e algoritmo de controle PID
│   ├── can_speed.h            # Driver TWAI para leitura e injeção F-CAN
│   ├── mcp4728_dac.h          # Driver I2C para controle dos canais analógicos
│   └── safety_supervisor.h    # Supervisão de segurança e intertravamentos
├── src/                       # Código-fonte C++ (FreeRTOS)
│   ├── main.cpp               # Setup, loop de tarefas e console de depuração
│   ├── cruise_controller.cpp  # Lógica do piloto automático e transição suave
│   ├── can_speed.cpp          # Processamento dos frames CAN Honda
│   ├── mcp4728_dac.cpp        # Geração das tensões duplas APP1 e APP2
│   └── safety_supervisor.cpp  # Auto-calibração do pedal e cortes de emergência
├── tools/                     # Scripts auxiliares em Python para simulação e sniffing
└── platformio.ini             # Configuração de build do PlatformIO para ESP32
```

---

## ⚠️ Isenção de Responsabilidade (Safety Disclaimer)

> [!CAUTION]
> A modificação do sistema de acelerador eletrônico de um veículo automotor envolve riscos intrínsecos de segurança. Este projeto é de caráter educacional e de pesquisa técnica. O uso, montagem e teste em vias públicas é de responsabilidade exclusiva do condutor. Certifique-se sempre de utilizar componentes de qualidade automotiva e manter o intertravamento físico de corte de emergência pelo pedal de freio funcional.
