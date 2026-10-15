#include "can_speed.h"
#include "driver/twai.h"

// Official Honda F-CAN Checksum calculation (Byte 7 lower nibble)
static uint8_t calculateHondaChecksum(uint32_t id, const uint8_t* data, uint8_t counter) {
    int32_t s = 0;
    uint32_t addr = id;
    while (addr > 0) {
        s += (addr & 0x0F);
        addr >>= 4;
    }
    for (int i = 0; i < 7; i++) {
        s += (data[i] & 0x0F) + (data[i] >> 4);
    }
    s += counter;
    return (uint8_t)((8 - s) & 0x0F);
}

CanSpeedReader::CanSpeedReader() 
    : _speedKmh(BENCH_SIM_MODE ? 70.0f : 0.0f),
      _engineRpm(BENCH_SIM_MODE ? 2200.0f : 0.0f),
      _lastRxTimeMs(BENCH_SIM_MODE ? millis() : 0),
      _initialized(false),
      _benchSimActive(BENCH_SIM_MODE),
      _realCanReceived(false),
      _brakePressedCan(false),
      _canMainSw(false),
      _canButton(SteeringButton::NONE),
      _cruiseLampOn(false),
      _manualLampTest(false),
      _has17c(false),
      _snifferMode(SnifferMode::OFF),
      _lastB0_164(0xFF),
      _lastB4_17C(0xFF) {
    memset(_latest17c, 0, sizeof(_latest17c));
}


bool CanSpeedReader::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX, 
        (gpio_num_t)PIN_CAN_RX, 
        TWAI_MODE_NORMAL
    );
    // Setting tx_queue_len = 0 uses direct hardware transmission, bypassing
    // the ESP-IDF twai_handle_tx_buffer_frame software queue race condition bug!
    g_config.tx_queue_len = 0;
    g_config.rx_queue_len = 64;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_QUEUE_FULL;
    
    // Honda F-CAN standard is 500kbit/s
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println(F("[CAN] Warning: TWAI driver not installed (Normal on bench/Wokwi)"));
        return false;
    }

    if (twai_start() != ESP_OK) {
        Serial.println(F("[CAN] Warning: TWAI driver not started (Normal on bench/Wokwi)"));
        return false;
    }

    Serial.println(F("[CAN] TWAI (CAN) initialized at 500 kbps"));
    _initialized = true;
    return true;
}

void CanSpeedReader::update() {
    if (_initialized) {
        // Safe, alert-based Bus-Off recovery (prevents twai_handle_tx_buffer_frame assert crashes)
        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                twai_initiate_recovery();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                twai_start();
            }
        }

        twai_message_t message;
        // Process pending packets in RX queue
        while (twai_receive(&message, 0) == ESP_OK) {
            if (_snifferMode == SnifferMode::ALL_RAW) {
                Serial.printf("\n[CAN ALL] ID: 0x%03X  DLC: %u  DATA:", message.identifier, message.data_length_code);
                for (uint8_t i = 0; i < message.data_length_code; i++) {
                    Serial.printf(" %02X", message.data[i]);
                }
                Serial.println();
            } else if (_snifferMode == SnifferMode::FILTERED) {
                // Filtered mode: only print relevant packets when they change
                if (message.identifier == 0x164 && message.data_length_code >= 1) {
                    uint8_t b0 = message.data[0];
                    if (b0 != _lastB0_164) {
                        _lastB0_164 = b0;
                        bool mainSw = (b0 & 0x20) != 0;
                        bool setBtn = (b0 & 0x40) != 0;
                        bool resBtn = (b0 & 0x80) != 0;
                        const char* btnName = "NENHUM";
                        if (setBtn && resBtn) btnName = "CANCEL";
                        else if (setBtn) btnName = "SET/DECEL";
                        else if (resBtn) btnName = "RES/ACCEL";

                        Serial.printf("\n>>> [CAN 0x164 VOLANTE] B0: 0x%02X | MAIN: %s | BOTAO: %s <<<\n",
                                      b0, mainSw ? "LIGADO" : "DESLIGADO", btnName);
                    }
                } else if (message.identifier == 0x17C && message.data_length_code >= 5) {
                    uint8_t b4 = message.data[4];
                    if (b4 != _lastB4_17C) {
                        _lastB4_17C = b4;
                        bool brakeOn = (b4 & 0x01) != 0;
                        bool lightOn = (b4 & 0x08) != 0;
                        Serial.printf("\n>>> [CAN 0x17C] B4: 0x%02X | FREIO: %s | LUZ CRUISE: %s <<<\n",
                                      b4, brakeOn ? "PÉ NO FREIO (ON)" : "SOLTO (OFF)", lightOn ? "ACESO" : "APAGADO");
                    }
                }
            }

            if (!(message.flags & TWAI_MSG_FLAG_RTR)) {
                parseHondaFrame(message.identifier, message.data, message.data_length_code);
            }
        }
    }

    // On bench or simulator: keep simulated speed timestamp fresh unless explicit timeout test is active
    if (_benchSimActive && !_realCanReceived) {
        if (_lastRxTimeMs != 0) {
            _lastRxTimeMs = millis();
        }
    }
}

void CanSpeedReader::parseHondaFrame(uint32_t identifier, const uint8_t* data, uint8_t length) {
    _realCanReceived = true;
    _benchSimActive = false;

    // Standard Honda 0x158: Engine RPM & Vehicle Speed
    if (identifier == CAN_HONDA_SPEED_ID && length >= 6) {
        // Engine RPM: (Byte 0 << 8) | Byte 1
        uint16_t rawRpm = (data[0] << 8) | data[1];
        if (rawRpm != 0xFFFF) {
            _engineRpm = (float)rawRpm;
        }

        // Vehicle Speed: (Byte 4 << 8) | Byte 5, scaled by 100 (km/h)
        uint16_t rawSpeed = (data[4] << 8) | data[5];
        // 0xFFFF is Honda SNA (Signal Not Available / Sensor Uncalibrated) -> causes false 655.35 km/h spike!
        if (rawSpeed != 0xFFFF && rawSpeed <= 25000) { // Ignore SNA and speeds above 250 km/h
            _speedKmh = (float)rawSpeed / 100.0f;
            _lastRxTimeMs = millis();
        }
    }
    // Honda 0x17C: Powertrain status, Brake pedal bit (Byte 4, Bit 0) - Broadcast at 100 Hz
    else if (identifier == 0x17C && length >= 5) {
        _brakePressedCan = (data[4] & 0x01) != 0;
        _lastRxTimeMs = millis(); // Valid powertrain CAN packet received

        // Cluster CRUISE CONTROL indicator injection (Byte 4, Bit 3 = 0x08)
        // Whenever cruise is ACTIVE/OVERRIDE (or manual lamp test is ON), inject updated 0x17C frame
#if CAN_LAMP_INJECTION_ENABLED
        if ((_cruiseLampOn || _manualLampTest) && length >= 8) {
            // Burst mode: Send continuous frames during the ON window (e.g. 250ms), then silence (750ms)
            uint32_t cycleMs = millis() % CAN_LAMP_BURST_PERIOD_MS;
            if (cycleMs < CAN_LAMP_BURST_ON_MS) {
                twai_status_info_t st;
                if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_RUNNING) {
                    twai_message_t tx_msg;
                    tx_msg.identifier = 0x17C;
                    tx_msg.extd = 0;
                    tx_msg.rtr = 0;
                    tx_msg.data_length_code = 8;
                    for (uint8_t i = 0; i < 7; i++) {
                        tx_msg.data[i] = data[i];
                    }
                    tx_msg.data[4] |= 0x08; // Set Bit 3: illuminate green CRUISE CONTROL light on cluster

                    // Honda Byte 7: Upper nibble = rolling counter (0..3), Lower nibble = checksum
                    uint8_t counter = (data[7] >> 4) & 0x03;
                    uint8_t chk = calculateHondaChecksum(0x17C, tx_msg.data, counter);
                    tx_msg.data[7] = (counter << 4) | chk;

                    twai_transmit(&tx_msg, 0); // Non-blocking transmit
                }
            }
        }
#endif
    }
    // Honda 0x164: Gauge Cluster Cruise Switches (Byte 0: Bit 5=MAIN, Bit 6=SET, Bit 7=RES)
    else if (identifier == 0x164 && length >= 1) {
        uint8_t b0 = data[0];
        _canMainSw = (b0 & 0x20) != 0; // Bit 5

        bool setBtn = (b0 & 0x40) != 0; // Bit 6
        bool resBtn = (b0 & 0x80) != 0; // Bit 7

        if (setBtn && resBtn) {
            _canButton = SteeringButton::CANCEL;
        } else if (setBtn) {
            _canButton = SteeringButton::SET_DECEL;
        } else if (resBtn) {
            _canButton = SteeringButton::RES_ACCEL;
        } else {
            _canButton = SteeringButton::NONE;
        }
        _lastRxTimeMs = millis();
    }
    // Standard OBD-II PID 0x0D (Vehicle Speed response)
    else if (identifier == 0x7E8 && length >= 4) {
        if (data[1] == 0x41 && data[2] == 0x0D) {
            _speedKmh = (float)data[3]; // Direct km/h
            _lastRxTimeMs = millis();
        }
    }
}

float CanSpeedReader::getSpeedKmh() const {
    return _speedKmh;
}

float CanSpeedReader::getEngineRpm() const {
    return _engineRpm;
}

bool CanSpeedReader::isSpeedValid() const {
    if (_lastRxTimeMs == 0) return false;
    return (millis() - _lastRxTimeMs) <= CAN_SPEED_TIMEOUT_MS;
}

bool CanSpeedReader::isBrakePressed() const {
    return _brakePressedCan;
}

bool CanSpeedReader::isCruiseMainActive() const {
    return _canMainSw;
}

SteeringButton CanSpeedReader::getPressedButton() const {
    return _canButton;
}

uint32_t CanSpeedReader::getLastReceivedMs() const {
    return _lastRxTimeMs;
}

void CanSpeedReader::setSimulatedSpeed(float speedKmh, bool valid) {
    _speedKmh = speedKmh;
    if (valid) {
        _lastRxTimeMs = millis();
    } else {
        _lastRxTimeMs = 0;
    }
}

void CanSpeedReader::toggleSnifferOutput() {
    if (_snifferMode == SnifferMode::OFF) {
        _snifferMode = SnifferMode::FILTERED;
        Serial.println(F("\n[CAN] Sniffer: FILTRADO ATIVADO (Apenas 0x164 botões, 0x17C painel quando mudarem)"));
    } else if (_snifferMode == SnifferMode::FILTERED) {
        _snifferMode = SnifferMode::ALL_RAW;
        Serial.println(F("\n[CAN] Sniffer: MODO BRUTO ATIVADO (Todos os pacotes da rede)"));
    } else {
        _snifferMode = SnifferMode::OFF;
        Serial.println(F("\n[CAN] Sniffer: DESATIVADO"));
    }
}

CanSpeedReader::SnifferMode CanSpeedReader::getSnifferMode() const {
    return _snifferMode;
}

void CanSpeedReader::setCruiseLamp(bool on) {
    _cruiseLampOn = on;
}

bool CanSpeedReader::isCruiseLampOn() const {
    return _cruiseLampOn || _manualLampTest;
}

void CanSpeedReader::toggleManualLampTest() {
    _manualLampTest = !_manualLampTest;
    Serial.printf("\n[CAN 0x17C] Teste Manual da Luz CRUISE: %s\n", _manualLampTest ? "LIGADA (FORÇADA)" : "DESLIGADA (MODO NORMAL)");
}

void CanSpeedReader::toggleVirtualMainSwitch() {
    _canMainSw = !_canMainSw;
    Serial.printf("\n[CAN 0x164] Virtual Cruise Main: %s\n", _canMainSw ? "ON" : "OFF");
}



