#include "canbox_emulator.h"

CanboxEmulator::CanboxEmulator()
    : _initialized(false),
      _debugEnabled(false),
      _currentBaudrate(CANBOX_UART_BAUDRATE),
      _simDoorActive(false),
      _doorDriver(false),
      _doorPassenger(false),
      _doorRearLeft(false),
      _doorRearRight(false),
      _trunk(false),
      _hood(false),
      _seatbeltWarning(false),
      _handbrake(false),
      _reverseGear(false),
      _headlights(false),
      _highBeam(false),
      _turnLeft(false),
      _turnRight(false),
      _currentSpeedKmh(0.0f),
      _currentRpm(0.0f),
      _brakePedal(false),
      _cruiseMain(false),
      _cruiseActive(false),
      _targetSpeedKmh(0.0f),
      _lastDoorByte(0xFF),
      _lastStatusByte0(0xFF),
      _lastStatusByte1(0xFF),
      _lastCruiseActive(false),
      _lastSpeedSendMs(0),
      _lastDoorsSendMs(0),
      _lastStatusSendMs(0),
      _lastCruiseSendMs(0),
      _lastVersionSendMs(0) {}

void CanboxEmulator::toggleDebug() {
    _debugEnabled = !_debugEnabled;
    Serial.printf("\n[CANBOX] Monitor Serial2 (TX2/RX2): %s\n", 
                  _debugEnabled ? "ATIVADO (Exibindo pacotes enviados e recebidos)" : "DESATIVADO");
}

void CanboxEmulator::toggleSimulatedDoor() {
    toggleSimDoor(1);
}

void CanboxEmulator::toggleSimDoor(uint8_t doorIndex) {
    switch (doorIndex) {
        case 1:
            _simDoorActive = true;
            _doorDriver = !_doorDriver;
            Serial.printf("\n>>> [CANBOX TEST] Porta Motorista (FL): %s <<<\n", _doorDriver ? "ABERTA" : "FECHADA");
            break;
        case 2:
            _simDoorActive = true;
            _doorPassenger = !_doorPassenger;
            Serial.printf("\n>>> [CANBOX TEST] Porta Passageiro (FR): %s <<<\n", _doorPassenger ? "ABERTA" : "FECHADA");
            break;
        case 3:
            _simDoorActive = true;
            _doorRearLeft = !_doorRearLeft;
            Serial.printf("\n>>> [CANBOX TEST] Porta Traseira Esquerda (RL): %s <<<\n", _doorRearLeft ? "ABERTA" : "FECHADA");
            break;
        case 4:
            _simDoorActive = true;
            _doorRearRight = !_doorRearRight;
            Serial.printf("\n>>> [CANBOX TEST] Porta Traseira Direita (RR): %s <<<\n", _doorRearRight ? "ABERTA" : "FECHADA");
            break;
        case 5:
            _simDoorActive = true;
            _trunk = !_trunk;
            Serial.printf("\n>>> [CANBOX TEST] Porta-Malas: %s <<<\n", _trunk ? "ABERTO" : "FECHADO");
            break;
        case 0:
            _doorDriver = false;
            _doorPassenger = false;
            _doorRearLeft = false;
            _doorRearRight = false;
            _trunk = false;
            _simDoorActive = false;
            Serial.println(F("\n>>> [CANBOX TEST] TODAS AS PORTAS FECHADAS (Reset) <<<"));
            break;
    }
    sendDoorsAndSeatbelt();
}

void CanboxEmulator::toggleBaudrate() {
    if (_currentBaudrate == 38400) {
        _currentBaudrate = 115200;
    } else if (_currentBaudrate == 115200) {
        _currentBaudrate = 19200;
    } else {
        _currentBaudrate = 38400;
    }
    Serial2.updateBaudRate(_currentBaudrate);
    Serial.printf("\n>>> [CANBOX] Baudrate da Serial2 alterado para: %u bps <<<\n", _currentBaudrate);
}

void CanboxEmulator::sendVersion() {
    const char* ver = "RZC-HONDA-V1.0";
    sendRaisePacket(0x30, (const uint8_t*)ver, strlen(ver));
}

void CanboxEmulator::begin() {
#if CANBOX_EMULATOR_ENABLED
    // Initialize Hardware UART2 on ESP32 (38400 baud, 8N1, RX: GPIO 16, TX: GPIO 17)
    Serial2.begin(_currentBaudrate, SERIAL_8N1, PIN_CANBOX_RX, PIN_CANBOX_TX);
    _initialized = true;
    Serial.printf("[CANBOX] Emulador Raise RZC Honda ativo na Serial2 (TX: GPIO %d, RX: GPIO %d, %d bps)\n",
                  PIN_CANBOX_TX, PIN_CANBOX_RX, _currentBaudrate);

    // Initial broadcast to announce presence to the head unit
    sendVersion();
    sendDoorsAndSeatbelt();
    sendVehicleStatus();
    sendSpeedAndRpm();
    sendCruiseStatus();
    _lastVersionSendMs = millis();
#endif
}

void CanboxEmulator::sendRaisePacket(uint8_t cmd, const uint8_t* payload, uint8_t length) {
#if CANBOX_EMULATOR_ENABLED
    if (!_initialized) return;

    uint8_t frame[24];
    if (length > 18) length = 18;

    frame[0] = 0x2E;     // Raise header (SOF)
    frame[1] = cmd;      // Command ID
    frame[2] = length;   // Payload length

    uint16_t sum = cmd + length;
    for (uint8_t i = 0; i < length; i++) {
        frame[3 + i] = payload[i];
        sum += payload[i];
    }

    // Standard Raise checksum: inverted 8-bit sum of (cmd + length + payload)
    frame[3 + length] = (uint8_t)((sum ^ 0xFF) & 0xFF);

    Serial2.write(frame, 4 + length);

    if (_debugEnabled) {
        const char* cmdName = "INFO";
        if (cmd == 0x24 || cmd == 0x41) cmdName = "PORTAS/STATUS";
        else if (cmd == 0x25) cmdName = "STATUS/LUZES";
        else if (cmd == 0x16) cmdName = "VEL/RPM";
        else if (cmd == 0x30) cmdName = "VERSAO";
        else if (cmd == 0x32) cmdName = "CRUISE";

        Serial.printf("[CANBOX TX] CMD 0x%02X (%s) ->", cmd, cmdName);
        for (uint8_t i = 0; i < 4 + length; i++) {
            Serial.printf(" %02X", frame[i]);
        }
        Serial.println();
    }
#endif
}

void CanboxEmulator::sendDoorsAndSeatbelt() {
    // 1. Raise Universal Car Status (CMD 0x41, Subcommand 0x01)
    // Used by Topway / Unisoc / Lelong head units for Door pop-ups
    uint8_t raiseDoor = 0;
    if (_doorDriver)       raiseDoor |= 0x01; // Bit 0: Front-Left (Driver) open
    if (_doorPassenger)    raiseDoor |= 0x02; // Bit 1: Front-Right (Passenger) open
    if (_doorRearLeft)     raiseDoor |= 0x04; // Bit 2: Rear-Left open
    if (_doorRearRight)    raiseDoor |= 0x08; // Bit 3: Rear-Right open
    if (_trunk)            raiseDoor |= 0x10; // Bit 4: Trunk open
    if (_hood)             raiseDoor |= 0x20; // Bit 5: Hood open

    uint8_t payload41[2] = { 0x01, raiseDoor };
    sendRaisePacket(0x41, payload41, 2);

    // 2. Honda Specific Format (CMD 0x24, 1 byte)
    uint8_t doorByte = 0;
    if (_doorDriver)       doorByte |= 0x80; // Bit 7: Driver door
    if (_doorPassenger)    doorByte |= 0x40; // Bit 6: Passenger door
    if (_doorRearLeft)     doorByte |= 0x20; // Bit 5: Rear-Left
    if (_doorRearRight)    doorByte |= 0x10; // Bit 4: Rear-Right
    if (_trunk)            doorByte |= 0x08; // Bit 3: Trunk
    if (_hood)             doorByte |= 0x04; // Bit 2: Hood

    uint8_t payload24[1] = { doorByte };
    sendRaisePacket(0x24, payload24, 1);

    _lastDoorByte = raiseDoor;
    _lastDoorsSendMs = millis();
}

void CanboxEmulator::sendVehicleStatus() {
    // Command 0x25: Handbrake, Reverse, Lights & Turn Signals
    uint8_t b0 = 0;
    if (_handbrake)    b0 |= 0x01; // Bit 0: Handbrake active
    if (_reverseGear)  b0 |= 0x02; // Bit 1: Reverse gear active
    if (_headlights)   b0 |= 0x04; // Bit 2: Headlights / Dimmer active
    if (_highBeam)     b0 |= 0x08; // Bit 3: High beam active
    if (_brakePedal)   b0 |= 0x10; // Bit 4: Foot brake pedal pressed

    uint8_t b1 = 0;
    if (_turnLeft)     b1 |= 0x01; // Bit 0: Left turn signal
    if (_turnRight)    b1 |= 0x02; // Bit 1: Right turn signal

    uint8_t payload[2] = { b0, b1 };
    sendRaisePacket(0x25, payload, 2);
    _lastStatusByte0 = b0;
    _lastStatusByte1 = b1;
    _lastStatusSendMs = millis();
}

void CanboxEmulator::sendSpeedAndRpm() {
    // 1. Raise Universal Car Telemetry (CMD 0x41, Subcommand 0x02 - 13 bytes)
    uint16_t rawRpm = (uint16_t)(_currentRpm >= 0.0f ? _currentRpm : 0.0f);
    uint16_t carSpeedX100 = (uint16_t)((_currentSpeedKmh >= 0.0f ? _currentSpeedKmh : 0.0f) * 100.0f);
    uint16_t voltageX100 = 1420; // 14.20V battery
    uint16_t tempX10 = 250;      // 25.0 C

    uint8_t payloadCar[13] = {
        0x02,
        (uint8_t)((rawRpm >> 8) & 0xFF),
        (uint8_t)(rawRpm & 0xFF),
        (uint8_t)((carSpeedX100 >> 8) & 0xFF),
        (uint8_t)(carSpeedX100 & 0xFF),
        (uint8_t)((voltageX100 >> 8) & 0xFF),
        (uint8_t)(voltageX100 & 0xFF),
        (uint8_t)((tempX10 >> 8) & 0xFF),
        (uint8_t)(tempX10 & 0xFF),
        0x00, 0x00, 0x00, // mileage
        0x50              // fuel %
    };
    sendRaisePacket(0x41, payloadCar, 13);

    // 2. Legacy Command 0x16 format (Speed and RPM)
    uint16_t spd = (uint16_t)(_currentSpeedKmh >= 0.0f ? _currentSpeedKmh : 0.0f);
    uint8_t payload16[4] = {
        (uint8_t)(spd & 0xFF),
        (uint8_t)((spd >> 8) & 0xFF),
        (uint8_t)((rawRpm >> 8) & 0xFF),
        (uint8_t)(rawRpm & 0xFF)
    };
    sendRaisePacket(0x16, payload16, 4);

    _lastSpeedSendMs = millis();
}

void CanboxEmulator::sendCruiseStatus() {
    // 1. Send Version Code (CMD 0x30) to keep head unit connection active
    const char* verStr = "RZC-HONDA-V1.0";
    sendRaisePacket(0x30, (const uint8_t*)verStr, strlen(verStr));

    // 2. Command 0x32: Cruise Control & Vehicle Information Status
    uint8_t cruiseStateCode = 0;
    if (_cruiseActive) {
        cruiseStateCode = 2; // ACTIVE
    } else if (_cruiseMain) {
        cruiseStateCode = 1; // STANDBY / ARMED
    }

    uint8_t targetSpd = (uint8_t)(_targetSpeedKmh >= 0.0f ? _targetSpeedKmh : 0.0f);
    uint8_t curSpd = (uint8_t)(_currentSpeedKmh >= 0.0f ? _currentSpeedKmh : 0.0f);

    uint8_t payload[4];
    payload[0] = cruiseStateCode; // Cruise status
    payload[1] = targetSpd;       // Cruise setpoint speed (km/h)
    payload[2] = curSpd;          // Current speed
    payload[3] = _brakePedal ? 1 : 0;

    sendRaisePacket(0x32, payload, 4);
    _lastCruiseActive = _cruiseActive;
    _lastCruiseSendMs = millis();
}

void CanboxEmulator::parseHondaFrame(uint32_t identifier, const uint8_t* data, uint8_t length) {
    if (!data || length < 1) return;

    // Honda 0x294: Body, Cluster Warnings, Seatbelt, Handbrake, Turn Signals
    if (identifier == 0x294 && length >= 3) {
        if (_debugEnabled) {
            static uint8_t lastD[3] = {0xFF, 0xFF, 0xFF};
            if (memcmp(lastD, data, 3) != 0) {
                memcpy(lastD, data, 3);
                Serial.printf("[CAN 0x294 RAW] D[0..2]: %02X %02X %02X\n",
                              data[0], data[1], data[2]);
            }
        }

        // Byte 2: Seatbelt and Handbrake
        // Bit 6 (0x40): Seatbelt unbuckled warning
        // Bit 0 (0x01): Parking / Handbrake lever pulled
        bool newSeatbelt = (data[2] & 0x40) != 0;
        bool newHandbrake = (data[2] & 0x01) != 0;

        // Byte 1: Lights / Illumination
        bool newHeadlights = (data[1] & 0x04) != 0 || (data[1] & 0x10) != 0;

        bool statusChanged = (newHandbrake != _handbrake) || (newHeadlights != _headlights) ||
                             (newSeatbelt != _seatbeltWarning);

        _seatbeltWarning = newSeatbelt;
        _handbrake = newHandbrake;
        _headlights = newHeadlights;

        if (statusChanged) {
            sendVehicleStatus();
        }
    }
    // Honda 0x405 / 0x324: Genuine Honda Doors Status Frame
    else if ((identifier == 0x405 || identifier == 0x324) && length >= 4) {
        if (_debugEnabled) {
            Serial.printf("[CAN DOORS 0x%03X] D:", identifier);
            for (uint8_t i = 0; i < length; i++) Serial.printf(" %02X", data[i]);
            Serial.println();
        }

        if (!_simDoorActive) {
            // Unpack real Honda door bits
            bool newFL = (data[0] & 0x01) != 0 || (data[3] & 0x20) != 0;
            bool newFR = (data[0] & 0x02) != 0 || (data[3] & 0x40) != 0;
            bool newRL = (data[0] & 0x04) != 0 || (data[3] & 0x80) != 0;
            bool newRR = (data[0] & 0x08) != 0 || (data[3] & 0x01) != 0;
            bool newTrunk = (data[0] & 0x10) != 0;

            bool doorChanged = (newFL != _doorDriver) || (newFR != _doorPassenger) ||
                               (newRL != _doorRearLeft) || (newRR != _doorRearRight) ||
                               (newTrunk != _trunk);

            _doorDriver = newFL;
            _doorPassenger = newFR;
            _doorRearLeft = newRL;
            _doorRearRight = newRR;
            _trunk = newTrunk;

            if (doorChanged) {
                sendDoorsAndSeatbelt();
            }
        }
    }
    // Honda 0x1A4: Automatic Transmission Gear Position (Reverse detection)
    else if (identifier == 0x1A4 && length >= 1) {
        // 0x02 typically marks Reverse (R) on Honda automatic transmissions
        bool isRev = (data[0] == 0x02);
        if (isRev != _reverseGear) {
            _reverseGear = isRev;
            sendVehicleStatus();
        }
    }
}

void CanboxEmulator::update(float speedKmh, float rpm, bool brakePressed, 
                           bool cruiseMain, bool cruiseActive, float targetSpeedKmh) {
#if CANBOX_EMULATOR_ENABLED
    if (!_initialized) return;

    _currentSpeedKmh = speedKmh;
    _currentRpm = rpm;
    _brakePedal = brakePressed;
    _cruiseMain = cruiseMain;
    _cruiseActive = cruiseActive;
    _targetSpeedKmh = targetSpeedKmh;

    uint32_t now = millis();

    // 0. Periodic Version announcement: every 5000ms
    if (now - _lastVersionSendMs >= 5000) {
        sendVersion();
        _lastVersionSendMs = now;
    }

    // 1. Speed & RPM broadcast: 10 Hz (every 100ms) for smooth dashboard needle
    if (now - _lastSpeedSendMs >= 100) {
        sendSpeedAndRpm();
    }

    // 2. Doors & Seatbelt broadcast: Heartbeat every 500ms
    if (now - _lastDoorsSendMs >= 500) {
        sendDoorsAndSeatbelt();
    }

    // 3. Vehicle Status (Handbrake, Lights, Reverse): Heartbeat every 500ms
    if (now - _lastStatusSendMs >= 500) {
        sendVehicleStatus();
    }

    // 4. Cruise Control Telemetry: Heartbeat every 250ms or instant push on state change
    if (_lastCruiseActive != _cruiseActive || (now - _lastCruiseSendMs >= 250)) {
        sendCruiseStatus();
    }

    // 5. Read incoming frames from Multimedia (RX2)
    while (Serial2.available()) {
        uint8_t rxByte = Serial2.read();
        if (_debugEnabled) {
            Serial.printf("[CANBOX RX <- CENTRAL] 0x%02X\n", rxByte);
        }
    }
#endif
}
