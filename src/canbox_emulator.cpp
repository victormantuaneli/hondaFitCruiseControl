#include "canbox_emulator.h"

CanboxEmulator::CanboxEmulator()
    : _initialized(false),
      _doorDriver(false),
      _doorPassenger(false),
      _doorRearLeft(false),
      _doorRearRight(false),
      _trunk(false),
      _hood(false),
      _seatbeltWarning(true),
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
      _lastCruiseSendMs(0) {}

void CanboxEmulator::begin() {
#if CANBOX_EMULATOR_ENABLED
    // Initialize Hardware UART2 on ESP32 (38400 baud, 8N1, RX: GPIO 16, TX: GPIO 17)
    Serial2.begin(CANBOX_UART_BAUDRATE, SERIAL_8N1, PIN_CANBOX_RX, PIN_CANBOX_TX);
    _initialized = true;
    Serial.printf("[CANBOX] Emulador Raise RZC Honda ativo na Serial2 (TX: GPIO %d, RX: GPIO %d, %d bps)\n",
                  PIN_CANBOX_TX, PIN_CANBOX_RX, CANBOX_UART_BAUDRATE);

    // Initial broadcast to announce presence to the head unit
    sendDoorsAndSeatbelt();
    sendVehicleStatus();
    sendSpeedAndRpm();
    sendCruiseStatus();
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
#endif
}

void CanboxEmulator::sendDoorsAndSeatbelt() {
    // Command 0x24: Doors, Trunk, Hood & Seatbelt
    uint8_t doorByte = 0;
    if (_doorDriver)       doorByte |= 0x80; // Bit 7: Driver door open
    if (_doorPassenger)    doorByte |= 0x40; // Bit 6: Passenger door open
    if (_doorRearLeft)     doorByte |= 0x20; // Bit 5: Rear-Left door open
    if (_doorRearRight)    doorByte |= 0x10; // Bit 4: Rear-Right door open
    if (_trunk)            doorByte |= 0x08; // Bit 3: Trunk open
    if (_hood)             doorByte |= 0x04; // Bit 2: Hood open
    if (_seatbeltWarning)  doorByte |= 0x02; // Bit 1: Driver seatbelt unbuckled

    uint8_t payload[1] = { doorByte };
    sendRaisePacket(0x24, payload, 1);
    _lastDoorByte = doorByte;
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
    // Command 0x16: Speed and RPM telemetry for Head Unit Dashboard
    uint16_t spd = (uint16_t)(_currentSpeedKmh >= 0.0f ? _currentSpeedKmh : 0.0f);
    uint16_t rpm = (uint16_t)(_currentRpm >= 0.0f ? _currentRpm : 0.0f);

    uint8_t payload[4];
    payload[0] = (uint8_t)(spd & 0xFF);         // Speed in km/h
    payload[1] = (uint8_t)((spd >> 8) & 0xFF);  // High byte (if speed > 255)
    payload[2] = (uint8_t)((rpm >> 8) & 0xFF);  // RPM high byte
    payload[3] = (uint8_t)(rpm & 0xFF);         // RPM low byte

    sendRaisePacket(0x16, payload, 4);
    _lastSpeedSendMs = millis();
}

void CanboxEmulator::sendCruiseStatus() {
    // Command 0x32: Cruise Control & Vehicle Information Status
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

    // Honda 0x294: Body, Cluster Warnings, Seatbelt, Handbrake, Doors, Turn Signals
    if (identifier == 0x294 && length >= 6) {
        // Byte 2: Seatbelt and Handbrake
        // Bit 6 (0x40): Seatbelt unbuckled warning
        // Bit 0 (0x01): Parking / Handbrake lever pulled
        bool newSeatbelt = (data[2] & 0x40) != 0;
        bool newHandbrake = (data[2] & 0x01) != 0;

        // Byte 1: Lights / Illumination
        bool newHeadlights = (data[1] & 0x04) != 0 || (data[1] & 0x10) != 0;

        // Byte 5: Door switches (Honda Fit G2)
        // Bit 0: Driver door
        // Bit 1: Passenger door
        // Bit 3: Rear-Left door
        // Bit 4: Rear-Right door
        // Bit 2: Trunk
        bool newDoorDriver = (data[5] & 0x01) != 0;
        bool newDoorPass = (data[5] & 0x02) != 0;
        bool newTrunk = (data[5] & 0x04) != 0;
        bool newDoorRL = (data[5] & 0x08) != 0;
        bool newDoorRR = (data[5] & 0x10) != 0;

        // Immediate push on any door or safety transition
        bool doorChanged = (newDoorDriver != _doorDriver) || (newDoorPass != _doorPassenger) ||
                           (newDoorRL != _doorRearLeft) || (newDoorRR != _doorRearRight) ||
                           (newTrunk != _trunk) || (newSeatbelt != _seatbeltWarning);

        bool statusChanged = (newHandbrake != _handbrake) || (newHeadlights != _headlights);

        _doorDriver = newDoorDriver;
        _doorPassenger = newDoorPass;
        _trunk = newTrunk;
        _doorRearLeft = newDoorRL;
        _doorRearRight = newDoorRR;
        _seatbeltWarning = newSeatbelt;
        _handbrake = newHandbrake;
        _headlights = newHeadlights;

        if (doorChanged) {
            sendDoorsAndSeatbelt();
        }
        if (statusChanged) {
            sendVehicleStatus();
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
#endif
}
