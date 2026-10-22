#pragma once

#include <Arduino.h>
#include "config.h"

class CanboxEmulator {
public:
    CanboxEmulator();

    void begin();
    
    // Process periodic transmissions to head unit
    void update(float speedKmh, float rpm, bool brakePressed, 
                bool cruiseMain, bool cruiseActive, float targetSpeedKmh);

    // Ingest relevant Honda CAN frames (0x294, 0x1A4, 0x17C, etc.)
    void parseHondaFrame(uint32_t identifier, const uint8_t* data, uint8_t length);

    // Getters for telemetry / dashboard
    bool isDoorOpen() const { return _doorDriver || _doorPassenger || _doorRearLeft || _doorRearRight || _trunk; }
    bool isSeatbeltWarning() const { return _seatbeltWarning; }
    bool isHandbrakeActive() const { return _handbrake; }
    bool isHeadlightsOn() const { return _headlights; }

    // Live UART debug
    void toggleDebug();
    bool isDebugEnabled() const { return _debugEnabled; }

    // Test tools
    void toggleSimulatedDoor();
    void toggleSimDoor(uint8_t doorIndex);
    void toggleBaudrate();
    uint32_t getBaudrate() const { return _currentBaudrate; }

private:
    bool _initialized;
    bool _debugEnabled;
    uint32_t _currentBaudrate;
    bool _simDoorActive;

    // Body & Safety state (decoded from Honda CAN)
    bool _doorDriver;
    bool _doorPassenger;
    bool _doorRearLeft;
    bool _doorRearRight;
    bool _trunk;
    bool _hood;
    bool _seatbeltWarning; // True if unbuckled
    bool _handbrake;       // True if parking brake is UP
    bool _reverseGear;     // True if in Reverse (R)
    bool _headlights;      // Small lights / Illumination
    bool _highBeam;        // High beam
    bool _turnLeft;
    bool _turnRight;

    // Running vehicle state
    float _currentSpeedKmh;
    float _currentRpm;
    bool  _brakePedal;
    bool  _cruiseMain;
    bool  _cruiseActive;
    float _targetSpeedKmh;

    // State change tracking for instant push
    uint8_t _lastDoorByte;
    uint8_t _lastStatusByte0;
    uint8_t _lastStatusByte1;
    bool    _lastCruiseActive;

    // Periodic timers
    uint32_t _lastSpeedSendMs;
    uint32_t _lastDoorsSendMs;
    uint32_t _lastStatusSendMs;
    uint32_t _lastCruiseSendMs;
    uint32_t _lastVersionSendMs;

    void sendRaisePacket(uint8_t cmd, const uint8_t* payload, uint8_t length);
    void sendVersion();
    void sendDoorsAndSeatbelt();
    void sendVehicleStatus();
    void sendSpeedAndRpm();
    void sendCruiseStatus();
};
