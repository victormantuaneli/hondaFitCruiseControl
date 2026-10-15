#pragma once

#include <Arduino.h>
#include "config.h"
#include "steering_switch.h"

class CanSpeedReader {
public:
    CanSpeedReader();
    
    // Initialize ESP32 TWAI (CAN) controller
    bool begin();
    
    // Process incoming CAN frames (called in CAN FreeRTOS task or loop)
    void update();
    
    // Speed & Brake telemetry
    float getSpeedKmh() const;
    float getEngineRpm() const;
    bool  isSpeedValid() const;
    bool  isBrakePressed() const;
    uint32_t getLastReceivedMs() const;

    // Steering wheel cruise buttons via CAN (0x164)
    bool isCruiseMainActive() const;
    SteeringButton getPressedButton() const;

    // Cluster CRUISE CONTROL indicator lamp injection (0x17C)
    void setCruiseLamp(bool on);
    bool isCruiseLampOn() const;
    void toggleManualLampTest();


    // Simulation / Bench Debug Injection
    void setSimulatedSpeed(float speedKmh, bool valid = true);
    void toggleVirtualMainSwitch();

    enum class SnifferMode {
        OFF = 0,
        FILTERED,   // Shows only relevant cruise/steering/speed IDs (0x158, 0x164, 0x17C) on change
        ALL_RAW     // Dumps every single raw CAN packet
    };

    // On-the-fly CAN sniffer mode
    void toggleSnifferOutput();
    SnifferMode getSnifferMode() const;

private:
    float _speedKmh;
    float _engineRpm;
    uint32_t _lastRxTimeMs;
    bool _initialized;
    bool _benchSimActive;
    bool _realCanReceived;
    bool _brakePressedCan;
    bool _canMainSw;
    SteeringButton _canButton;
    bool _cruiseLampOn;
    bool _manualLampTest;
    uint8_t _latest17c[8];
    bool _has17c;
    SnifferMode _snifferMode;

    // Last seen state for filtered change detection
    uint8_t _lastB0_164;
    uint8_t _lastB4_17C;
    
    void parseHondaFrame(uint32_t identifier, const uint8_t* data, uint8_t length);
};
