#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class Mcp4728Dac {
public:
    Mcp4728Dac();

    bool begin(uint8_t i2cAddress = MCP4728_I2C_ADDR);

    // Set normalized throttle (0.0 = Idle, 1.0 = WOT)
    // Automatically calculates APP1 and APP2 voltages with strict 2:1 ratio.
    // If allowFullAuthority is false, clamped to CRUISE_MAX_THROTTLE_LIMIT (50% safety ceiling for PID).
    // If allowFullAuthority is true, allows up to 1.0 (100% WOT) for driver pedal override.
    void setThrottle(float normalizedThrottle, bool allowFullAuthority = false);

    // Immediately output idle voltages (e.g. on cruise disengage)
    void setIdle();

    // Direct voltage setting (in Volts)
    void setOutputs(float vOutA, float vOutB);

    float getCurrentThrottle() const;
    float getCurrentApp1Volts() const;
    float getCurrentApp2Volts() const;
    float getVoutA() const { return _vOutA; }
    float getVoutB() const { return _vOutB; }

    bool isDeviceFound() const { return _deviceFound; }
    uint8_t pingDevice();
    static void scanBus();

private:
    uint8_t _i2cAddr;
    bool _deviceFound;
    uint8_t _lastI2cError;
    float _currentThrottle;
    float _app1Volts;
    float _app2Volts;
    float _vOutA;
    float _vOutB;

    void writeFastDac(uint16_t dacA, uint16_t dacB);
};
