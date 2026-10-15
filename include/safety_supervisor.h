#pragma once

#include <Arduino.h>
#include "config.h"

enum class DisengageReason {
    NONE = 0,
    BRAKE_PRESSED,
    CLUTCH_PRESSED,
    CANCEL_BUTTON,
    MAIN_SWITCH_OFF,
    SPEED_TOO_LOW,
    SPEED_TOO_HIGH,
    CAN_TIMEOUT,
    DRIVER_OVERRIDE,
    SENSOR_FAULT
};

class SafetySupervisor {
public:
    SafetySupervisor();

    void begin();

    // Fast check executed in safety loop
    bool isSafeToEngage(float currentSpeedKmh, bool canSpeedValid);
    bool shouldDisengage(float currentSpeedKmh, bool canSpeedValid, DisengageReason &reason);

    // Relay control
    void setRelayEngaged(bool engage);
    bool isRelayEngaged() const;

    // Pedal override check (driver pressing physical gas pedal)
    bool isDriverOverriding(float cruiseCurrentThrottle);

    // Brake state
    void setCanBrakePressed(bool pressed);
    bool isBrakePressed() const;
    bool isClutchDepressed() const;

    // Bench debug / simulation injection
    void triggerSimulatedBrake(uint32_t durationMs = 500);
    void toggleSimulatedOverride();

    float readPhysicalPedalThrottle();
    float getRawPedalVolts() const { return _lastPedalVolts; }

private:
    bool _relayState;
    uint32_t _lastSafeCheckTime;
    uint32_t _simBrakeUntilMs;
    bool _simOverrideActive;
    bool _canBrakePressed;
    float _restingPedalVolts;
    float _lastPedalVolts;
};
