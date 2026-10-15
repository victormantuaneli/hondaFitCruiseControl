#pragma once

#include <Arduino.h>
#include "config.h"
#include "steering_switch.h"
#include "can_speed.h"
#include "mcp4728_dac.h"
#include "safety_supervisor.h"

enum class CruiseState {
    OFF = 0,        // Cruise Main switch disabled
    STANDBY,        // Armed, waiting for SET- or RES+
    ACTIVE,         // Regulating speed
    OVERRIDE,       // Driver temporarily accelerating past setpoint
    FAULT           // Safety trip / CAN failure
};

class CruiseController {
public:
    CruiseController(
        SteeringSwitchDecoder &switches,
        CanSpeedReader &can,
        Mcp4728Dac &dac,
        SafetySupervisor &safety
    );

    void begin();
    
    // Core control loop (runs at 20 Hz / 50ms)
    void update();

    CruiseState getState() const;
    float getTargetSpeed() const;
    float getCurrentThrottle() const;
    const char* getStateString() const;

private:
    SteeringSwitchDecoder &_switches;
    CanSpeedReader &_can;
    Mcp4728Dac &_dac;
    SafetySupervisor &_safety;

    CruiseState _state;
    float _targetSpeedKmh;
    float _lastSpeedKmh;
    float _currentThrottle;
    
    // PID state
    float _integralTerm;
    float _lastError;
    uint32_t _lastPidTime;

    void handleInputs();
    void computePid(float currentSpeed);
    void disengage(DisengageReason reason);
    void resetPid();
};
