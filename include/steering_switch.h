#pragma once

#include <Arduino.h>
#include "config.h"

enum class SteeringButton {
    NONE = 0,
    CANCEL,
    SET_DECEL,
    RES_ACCEL
};

enum class ButtonEvent {
    NONE = 0,
    SHORT_PRESS,
    LONG_PRESS_HOLD,
    RELEASED
};

struct SwitchStateEvent {
    SteeringButton button;
    ButtonEvent event;
    uint32_t durationMs;
};

class SteeringSwitchDecoder {
public:
    SteeringSwitchDecoder();
    
    void begin();
    
    // Call every 10-20ms to sample inputs and decode events
    SwitchStateEvent update();
    
    // CAN Bus Input Injection (0x164)
    void setCanInputs(bool mainSw, SteeringButton btn);

    // Checks the state of the mechanical latching CRUISE MAIN switch
    bool isCruiseMainActive();

    // Bench debug / simulation injection
    void injectVirtualEvent(SteeringButton btn, ButtonEvent evt);
    void toggleVirtualMainSwitch();

private:
    SteeringButton _currentRawButton;
    SteeringButton _debouncedButton;
    uint32_t _pressStartTime;
    uint32_t _lastRawSampleTime;
    bool _isLongPressReported;

    // Latching switch debouncing
    bool _debouncedMainSw;
    bool _lastRawMainSw;
    uint32_t _mainSwDebounceTime;

    // CAN bus inputs
    bool _canMainSw;
    SteeringButton _canButton;

    SteeringButton sampleDiscreteButtons();
};
