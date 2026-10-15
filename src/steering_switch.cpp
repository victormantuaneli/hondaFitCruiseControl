#include "steering_switch.h"

SteeringSwitchDecoder::SteeringSwitchDecoder()
    : _currentRawButton(SteeringButton::NONE),
      _debouncedButton(SteeringButton::NONE),
      _pressStartTime(0),
      _lastRawSampleTime(0),
      _isLongPressReported(false),
      _debouncedMainSw(false),
      _lastRawMainSw(false),
      _mainSwDebounceTime(0),
      _canMainSw(false),
      _canButton(SteeringButton::NONE) {}

void SteeringSwitchDecoder::begin() {
    pinMode(PIN_SW_SET_DECEL, INPUT_PULLUP);
    pinMode(PIN_SW_RES_ACCEL, INPUT_PULLUP);
    pinMode(PIN_CRUISE_MAIN_SW, INPUT_PULLUP);

    if (!SW_READ_FROM_CAN) {
        // Initial read of latching switch
        _debouncedMainSw = (digitalRead(PIN_CRUISE_MAIN_SW) == LOW);
        _lastRawMainSw = _debouncedMainSw;
    }
}

void SteeringSwitchDecoder::setCanInputs(bool mainSw, SteeringButton btn) {
    _canMainSw = mainSw;
    _canButton = btn;
}

SteeringButton SteeringSwitchDecoder::sampleDiscreteButtons() {
    if (SW_READ_FROM_CAN) {
        return _canButton;
    }

    bool setLineLow = (digitalRead(PIN_SW_SET_DECEL) == LOW);
    bool resLineLow = (digitalRead(PIN_SW_RES_ACCEL) == LOW);

    // Honda Combination Switch Truth Table:
    // - CANCEL: Pulls BOTH YEL/RED and LT GRN/BLK to GND simultaneously
    // - SET:    Pulls ONLY YEL/RED to GND
    // - RES:    Pulls ONLY LT GRN/BLK to GND
    // - IDLE:   Both lines pulled HIGH (3.3V)
    if (setLineLow && resLineLow) {
        return SteeringButton::CANCEL;
    } else if (setLineLow && !resLineLow) {
        return SteeringButton::SET_DECEL;
    } else if (!setLineLow && resLineLow) {
        return SteeringButton::RES_ACCEL;
    }
    return SteeringButton::NONE;
}

static SwitchStateEvent s_virtualEvent = {SteeringButton::NONE, ButtonEvent::NONE, 0};
static bool s_hasVirtualEvent = false;

void SteeringSwitchDecoder::injectVirtualEvent(SteeringButton btn, ButtonEvent evt) {
    s_virtualEvent.button = btn;
    s_virtualEvent.event = evt;
    s_virtualEvent.durationMs = (evt == ButtonEvent::LONG_PRESS_HOLD) ? 600 : 50;
    s_hasVirtualEvent = true;
}

void SteeringSwitchDecoder::toggleVirtualMainSwitch() {
    _debouncedMainSw = !_debouncedMainSw;
    _lastRawMainSw = _debouncedMainSw;
    _canMainSw = _debouncedMainSw;
    Serial.printf("[DEBUG] Virtual Cruise Main toggled: %s\n", _debouncedMainSw ? "ON" : "OFF");
}

SwitchStateEvent SteeringSwitchDecoder::update() {
    if (s_hasVirtualEvent) {
        s_hasVirtualEvent = false;
        return s_virtualEvent;
    }

    uint32_t now = millis();
    SwitchStateEvent result = {SteeringButton::NONE, ButtonEvent::NONE, 0};

    // 1. Process CRUISE Main Switch
    bool rawMainSw = SW_READ_FROM_CAN ? _canMainSw : (digitalRead(PIN_CRUISE_MAIN_SW) == LOW);
    if (rawMainSw != _lastRawMainSw) {
        _lastRawMainSw = rawMainSw;
        _mainSwDebounceTime = now;
    } else if ((now - _mainSwDebounceTime) >= (SW_READ_FROM_CAN ? 100 : 50)) {
        if (_debouncedMainSw != rawMainSw) {
            _debouncedMainSw = rawMainSw;
            Serial.printf("\n[SWITCH] Cruise Main is now: %s\n", _debouncedMainSw ? "ON (STANDBY)" : "OFF");
        }
    }

    // 2. Sample 3-wire combination buttons
    SteeringButton sampled = sampleDiscreteButtons();

    // Instant safety priority on CANCEL (don't wait for debounce)
    if (sampled == SteeringButton::CANCEL && _debouncedButton != SteeringButton::CANCEL) {
        _debouncedButton = SteeringButton::CANCEL;
        _pressStartTime = now;
        result.button = SteeringButton::CANCEL;
        result.event = ButtonEvent::SHORT_PRESS;
        result.durationMs = 0;
        return result;
    }

    // Debounce state tracking for SET and RES
    if (sampled != _currentRawButton) {
        _currentRawButton = sampled;
        _lastRawSampleTime = now;
    } else if ((now - _lastRawSampleTime) >= SW_DEBOUNCE_MS) {
        if (_currentRawButton != _debouncedButton) {
            // Button pressed down
            if (_debouncedButton == SteeringButton::NONE && _currentRawButton != SteeringButton::NONE) {
                _pressStartTime = now;
                _isLongPressReported = false;
            }
            // Button released
            else if (_debouncedButton != SteeringButton::NONE && _currentRawButton == SteeringButton::NONE) {
                uint32_t duration = now - _pressStartTime;
                if (!_isLongPressReported && _debouncedButton != SteeringButton::CANCEL) {
                    result.button = _debouncedButton;
                    result.event = ButtonEvent::SHORT_PRESS;
                    result.durationMs = duration;
                } else {
                    result.button = _debouncedButton;
                    result.event = ButtonEvent::RELEASED;
                    result.durationMs = duration;
                }
            }
            _debouncedButton = _currentRawButton;
        } else if (_debouncedButton != SteeringButton::NONE && _debouncedButton != SteeringButton::CANCEL) {
            // Button being held
            uint32_t holdDuration = now - _pressStartTime;
            if (holdDuration >= SW_LONG_PRESS_MS) {
                _isLongPressReported = true;
                result.button = _debouncedButton;
                result.event = ButtonEvent::LONG_PRESS_HOLD;
                result.durationMs = holdDuration;
            }
        }
    }

    return result;
}

bool SteeringSwitchDecoder::isCruiseMainActive() {
    return _debouncedMainSw;
}
