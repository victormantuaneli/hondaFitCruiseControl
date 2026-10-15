#include "cruise_controller.h"

CruiseController::CruiseController(
    SteeringSwitchDecoder &switches,
    CanSpeedReader &can,
    Mcp4728Dac &dac,
    SafetySupervisor &safety
) : _switches(switches),
    _can(can),
    _dac(dac),
    _safety(safety),
    _state(CruiseState::OFF),
    _targetSpeedKmh(0.0f),
    _lastSpeedKmh(0.0f),
    _currentThrottle(0.0f),
    _integralTerm(0.0f),
    _lastError(0.0f),
    _lastPidTime(0) {}

void CruiseController::begin() {
    _state = CruiseState::OFF;
    resetPid();
    _dac.setIdle();
    _safety.setRelayEngaged(false);
    _can.setCruiseLamp(false);
}

void CruiseController::resetPid() {
    _integralTerm = 0.0f;
    _lastError = 0.0f;
    _currentThrottle = 0.0f;
    _lastPidTime = millis();
}

static const char* getDisengageReasonString(DisengageReason reason) {
    switch (reason) {
        case DisengageReason::BRAKE_PRESSED:   return "PÉ NO FREIO (Brake Pedal)";
        case DisengageReason::CLUTCH_PRESSED:  return "EMBREAGEM (Clutch)";
        case DisengageReason::CANCEL_BUTTON:   return "BOTÃO CANCEL";
        case DisengageReason::MAIN_SWITCH_OFF: return "CHAVE CRUISE MAIN DESLIGADA";
        case DisengageReason::SPEED_TOO_LOW:   return "VELOCIDADE ABAIXO DO MÍNIMO";
        case DisengageReason::SPEED_TOO_HIGH:  return "VELOCIDADE ACIMA DO MÁXIMO";
        case DisengageReason::CAN_TIMEOUT:     return "CAN TIMEOUT (Sem sinal de velocidade)";
        case DisengageReason::DRIVER_OVERRIDE: return "DRIVER OVERRIDE";
        case DisengageReason::SENSOR_FAULT:    return "FALHA DE SENSOR";
        default:                               return "OUTRO / DESCONHECIDO";
    }
}

void CruiseController::disengage(DisengageReason reason) {
    if (_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE) {
        Serial.printf("\n\n>>> [CRUISE DESATIVOU] Motivo: %s (código %d) | Vel: %.1f km/h <<<\n\n", 
                      getDisengageReasonString(reason), (int)reason, _can.getSpeedKmh());
    }
    _state = _switches.isCruiseMainActive() ? CruiseState::STANDBY : CruiseState::OFF;
    _safety.setRelayEngaged(false);
    _dac.setIdle();
    _can.setCruiseLamp(false);
    resetPid();
}

void CruiseController::update() {
    uint32_t now = millis();
    float currentSpeed = _can.getSpeedKmh();
    bool speedValid = _can.isSpeedValid();

    // 1. Critical Safety Check
    if (_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE) {
        DisengageReason disengageReason = DisengageReason::NONE;
        if (_safety.shouldDisengage(currentSpeed, speedValid, disengageReason)) {
            disengage(disengageReason);
            return;
        }
    }

    // 2. Main Switch Supervision
    bool mainActive = _switches.isCruiseMainActive();
    digitalWrite(PIN_CRUISE_MAIN_LED, mainActive ? HIGH : LOW);

    if (!mainActive && _state != CruiseState::OFF) {
        disengage(DisengageReason::MAIN_SWITCH_OFF);
        _state = CruiseState::OFF;
        return;
    } else if (mainActive && _state == CruiseState::OFF) {
        _state = CruiseState::STANDBY;
        Serial.println(F("[CRUISE] System Armed (STANDBY)"));
    }

    // 3. Process Steering Buttons
    handleInputs();

    // 4. PID Execution & Driver Override (Mode A - Seamless Electronic Pass-Through)
    if (_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE) {
        float driverPedal = _safety.readPhysicalPedalThrottle();

        // Check if driver is pressing physical pedal (clear 3% resting deadband)
        if (driverPedal > 0.03f) {
            if (_state != CruiseState::OVERRIDE) {
                _state = CruiseState::OVERRIDE;
                Serial.printf("[CRUISE] Driver Override Active (Pedal: %.1f%%)\n", driverPedal * 100.0f);
            }
        } else {
            if (_state == CruiseState::OVERRIDE && driverPedal <= 0.02f) {
                _state = CruiseState::ACTIVE;
                Serial.println(F("[CRUISE] Resuming from Driver Override (PID back in control)"));
                _lastPidTime = now;
            }
        }

        // Execute PID at 50ms intervals
        if (now - _lastPidTime >= PID_SAMPLE_TIME_MS) {
            if (_state == CruiseState::OVERRIDE) {
                // When driver accelerates above setpoint, track error without winding up integral
                // so there is no sudden throttle drop/surge when driver releases the gas!
                _lastError = _targetSpeedKmh - currentSpeed;
            } else {
                computePid(currentSpeed);
            }
            _lastPidTime = now;
        }

        // Mode A: Seamless Electronic Pass-Through
        // Progressive response: gives driver immediate additional authority above current cruise throttle,
        // scaling smoothly to 100% WOT with zero dead zone!
        float effectiveThrottle = _currentThrottle;
        if (_state == CruiseState::OVERRIDE) {
            effectiveThrottle = _currentThrottle + (1.0f - _currentThrottle) * driverPedal;
            effectiveThrottle = constrain(effectiveThrottle, 0.0f, 1.0f);
        }

        _dac.setThrottle(effectiveThrottle, (_state == CruiseState::OVERRIDE));
    }

    // Set cluster indicator lamp (0x17C) whenever cruise is ACTIVE or OVERRIDE
    _can.setCruiseLamp(_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE);

    _lastSpeedKmh = currentSpeed;
}

void CruiseController::handleInputs() {
    SwitchStateEvent evt = _switches.update();
    float currentSpeed = _can.getSpeedKmh();
    bool speedValid = _can.isSpeedValid();

    if (evt.button == SteeringButton::NONE) return;

    // CANCEL button: Instant disengage
    if (evt.button == SteeringButton::CANCEL) {
        if (_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE) {
            disengage(DisengageReason::CANCEL_BUTTON);
        }
        return;
    }

    // SET / DECEL button
    if (evt.button == SteeringButton::SET_DECEL) {
        if (_state == CruiseState::STANDBY) {
            if (evt.event == ButtonEvent::SHORT_PRESS || evt.event == ButtonEvent::RELEASED) {
                if (_safety.isSafeToEngage(currentSpeed, speedValid)) {
                    _targetSpeedKmh = currentSpeed;
                    resetPid();

                    // Pre-match DAC to current pedal before relay switches over (seamless transition)
                    float currentPedal = _safety.readPhysicalPedalThrottle();
                    _currentThrottle = constrain(currentPedal, 0.05f, 0.35f);
                    _integralTerm = _currentThrottle;
                    _dac.setThrottle(_currentThrottle);
                    delay(5); // Allow DAC output to stabilize before closing relay contacts

                    _state = CruiseState::ACTIVE;
                    _safety.setRelayEngaged(true);
                    Serial.printf("[CRUISE] Engaged at %.1f km/h (Initial Thr: %.1f%%)\n", 
                                  _targetSpeedKmh, _currentThrottle * 100.0f);
                } else {
                    Serial.printf("[CRUISE] Cannot engage: Speed %.1f km/h out of range\n", currentSpeed);
                }
            }
        } else if (_state == CruiseState::ACTIVE || _state == CruiseState::OVERRIDE) {
            if (_state == CruiseState::OVERRIDE && evt.event == ButtonEvent::SHORT_PRESS) {
                // Tapping SET while overtaking locks in the new higher speed!
                _targetSpeedKmh = constrain(currentSpeed, SPEED_MIN_ENGAGE_KMH, SPEED_MAX_ENGAGE_KMH);
                Serial.printf("[CRUISE] New target speed locked in during override: %.1f km/h\n", _targetSpeedKmh);
            } else if (evt.event == ButtonEvent::SHORT_PRESS) {
                _targetSpeedKmh = max(SPEED_MIN_ENGAGE_KMH, _targetSpeedKmh - SPEED_TAP_STEP_KMH);
                Serial.printf("[CRUISE] Target: %.1f km/h\n", _targetSpeedKmh);
            } else if (evt.event == ButtonEvent::LONG_PRESS_HOLD) {
                _targetSpeedKmh = max(SPEED_MIN_ENGAGE_KMH, _targetSpeedKmh - (SPEED_RAMP_RATE_KMH_S * 0.05f));
            }
        }
    }

    // RES / ACCEL button
    if (evt.button == SteeringButton::RES_ACCEL) {
        if (_state == CruiseState::STANDBY && _targetSpeedKmh >= SPEED_MIN_ENGAGE_KMH) {
            if (evt.event == ButtonEvent::SHORT_PRESS || evt.event == ButtonEvent::RELEASED) {
                if (_safety.isSafeToEngage(currentSpeed, speedValid)) {
                    resetPid();

                    // Pre-match DAC to current pedal before relay switches over
                    float currentPedal = _safety.readPhysicalPedalThrottle();
                    _currentThrottle = constrain(currentPedal, 0.05f, 0.35f);
                    _integralTerm = _currentThrottle;
                    _dac.setThrottle(_currentThrottle);
                    delay(5);

                    _state = CruiseState::ACTIVE;
                    _safety.setRelayEngaged(true);
                    Serial.printf("[CRUISE] Resumed to %.1f km/h (Initial Thr: %.1f%%)\n", 
                                  _targetSpeedKmh, _currentThrottle * 100.0f);
                }
            }
        } else if (_state == CruiseState::ACTIVE) {
            if (evt.event == ButtonEvent::SHORT_PRESS) {
                _targetSpeedKmh = min(SPEED_MAX_ENGAGE_KMH, _targetSpeedKmh + SPEED_TAP_STEP_KMH);
                Serial.printf("[CRUISE] Target: %.1f km/h\n", _targetSpeedKmh);
            } else if (evt.event == ButtonEvent::LONG_PRESS_HOLD) {
                _targetSpeedKmh = min(SPEED_MAX_ENGAGE_KMH, _targetSpeedKmh + (SPEED_RAMP_RATE_KMH_S * 0.05f));
            }
        }
    }
}

void CruiseController::computePid(float currentSpeed) {
    float dt = (float)PID_SAMPLE_TIME_MS / 1000.0f; // 0.05 seconds
    float error = _targetSpeedKmh - currentSpeed;

    // 1. Proportional term
    float pTerm = PID_KP * error;

    // 2. Integral term with anti-windup clamping
    _integralTerm += (PID_KI * error * dt);
    _integralTerm = constrain(_integralTerm, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);

    // 3. Derivative on error
    float dTerm = PID_KD * (error - _lastError) / dt;
    _lastError = error;

    // Base target throttle from PID
    float rawDemand = pTerm + _integralTerm + dTerm;
    if (rawDemand < 0.0f) rawDemand = 0.0f;

    // 4. Slew Rate Limiter (prevents sudden pedal jumps)
    float throttleDelta = rawDemand - _currentThrottle;
    throttleDelta = constrain(throttleDelta, -MAX_THROTTLE_SLEW_PER_CYCLE, MAX_THROTTLE_SLEW_PER_CYCLE);
    _currentThrottle += throttleDelta;

    // 5. Final Safety Clamp for PID authority (capped at CRUISE_MAX_THROTTLE_LIMIT, e.g. 50%)
    _currentThrottle = constrain(_currentThrottle, 0.0f, CRUISE_MAX_THROTTLE_LIMIT);

    // Note: Actual DAC write is managed in update() with effectiveThrottle = max(PID, driverPedal)
}

CruiseState CruiseController::getState() const {
    return _state;
}

float CruiseController::getTargetSpeed() const {
    return _targetSpeedKmh;
}

float CruiseController::getCurrentThrottle() const {
    return _currentThrottle;
}

const char* CruiseController::getStateString() const {
    switch (_state) {
        case CruiseState::OFF:      return "OFF";
        case CruiseState::STANDBY:  return "STANDBY";
        case CruiseState::ACTIVE:   return "ACTIVE";
        case CruiseState::OVERRIDE: return "OVERRIDE";
        case CruiseState::FAULT:    return "FAULT";
        default:                    return "UNKNOWN";
    }
}
