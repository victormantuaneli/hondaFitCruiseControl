#include "safety_supervisor.h"

SafetySupervisor::SafetySupervisor() 
    : _relayState(false), _lastSafeCheckTime(0), _simBrakeUntilMs(0), _simOverrideActive(false), _canBrakePressed(false) {}

void SafetySupervisor::begin() {
    if (!BRAKE_READ_FROM_CAN) {
        // Isolated brake switch pins
        // NC contact: grounded at rest -> reads LOW; opens on brake press -> pulled HIGH
        pinMode(PIN_BRAKE_NC_SW, INPUT_PULLUP);
        // NO contact: open at rest -> pulled HIGH; closes to GND on brake press -> reads LOW
        pinMode(PIN_BRAKE_NO_SW, INPUT_PULLUP);
    }

    // Clutch / Neutral switch (grounded when pedal pressed / out of gear)
    pinMode(PIN_CLUTCH_SW, INPUT_PULLUP);

    // Physical pedal analog input
    pinMode(PIN_PEDAL_RAW_APP1_ADC, INPUT);

    // Auto-calibrate resting pedal baseline on startup
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(PIN_PEDAL_RAW_APP1_ADC);
        delay(2);
    }
    float rawAvg = sum / 16.0f;
    _restingPedalVolts = (rawAvg / 4095.0f) * 3.3f;
    if (_restingPedalVolts < 0.05f) _restingPedalVolts = 0.22f; // Fallback
    _lastPedalVolts = _restingPedalVolts;
    Serial.printf("[PEDAL] Calibrado repouso: %.2fV\n", _restingPedalVolts);

    // Relay control pin (must start LOW / de-energized)
    pinMode(PIN_RELAY_EN, OUTPUT);
    digitalWrite(PIN_RELAY_EN, LOW);

    // Indicator LEDs
    pinMode(PIN_CRUISE_ACTIVE_LED, OUTPUT);
    pinMode(PIN_CRUISE_MAIN_LED, OUTPUT);
    digitalWrite(PIN_CRUISE_ACTIVE_LED, LOW);
    digitalWrite(PIN_CRUISE_MAIN_LED, LOW);
}

void SafetySupervisor::setCanBrakePressed(bool pressed) {
    _canBrakePressed = pressed;
}

bool SafetySupervisor::isBrakePressed() const {
    if (millis() < _simBrakeUntilMs) {
        return true;
    }
    if (BRAKE_READ_FROM_CAN) {
        return _canBrakePressed;
    }
    // Redundant physical check:
    // If NC switch opens (HIGH) OR NO switch closes to GND (LOW) -> Brake is PRESSED
    bool ncTripped = (digitalRead(PIN_BRAKE_NC_SW) == HIGH);
    bool noTripped = (digitalRead(PIN_BRAKE_NO_SW) == LOW);

    return (ncTripped || noTripped);
}

bool SafetySupervisor::isClutchDepressed() const {
#if !CLUTCH_CHECK_ENABLED
    return false;
#else
    // Low means clutch pedal is pressed down (or gear shift in Neutral)
    return (digitalRead(PIN_CLUTCH_SW) == LOW);
#endif
}

float SafetySupervisor::readPhysicalPedalThrottle() {
    uint16_t raw = analogRead(PIN_PEDAL_RAW_APP1_ADC);
    float volts = (raw / 4095.0f) * 3.3f; // ESP32 3.3V reference
    _lastPedalVolts = volts;

    // Adapt baseline if pedal voltage drops lower while released (slow downward drift tracking)
    if (volts > 0.05f && volts < _restingPedalVolts) {
        _restingPedalVolts = 0.90f * _restingPedalVolts + 0.10f * volts;
    }

    // 25mV noise margin above rest ensures clean 0.0% without dead zone
    float idleV = _restingPedalVolts + 0.025f;

    // Detect span automatically based on resting voltage (with or without 2:1 divider)
    float wotSpan = 1.80f; // Standard span (~0.45V to 2.25V)
    if (_restingPedalVolts < 0.35f) {
        wotSpan = 0.90f; // Scaled span for APP2 through 2:1 divider (~0.21V to 1.12V)
    }

    float throttle = (volts - idleV) / wotSpan;
    return constrain(throttle, 0.0f, 1.0f);
}

bool SafetySupervisor::isDriverOverriding(float cruiseCurrentThrottle) {
    if (_simOverrideActive) return true;
    float driverPedal = readPhysicalPedalThrottle();
    // Driver presses pedal past resting threshold (2.5% deadband)
    return (driverPedal > 0.025f);
}

void SafetySupervisor::triggerSimulatedBrake(uint32_t durationMs) {
    _simBrakeUntilMs = millis() + durationMs;
    Serial.printf("[DEBUG] Simulated Brake Pressed for %d ms\n", durationMs);
}

void SafetySupervisor::toggleSimulatedOverride() {
    _simOverrideActive = !_simOverrideActive;
    Serial.printf("[DEBUG] Simulated Driver Override: %s\n", _simOverrideActive ? "ACTIVE (Foot on gas)" : "RELEASED");
}

bool SafetySupervisor::isSafeToEngage(float currentSpeedKmh, bool canSpeedValid) {
    if (!canSpeedValid) return false;
    if (isBrakePressed()) return false;
    if (isClutchDepressed()) return false;
    if (currentSpeedKmh < SPEED_MIN_ENGAGE_KMH || currentSpeedKmh > SPEED_MAX_ENGAGE_KMH) return false;
    return true;
}

bool SafetySupervisor::shouldDisengage(float currentSpeedKmh, bool canSpeedValid, DisengageReason &reason) {
    if (isBrakePressed()) {
        reason = DisengageReason::BRAKE_PRESSED;
        return true;
    }
    if (isClutchDepressed()) {
        reason = DisengageReason::CLUTCH_PRESSED;
        return true;
    }
    if (!canSpeedValid) {
        reason = DisengageReason::CAN_TIMEOUT;
        return true;
    }
    if (currentSpeedKmh < (SPEED_MIN_ENGAGE_KMH - 5.0f)) {
        reason = DisengageReason::SPEED_TOO_LOW;
        return true;
    }
    if (currentSpeedKmh > (SPEED_MAX_ENGAGE_KMH + 10.0f)) {
        reason = DisengageReason::SPEED_TOO_HIGH;
        return true;
    }
    reason = DisengageReason::NONE;
    return false;
}

void SafetySupervisor::setRelayEngaged(bool engage) {
    _relayState = engage;
#if RELAY_OUTPUT_ENABLED
    digitalWrite(PIN_RELAY_EN, engage ? HIGH : LOW);
#else
    digitalWrite(PIN_RELAY_EN, LOW); // Safe test mode: relay coil unpowered
#endif
    digitalWrite(PIN_CRUISE_ACTIVE_LED, engage ? HIGH : LOW);

    if (!engage) {
        // Double-check pin went LOW immediately
        digitalWrite(PIN_RELAY_EN, LOW);
    }
}

bool SafetySupervisor::isRelayEngaged() const {
    return _relayState;
}
