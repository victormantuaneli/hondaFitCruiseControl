#include "mcp4728_dac.h"

Mcp4728Dac::Mcp4728Dac()
    : _i2cAddr(MCP4728_I2C_ADDR),
      _deviceFound(false),
      _lastI2cError(0),
      _currentThrottle(0.0f),
      _app1Volts(APP1_IDLE_VOLTS),
      _app2Volts(APP2_IDLE_VOLTS),
      _vOutA(DAC_INVERT_CHANNELS_A_B ? APP2_IDLE_VOLTS : APP1_IDLE_VOLTS),
      _vOutB(DAC_INVERT_CHANNELS_A_B ? APP1_IDLE_VOLTS : APP2_IDLE_VOLTS) {}

bool Mcp4728Dac::begin(uint8_t i2cAddress) {
    _i2cAddr = i2cAddress;

    // Enable internal pull-ups on ESP32 as fallback
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000); // 100kHz standard mode for high noise margin & reliability
    Wire.setTimeOut(25); // 25ms timeout

    uint8_t err = pingDevice();
    if (err != 0) {
        Serial.printf("[DAC] Warning: MCP4728 not detected on I2C bus (Addr 0x%02X, Error %d)\n", _i2cAddr, err);
        _deviceFound = false;
        return false;
    }

    _deviceFound = true;
    Serial.printf("[DAC] MCP4728 connected successfully at 0x%02X (Channel A: %s, Channel B: %s).\n",
                  _i2cAddr,
                  DAC_INVERT_CHANNELS_A_B ? "APP2" : "APP1",
                  DAC_INVERT_CHANNELS_A_B ? "APP1" : "APP2");
    setIdle(); // Ensure idle voltages on startup
    return true;
}

uint8_t Mcp4728Dac::pingDevice() {
    Wire.beginTransmission(_i2cAddr);
    _lastI2cError = Wire.endTransmission();
    _deviceFound = (_lastI2cError == 0);
    return _lastI2cError;
}

void Mcp4728Dac::scanBus() {
    Serial.println(F("\n--- [I2C BUS SCANNER] ---"));
    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("  [+] Device found at 0x%02X", addr);
            if (addr == 0x60) {
                Serial.print(F(" (MCP4728 DAC Default)"));
            }
            Serial.println();
            count++;
        } else if (error == 4) {
            Serial.printf("  [!] Unknown error at 0x%02X\n", addr);
        }
    }
    if (count == 0) {
        Serial.println(F("  [-] No I2C devices found! Check wiring, pull-ups (SDA/SCL) and power (VCC/GND)."));
    } else {
        Serial.printf("--- Scan complete: %d device(s) found ---\n\n", count);
    }
}

void Mcp4728Dac::setThrottle(float normalizedThrottle, bool allowFullAuthority) {
    // 1. Clamp throttle to safe upper limit (e.g. 50% max authority for PID, 100% for driver override)
    if (normalizedThrottle < 0.0f) normalizedThrottle = 0.0f;
    float maxLimit = allowFullAuthority ? 1.0f : CRUISE_MAX_THROTTLE_LIMIT;
    if (normalizedThrottle > maxLimit) {
        normalizedThrottle = maxLimit;
    }

    _currentThrottle = normalizedThrottle;

    // 2. Compute paired voltages for APP1 and APP2
    _app1Volts = APP1_IDLE_VOLTS + _currentThrottle * (APP1_WOT_VOLTS - APP1_IDLE_VOLTS);
    _app2Volts = APP2_IDLE_VOLTS + _currentThrottle * (APP2_WOT_VOLTS - APP2_IDLE_VOLTS);

#if DAC_INVERT_CHANNELS_A_B
    // Inverted: Channel A (VOUTA) = APP2, Channel B (VOUTB) = APP1
    setOutputs(_app2Volts, _app1Volts);
#else
    // Standard: Channel A (VOUTA) = APP1, Channel B (VOUTB) = APP2
    setOutputs(_app1Volts, _app2Volts);
#endif
}

void Mcp4728Dac::setIdle() {
    _currentThrottle = 0.0f;
    _app1Volts = APP1_IDLE_VOLTS;
    _app2Volts = APP2_IDLE_VOLTS;

#if DAC_INVERT_CHANNELS_A_B
    // Inverted: Channel A (VOUTA) = APP2, Channel B (VOUTB) = APP1
    setOutputs(_app2Volts, _app1Volts);
#else
    // Standard: Channel A (VOUTA) = APP1, Channel B (VOUTB) = APP2
    setOutputs(_app1Volts, _app2Volts);
#endif
}

void Mcp4728Dac::setOutputs(float vOutA, float vOutB) {
    // Clamp to valid range (0V to DAC_VREF_VOLTS)
    vOutA = constrain(vOutA, 0.0f, DAC_VREF_VOLTS);
    vOutB = constrain(vOutB, 0.0f, DAC_VREF_VOLTS);

    _vOutA = vOutA;
    _vOutB = vOutB;

    // Convert to 12-bit DAC codes (0 .. 4095)
    uint16_t dacA = (uint16_t)((vOutA / DAC_VREF_VOLTS) * 4095.0f);
    uint16_t dacB = (uint16_t)((vOutB / DAC_VREF_VOLTS) * 4095.0f);

    writeFastDac(dacA, dacB);
}

void Mcp4728Dac::writeFastDac(uint16_t dacA, uint16_t dacB) {
    if (!_deviceFound) return; // Skip hardware write if on bench/simulator

    // MCP4728 Multi-Write Command (Datasheet Section 5.6.2):
    // Each channel requires its own command byte [0x40 | (channel << 1)], HighByte, LowByte.

    // 1. Update Channel A (Command 0x40, Ch A, UDAC=0)
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x40);
    Wire.write((uint8_t)((dacA >> 8) & 0x0F));
    Wire.write((uint8_t)(dacA & 0xFF));
    uint8_t errA = Wire.endTransmission();

    // 2. Update Channel B (Command 0x42, Ch B, UDAC=0)
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x42);
    Wire.write((uint8_t)((dacB >> 8) & 0x0F));
    Wire.write((uint8_t)(dacB & 0xFF));
    uint8_t errB = Wire.endTransmission();

    if (errA != 0 || errB != 0) {
        _lastI2cError = (errA != 0) ? errA : errB;
        _deviceFound = false;
    }
}

float Mcp4728Dac::getCurrentThrottle() const {
    return _currentThrottle;
}

float Mcp4728Dac::getCurrentApp1Volts() const {
    return _app1Volts;
}

float Mcp4728Dac::getCurrentApp2Volts() const {
    return _app2Volts;
}
