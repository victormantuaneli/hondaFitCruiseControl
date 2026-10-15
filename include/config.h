#pragma once

#include <Arduino.h>

// ==============================================================================
// 1. HARDWARE PIN ASSIGNMENTS (ESP32)
// ==============================================================================

// Steering Wheel Combination Switch
// Set SW_READ_FROM_CAN to true to read CRUISE MAIN, SET, RES, CANCEL from Honda F-CAN (ID 0x164)
#define SW_READ_FROM_CAN            true
#define PIN_SW_SET_DECEL            32  // Digital input with PULLUP (YEL/RED)
#define PIN_SW_RES_ACCEL            33  // Digital input with PULLUP (LT GRN/BLK)
#define PIN_CRUISE_MAIN_SW          14  // Digital input with PULLUP (LT GRN - Latching switch to GND)

// CAN Bus (TWAI) Transceiver Pins (e.g., SN65HVD230 / VP230)
#define PIN_CAN_TX                  5   // ESP32 TWAI TX -> Transceiver TXD
#define PIN_CAN_RX                  4   // ESP32 TWAI RX -> Transceiver RXD

// MCP2515 Standalone SPI CAN Controller Pins (Standard ESP32 VSPI)
#define PIN_MCP2515_CS              5   // Chip Select (CS)
#define PIN_MCP2515_SCK             18  // SPI Clock (SCK)
#define PIN_MCP2515_MISO            19  // SPI MISO (SO on MCP2515 board)
#define PIN_MCP2515_MOSI            23  // SPI MOSI (SI on MCP2515 board)
#define PIN_MCP2515_INT             4   // Interrupt pin (INT on MCP2515 board)
#define MCP2515_DEFAULT_OSC_MHZ     8   // Most blue Chinese modules use 8MHz crystal (some use 16MHz)

// I2C Bus for MCP4728 12-bit DAC
#define PIN_I2C_SDA                 21  // ESP32 SDA -> MCP4728 SDA
#define PIN_I2C_SCL                 22  // ESP32 SCL -> MCP4728 SCL
#define PIN_MCP4728_LDAC            13  // MCP4728 LDAC (Optional, pull to GND or control via GPIO)

// Safety Disengage Inputs
// Set BRAKE_READ_FROM_CAN to true to read brake pedal switch digitally from Honda F-CAN (ID 0x17C / 0x1A6)
// Set to false if using physical discrete switch wiring to GPIO 18/19
#define BRAKE_READ_FROM_CAN         true
#define PIN_BRAKE_NC_SW             18  // Brake switch NC (Normally Closed: HIGH when brake pressed / wire broken)
#define PIN_BRAKE_NO_SW             19  // Brake switch NO (Normally Open: HIGH when brake pressed)

// Clutch Supervision (Set to false if automatic or no physical clutch switch wired to GPIO 23)
#define CLUTCH_CHECK_ENABLED        false
#define PIN_CLUTCH_SW               23  // Clutch / Neutral switch (LOW when pedal depressed / not in gear)

// Physical Gas Pedal Readout (Driver Override Detection)
#define PIN_PEDAL_RAW_APP1_ADC      36    // ADC1_CH0 (VP): Reads physical gas pedal voltage before relay
#define PEDAL_ADC_READS_APP2        true  // GPIO 36 is wired to APP2 (0.42V-2.25V)
#define PEDAL_APP1_IDLE_VOLTS_DIV   0.45f // Voltage at ADC pin at idle (0.85V / 2 + offset margin)
#define PEDAL_APP1_WOT_VOLTS_DIV    2.25f // Voltage at ADC pin at WOT (4.50V / 2)
#define PEDAL_APP2_IDLE_VOLTS       0.45f // Idle voltage threshold for APP2
#define PEDAL_APP2_WOT_VOLTS        2.25f // WOT voltage for APP2

// Actuator / Relay Control & Indicators
#define RELAY_OUTPUT_ENABLED        true // Set to false to test logic without energizing relay coil (diagnosing brownouts)
#define PIN_RELAY_EN                25  // Drives DPDT Safety Relay coil via NPN/MOSFET (HIGH = Engage Cruise DAC)
#define PIN_CRUISE_ACTIVE_LED       26  // Indicator LED: Cruise actively controlling speed
#define PIN_CRUISE_MAIN_LED         27  // Indicator LED: Cruise system armed (Main switch ON)

// ==============================================================================
// 2. SWITCH TIMING & DEBOUNCE
// ==============================================================================
#define SW_DEBOUNCE_MS              25  // Debounce time for button press (ms)
#define SW_LONG_PRESS_MS            500 // Hold threshold for continuous accel/decel ramp (ms)

// ==============================================================================
// 3. CAN BUS CONFIGURATION (HONDA F-CAN)
// ==============================================================================
#define CAN_BAUDRATE_KBPS           500   // Honda F-CAN default is 500 kbps
#define CAN_HONDA_SPEED_ID          0x158 // Common Honda CAN broadcast frame for Vehicle Speed & RPM
#define CAN_SPEED_TIMEOUT_MS        1000  // Disengage if no valid speed received in 1000ms (prevents transient drops)
#define CAN_LAMP_INJECTION_ENABLED  true  // Enable 0x17C cluster light injection (set to false if testing listen-only)
#define CAN_LAMP_BURST_PERIOD_MS    1000  // Período total do ciclo (1000ms = 1 segundo)
#define CAN_LAMP_BURST_ON_MS        250   // Duração da rajada contínua (250ms acesa, 750ms apagada)

// When BENCH_SIM_MODE is true, ESP32 generates virtual 70 km/h speed when no real
// vehicle CAN bus is connected, allowing complete testing on bench/Wokwi.
// Once real CAN packets arrive from a vehicle, real CAN automatically takes over!
#define BENCH_SIM_MODE              true

// ==============================================================================
// 4. SPEED & SAFETY THRESHOLDS
// ==============================================================================
#define SPEED_MIN_ENGAGE_KMH        25.0f // Minimum speed allowed to engage cruise (25 km/h)
#define SPEED_MAX_ENGAGE_KMH        140.0f// Maximum speed allowed to engage cruise (140 km/h)
#define SPEED_TAP_STEP_KMH          1.0f  // Speed change per short tap on SET- or RES+
#define SPEED_RAMP_RATE_KMH_S       2.0f  // Acceleration rate when holding RES+ (km/h per second)

// Maximum throttle authority granted to cruise PID (Safety ceiling)
// 0.0f = 0% throttle, 1.0f = 100% throttle
#define CRUISE_MAX_THROTTLE_LIMIT   0.50f // Max 50% throttle authority to prevent runaway

// Rate limiter: Max throttle change per 50ms control loop cycle (prevents jerky response)
#define MAX_THROTTLE_SLEW_PER_CYCLE 0.02f // 2% change per 50ms cycle (~40%/sec max slew)

// ==============================================================================
// 5. MCP4728 & ACCELERATOR PEDAL (APP) VOLTAGE MAPPING
// ==============================================================================
#define MCP4728_I2C_ADDR            0x60

// Honda DBW typically uses dual potentiometer signals:
// APP1: 0.85V (idle) to 4.50V (WOT)
// APP2: 0.42V (idle) to 2.25V (WOT) - exactly half of APP1
#define APP1_IDLE_VOLTS             0.85f
#define APP1_WOT_VOLTS              4.50f

#define APP2_IDLE_VOLTS             0.425f
#define APP2_WOT_VOLTS              2.25f

#define DAC_VREF_VOLTS              5.00f // Reference voltage (vehicle 5V sensor rail or external LDO)

// Invert DAC channels (true: Channel A = APP2, Channel B = APP1; false: Channel A = APP1, Channel B = APP2)
#define DAC_INVERT_CHANNELS_A_B     true

// ==============================================================================
// 6. PID TUNING PARAMETERS
// ==============================================================================
#define PID_KP                      0.045f // Proportional gain
#define PID_KI                      0.012f // Integral gain
#define PID_KD                      0.005f // Derivative gain
#define PID_INTEGRAL_MAX            0.30f  // Anti-windup limit for integral term
#define PID_SAMPLE_TIME_MS          50     // 20 Hz control loop
