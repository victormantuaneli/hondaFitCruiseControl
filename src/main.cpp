#include <Arduino.h>
#include "config.h"
#include "can_speed.h"
#include "steering_switch.h"
#include "mcp4728_dac.h"
#include "safety_supervisor.h"
#include "cruise_controller.h"

// Hardware Singletons
CanSpeedReader      canReader;
SteeringSwitchDecoder switchDecoder;
Mcp4728Dac          dacOutput;
SafetySupervisor    safetySupervisor;
CruiseController    cruiseController(switchDecoder, canReader, dacOutput, safetySupervisor);

// FreeRTOS Task Handles
TaskHandle_t hTaskCan = NULL;
TaskHandle_t hTaskControl = NULL;

// Fast CAN RX Task (runs on Core 0)
void taskCanRx(void *pvParameters) {
    for (;;) {
        canReader.update();
        safetySupervisor.setCanBrakePressed(canReader.isBrakePressed());
        switchDecoder.setCanInputs(canReader.isCruiseMainActive(), canReader.getPressedButton());
        vTaskDelay(pdMS_TO_TICKS(1)); // Poll every 1ms for fast sub-millisecond response
    }
}

// Deterministic Control Loop Task (runs on Core 1 at 20Hz / 50ms)
void taskControlLoop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PID_SAMPLE_TIME_MS);

    for (;;) {
        // Run cruise control state machine and PID
        cruiseController.update();

        // Wait for next 50ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("\n============================================="));
    Serial.println(F("   ESP32 Cruise Control System Starting...   "));
    Serial.println(F("============================================="));

    // 1. Initialize Safety Supervisor first (keeps relay de-energized)
    safetySupervisor.begin();
    Serial.println(F("[OK] Safety Supervisor initialized."));

    // 2. Initialize Steering Switch Decoder
    switchDecoder.begin();
    Serial.println(F("[OK] Steering switch decoder initialized."));

    // 3. Initialize MCP4728 12-bit DAC
    if (!dacOutput.begin()) {
        Serial.println(F("[OK] DAC set to bench/simulation mode."));
    }

    // 4. Initialize CAN Bus (TWAI) at 500 kbps
    if (!canReader.begin()) {
        Serial.println(F("[OK] CAN set to bench/simulation mode (70 km/h virtual speed)."));
    }

    // 5. Initialize Controller
    cruiseController.begin();
    Serial.println(F("[OK] Cruise controller initialized."));

    // 6. Launch FreeRTOS Tasks
    xTaskCreatePinnedToCore(
        taskCanRx,
        "TaskCAN",
        4096,
        NULL,
        2,              // High priority
        &hTaskCan,
        0               // Core 0
    );

    xTaskCreatePinnedToCore(
        taskControlLoop,
        "TaskControl",
        4096,
        NULL,
        3,              // Higher priority
        &hTaskControl,
        1               // Core 1
    );

    Serial.println(F("[OK] System running. Entering telemetry loop...\n"));
}

void handleSerialDebugCommand(char c) {
    static float simSpeed = 70.0f;
    switch (c) {
        case 'm':
        case 'M':
            canReader.toggleVirtualMainSwitch();
            switchDecoder.toggleVirtualMainSwitch();
            break;
        case 's':
            Serial.println(F("[DEBUG] Injected: SET/DECEL Tap"));
            switchDecoder.injectVirtualEvent(SteeringButton::SET_DECEL, ButtonEvent::SHORT_PRESS);
            break;
        case 'S':
            Serial.println(F("[DEBUG] Injected: SET/DECEL Hold"));
            switchDecoder.injectVirtualEvent(SteeringButton::SET_DECEL, ButtonEvent::LONG_PRESS_HOLD);
            break;
        case 'r':
            Serial.println(F("[DEBUG] Injected: RES/ACCEL Tap"));
            switchDecoder.injectVirtualEvent(SteeringButton::RES_ACCEL, ButtonEvent::SHORT_PRESS);
            break;
        case 'R':
            Serial.println(F("[DEBUG] Injected: RES/ACCEL Hold"));
            switchDecoder.injectVirtualEvent(SteeringButton::RES_ACCEL, ButtonEvent::LONG_PRESS_HOLD);
            break;
        case 'c':
        case 'C':
            Serial.println(F("[DEBUG] Injected: CANCEL"));
            switchDecoder.injectVirtualEvent(SteeringButton::CANCEL, ButtonEvent::SHORT_PRESS);
            break;
        case 'b':
        case 'B':
            safetySupervisor.triggerSimulatedBrake(500);
            break;
        case 'o':
        case 'O':
            safetySupervisor.toggleSimulatedOverride();
            break;
        case '+':
            simSpeed += 5.0f;
            if (simSpeed > 160.0f) simSpeed = 160.0f;
            canReader.setSimulatedSpeed(simSpeed, true);
            Serial.printf("[DEBUG] Injected Speed: %.1f km/h\n", simSpeed);
            break;
        case '-':
            simSpeed -= 5.0f;
            if (simSpeed < 0.0f) simSpeed = 0.0f;
            canReader.setSimulatedSpeed(simSpeed, true);
            Serial.printf("[DEBUG] Injected Speed: %.1f km/h\n", simSpeed);
            break;
        case 't':
        case 'T':
            canReader.setSimulatedSpeed(simSpeed, false);
            Serial.println(F("[DEBUG] Injected: CAN Timeout"));
            break;
        case 'x':
        case 'X':
            canReader.toggleSnifferOutput();
            break;
        case 'l':
        case 'L':
            canReader.toggleManualLampTest();
            break;
        case 'i':
        case 'I':
            Serial.println(F("\n[I2C] Executando diagnóstico I2C..."));
            dacOutput.scanBus();
            {
                uint8_t err = dacOutput.pingDevice();
                if (err == 0) {
                    Serial.println(F("[DAC] MCP4728 respondeu com ACK (Sucesso)!"));
                } else {
                    Serial.printf("[DAC] MCP4728 NÃO respondeu! Código de erro Wire: %d\n", err);
                    Serial.println(F("       (2=NACK endereço, 3=NACK dados, 4/5=Barramento travado/sem pull-up)"));
                }
            }
            break;
        case '?':
        case 'h':
            Serial.println(F("\n=== BENCH DEBUG COMMANDS ==="));
            Serial.println(F("  m : Alternar chave Cruise Main"));
            Serial.println(F("  s : Pressionar SET/DECEL"));
            Serial.println(F("  r : Pressionar RES/ACCEL"));
            Serial.println(F("  c : Pressionar CANCEL"));
            Serial.println(F("  b : Teste de Freio (pulso 500ms)"));
            Serial.println(F("  o : Alternar Override do acelerador"));
            Serial.println(F("  l : Teste Manual da Luz CRUISE no Painel (0x17C)"));
            Serial.println(F("  i : Diagnóstico I2C / Testar MCP4728"));
            Serial.println(F("  + : Velocidade +5 km/h (simulado)"));
            Serial.println(F("  - : Velocidade -5 km/h (simulado)"));
            Serial.println(F("  t : Simular Timeout de CAN"));
            Serial.println(F("  x : Alternar Sniffer CAN (OFF -> FILTRADO -> TODOS RAW)"));
            Serial.println(F("============================\n"));
            break;
        default:
            break;
    }
}

void loop() {
    // Process interactive debug commands from USB Serial
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\r' && c != '\n') {
            handleSerialDebugCommand(c);
        }
    }

    static uint32_t lastTelemetry = 0;
    uint32_t now = millis();

    // 5Hz Telemetry output - Using '\r' without '\n' to keep the terminal static (in-place dashboard)
    if (now - lastTelemetry >= 200) {
        lastTelemetry = now;

        Serial.printf(
            "\r\033[KState: %-7s | Spd: %4.1f/%-4.1f km/h | Thr:%4.1f%% | Ped:%4.1f%% | APP: %.2f/%.2fV | DAC:%-2s | CAN:%-3s | Brk:%-3s | Lamp:%-3s",
            cruiseController.getStateString(),
            canReader.getSpeedKmh(),
            cruiseController.getTargetSpeed(),
            cruiseController.getCurrentThrottle() * 100.0f,
            safetySupervisor.readPhysicalPedalThrottle() * 100.0f,
            dacOutput.getVoutA(),
            dacOutput.getVoutB(),
            dacOutput.isDeviceFound() ? "OK" : "ERR",
            canReader.isSpeedValid() ? "OK" : "ERR",
            safetySupervisor.isBrakePressed() ? "ON" : "OFF",
            canReader.isCruiseLampOn() ? "ON" : "OFF"
        );
    }

    delay(20);
}
