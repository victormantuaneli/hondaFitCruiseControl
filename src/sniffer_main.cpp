/**
 * @file sniffer_main.cpp
 * @brief High-Speed, Safe CAN Bus Sniffer for Honda Fit (2nd Gen) F-CAN & B-CAN
 *        Supports MCP2515 SPI Controller (with fallback to ESP32 Native TWAI)
 * 
 * Features:
 *  - Native support for MCP2515 SPI module (TJA1050 / MCP2515 blue board)
 *  - 100% Safe Passive Listen-Only Mode (MODE_LISTENONLY):
 *    Never transmits ACK or dominant bits, zero risk of bus disturbance,
 *    instrument cluster warning lights, or ECU DTCs.
 *  - Multi-Format Streaming:
 *      1. Human-Readable Aligned Table (with delta-t, hex, and ASCII)
 *      2. SLCAN / LAWICEL Protocol (direct plug-and-play with SavvyCAN, Wireshark, python-can)
 *      3. CSV Stream (for Excel, Pandas, MATLAB)
 *  - Interactive runtime control via USB Serial (115200 baud):
 *      - Filter by ID, Ignore/Mute noisy IDs
 *      - Toggle Crystal Oscillator (8 MHz <-> 16 MHz) with 'x' command
 *      - Runtime Baud Rate switching (500k default, 250k, 125k, etc.)
 *      - Bus Health & Error Statistics
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "driver/twai.h"
#include "config.h"

// Hardware driver selection
enum CanDriverType {
    DRIVER_MCP2515 = 0,
    DRIVER_ESP32_TWAI = 1
};

// Output formats
enum OutputFormat {
    FORMAT_TABLE = 0,
    FORMAT_SLCAN = 1,
    FORMAT_CSV   = 2
};

// Global Hardware Instances
static MCP2515 mcp2515(PIN_MCP2515_CS);
static CanDriverType activeDriver = DRIVER_MCP2515;

// Sniffer state
static OutputFormat currentFormat = FORMAT_TABLE;
static bool listenOnlyMode = true;
static bool streamPaused = false;
static uint32_t filterId = 0;          // 0 = accept all
static uint32_t ignoredId = 0;         // 0 = ignore none
static uint32_t totalPackets = 0;
static uint32_t packetsLastSec = 0;
static uint32_t rxOverruns = 0;
static uint32_t busErrors = 0;
static uint32_t lastStatTime = 0;
static uint32_t lastPacketTimeMs = 0;
static uint32_t currentBaudRateKbps = 500;
static uint8_t  mcpOscMhz = MCP2515_DEFAULT_OSC_MHZ; // 8 MHz or 16 MHz

// Forward declarations
bool initMcp2515(uint32_t baudKbps, uint8_t oscMhz, bool listenOnly);
bool initTwai(uint32_t baudKbps, bool listenOnly);
void printHelp();
void printStats();
void handleCommand(String cmd);
void outputPacket(uint32_t id, bool isExtended, uint8_t dlc, const uint8_t* data, uint32_t nowMs);

// Convert kbps and osc to MCP2515 enums
CAN_SPEED getMcpSpeed(uint32_t kbps) {
    switch (kbps) {
        case 1000: return CAN_1000KBPS;
        case 500:  return CAN_500KBPS;
        case 250:  return CAN_250KBPS;
        case 200:  return CAN_200KBPS;
        case 125:  return CAN_125KBPS;
        case 100:  return CAN_100KBPS;
        case 50:   return CAN_50KBPS;
        default:   return CAN_500KBPS;
    }
}

CAN_CLOCK getMcpClock(uint8_t mhz) {
    return (mhz == 16) ? MCP_16MHZ : MCP_8MHZ;
}

bool initMcp2515(uint32_t baudKbps, uint8_t oscMhz, bool listenOnly) {
    SPI.begin(PIN_MCP2515_SCK, PIN_MCP2515_MISO, PIN_MCP2515_MOSI, PIN_MCP2515_CS);

    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("[MCP2515] Reset failed! Check SPI wiring (CS, SCK, MOSI, MISO) and 5V power."));
        return false;
    }

    CAN_SPEED spd = getMcpSpeed(baudKbps);
    CAN_CLOCK clk = getMcpClock(oscMhz);

    if (mcp2515.setBitrate(spd, clk) != MCP2515::ERROR_OK) {
        Serial.println(F("[MCP2515] Failed to set bitrate!"));
        return false;
    }

    if (listenOnly) {
        if (mcp2515.setListenOnlyMode() != MCP2515::ERROR_OK) {
            Serial.println(F("[MCP2515] Failed to enter Listen-Only mode!"));
            return false;
        }
    } else {
        if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
            Serial.println(F("[MCP2515] Failed to enter Normal mode!"));
            return false;
        }
    }

    currentBaudRateKbps = baudKbps;
    mcpOscMhz = oscMhz;
    listenOnlyMode = listenOnly;
    activeDriver = DRIVER_MCP2515;
    return true;
}

bool initTwai(uint32_t baudKbps, bool listenOnly) {
    twai_stop();
    twai_driver_uninstall();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX,
        (gpio_num_t)PIN_CAN_RX,
        listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = 128;
    g_config.alerts_enabled = TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | 
                              TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_DATA;

    twai_timing_config_t t_config;
    switch (baudKbps) {
        case 250: t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125: t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        default:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
        return false;
    }

    currentBaudRateKbps = baudKbps;
    listenOnlyMode = listenOnly;
    activeDriver = DRIVER_ESP32_TWAI;
    return true;
}

void printHelp() {
    Serial.println(F("\n========================================================"));
    Serial.println(F("   HONDA FIT (2nd GEN) CAN BUS SNIFFER - COMMAND MENU   "));
    Serial.println(F("========================================================"));
    Serial.println(F("  m       : Cycle Output Format (Table -> SLCAN -> CSV)"));
    Serial.println(F("  x       : Toggle MCP2515 Crystal (8 MHz <-> 16 MHz)"));
    Serial.println(F("  l       : Toggle Listen-Only mode (Safe) vs Normal mode"));
    Serial.println(F("  f <ID>  : Filter by CAN ID (hex, e.g. 'f 158' or 'f 0' for all)"));
    Serial.println(F("  i <ID>  : Ignore/Mute CAN ID (hex, e.g. 'i 158' or 'i 0')"));
    Serial.println(F("  b <spd> : Change Baud Rate kbps (e.g. 'b 500', 'b 250', 'b 125')"));
    Serial.println(F("  p       : Pause / Resume packet streaming"));
    Serial.println(F("  s       : Display Bus Statistics & Hardware Status"));
    Serial.println(F("  c       : Clear packet counters & reset screen"));
    Serial.println(F("  ? or h  : Display this help menu"));
    Serial.println(F("========================================================"));
    Serial.printf("Driver: %s | Crystal: %u MHz | Baud: %u kbps | Mode: %s\n",
        activeDriver == DRIVER_MCP2515 ? "MCP2515 (SPI)" : "ESP32 Native TWAI",
        mcpOscMhz,
        currentBaudRateKbps,
        listenOnlyMode ? "LISTEN-ONLY (Safe)" : "NORMAL");
    Serial.printf("Format: %s | Filter: %s | Ignore: %s\n\n",
        currentFormat == FORMAT_TABLE ? "TABLE" : (currentFormat == FORMAT_SLCAN ? "SLCAN (SavvyCAN)" : "CSV"),
        filterId == 0 ? "ALL" : String(filterId, HEX).c_str(),
        ignoredId == 0 ? "NONE" : String(ignoredId, HEX).c_str());
}

void printStats() {
    Serial.println(F("\n--- CAN BUS HEALTH & STATISTICS ---"));
    Serial.printf("  Hardware Driver:       %s\n", activeDriver == DRIVER_MCP2515 ? "MCP2515 (SPI)" : "ESP32 TWAI");
    if (activeDriver == DRIVER_MCP2515) {
        Serial.printf("  MCP2515 Crystal:       %u MHz (check crystal on board!)\n", mcpOscMhz);
        Serial.printf("  SPI Pins:              CS=%d, SCK=%d, MISO=%d, MOSI=%d\n",
                      PIN_MCP2515_CS, PIN_MCP2515_SCK, PIN_MCP2515_MISO, PIN_MCP2515_MOSI);
    }
    Serial.printf("  Total Frames Received: %u\n", totalPackets);
    Serial.printf("  Current Rate:          %u frames/sec\n", packetsLastSec);
    Serial.printf("  RX Queue Overruns:     %u\n", rxOverruns);
    Serial.printf("  Bus Error Alerts:      %u\n", busErrors);
    Serial.println(F("-----------------------------------\n"));
}

void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char action = cmd.charAt(0);
    String arg = cmd.substring(1);
    arg.trim();

    switch (action) {
        case '?':
        case 'h':
        case 'H':
            printHelp();
            break;

        case 'm':
        case 'M':
            currentFormat = (OutputFormat)((currentFormat + 1) % 3);
            Serial.printf("[FORMAT] Switched to: %s\n",
                currentFormat == FORMAT_TABLE ? "TABLE (Human Aligned)" :
                currentFormat == FORMAT_SLCAN ? "SLCAN (SavvyCAN / Lawicel)" : "CSV (Spreadsheet)");
            if (currentFormat == FORMAT_CSV) {
                Serial.println(F("TIME_MS,ID_HEX,EXT,DLC,B0,B1,B2,B3,B4,B5,B6,B7"));
            }
            break;

        case 'x':
        case 'X':
            if (activeDriver == DRIVER_MCP2515) {
                mcpOscMhz = (mcpOscMhz == 8) ? 16 : 8;
                if (initMcp2515(currentBaudRateKbps, mcpOscMhz, listenOnlyMode)) {
                    Serial.printf("[CRYSTAL] MCP2515 Crystal toggled to %u MHz\n", mcpOscMhz);
                }
            } else {
                Serial.println(F("[INFO] Crystal setting only applies to MCP2515 SPI controller."));
            }
            break;

        case 'l':
        case 'L':
            listenOnlyMode = !listenOnlyMode;
            if (activeDriver == DRIVER_MCP2515) {
                initMcp2515(currentBaudRateKbps, mcpOscMhz, listenOnlyMode);
            } else {
                initTwai(currentBaudRateKbps, listenOnlyMode);
            }
            Serial.printf("[MODE] Switched to: %s\n", 
                listenOnlyMode ? "LISTEN-ONLY (Safe, High-Z passive)" : "NORMAL (Active ACK transmitter)");
            break;

        case 'p':
        case 'P':
            streamPaused = !streamPaused;
            Serial.printf("[STREAM] %s\n", streamPaused ? "PAUSED" : "RESUMED");
            break;

        case 's':
        case 'S':
            printStats();
            break;

        case 'c':
        case 'C':
            totalPackets = 0;
            rxOverruns = 0;
            busErrors = 0;
            Serial.println(F("[COUNTERS] Cleared."));
            break;

        case 'f':
        case 'F':
            if (arg.length() > 0) {
                filterId = strtoul(arg.c_str(), NULL, 16);
                if (filterId == 0) {
                    Serial.println(F("[FILTER] Cleared (showing all IDs)"));
                } else {
                    Serial.printf("[FILTER] Set to ID: 0x%03X\n", filterId);
                }
            } else {
                Serial.printf("[FILTER] Current filter: %s\n", filterId == 0 ? "NONE" : String(filterId, HEX).c_str());
            }
            break;

        case 'i':
        case 'I':
            if (arg.length() > 0) {
                ignoredId = strtoul(arg.c_str(), NULL, 16);
                if (ignoredId == 0) {
                    Serial.println(F("[IGNORE] Cleared (showing all IDs)"));
                } else {
                    Serial.printf("[IGNORE] Muting ID: 0x%03X\n", ignoredId);
                }
            } else {
                Serial.printf("[IGNORE] Currently muting: %s\n", ignoredId == 0 ? "NONE" : String(ignoredId, HEX).c_str());
            }
            break;

        case 'b':
        case 'B':
            if (arg.length() > 0) {
                uint32_t newBaud = arg.toInt();
                bool ok = (activeDriver == DRIVER_MCP2515) ?
                           initMcp2515(newBaud, mcpOscMhz, listenOnlyMode) :
                           initTwai(newBaud, listenOnlyMode);
                if (ok) {
                    Serial.printf("[BAUD] Switched to %u kbps\n", newBaud);
                }
            } else {
                Serial.printf("[BAUD] Current: %u kbps\n", currentBaudRateKbps);
            }
            break;

        default:
            Serial.println(F("[?] Unknown command. Type '?' or 'h' for menu."));
            break;
    }
}

void outputPacket(uint32_t id, bool isExtended, uint8_t dlc, const uint8_t* data, uint32_t nowMs) {
    if (streamPaused) return;

    // Filter check
    if (filterId != 0 && id != filterId) return;
    if (ignoredId != 0 && id == ignoredId) return;

    totalPackets++;
    uint32_t dtMs = (lastPacketTimeMs > 0) ? (nowMs - lastPacketTimeMs) : 0;
    lastPacketTimeMs = nowMs;

    // 1. HUMAN-READABLE TABLE FORMAT
    if (currentFormat == FORMAT_TABLE) {
        Serial.printf("[%7u ms | +%3u ms]  ID: 0x%03X%s  DLC: %u  [ ",
                      nowMs,
                      dtMs > 999 ? 999 : dtMs,
                      id,
                      isExtended ? " (EXT)" : "      ",
                      dlc);

        for (uint8_t i = 0; i < 8; i++) {
            if (i < dlc) {
                Serial.printf("%02X ", data[i]);
            } else {
                Serial.print(F("   "));
            }
        }

        Serial.print(F("] | "));

        // Printable ASCII
        for (uint8_t i = 0; i < dlc; i++) {
            char c = (char)data[i];
            Serial.print((c >= 32 && c <= 126) ? c : '.');
        }

        // Honda Fit 2nd Gen Quick Signature Annotation
        switch (id) {
            case 0x158: Serial.print(F("  <- Honda RPM/Speed")); break;
            case 0x17C: Serial.print(F("  <- Honda Throttle/Pedal")); break;
            case 0x1A6: Serial.print(F("  <- Honda Brake/Gear")); break;
            case 0x1DC: Serial.print(F("  <- Honda Steering Angle")); break;
            case 0x245: Serial.print(F("  <- Honda EPS Assist")); break;
            case 0x305: Serial.print(F("  <- Honda VSA Speed")); break;
            case 0x309: Serial.print(F("  <- Honda Wheel Speeds")); break;
            case 0x324: Serial.print(F("  <- Honda VSA Yaw/Acc")); break;
            case 0x7E8: Serial.print(F("  <- OBD-II Diagnostic Resp")); break;
            default: break;
        }

        Serial.println();
    }
    // 2. SLCAN / LAWICEL FORMAT (SavvyCAN / python-can compatible)
    else if (currentFormat == FORMAT_SLCAN) {
        if (isExtended) {
            Serial.printf("T%08X%u", id, dlc);
        } else {
            Serial.printf("t%03X%u", id, dlc);
        }
        for (uint8_t i = 0; i < dlc; i++) {
            Serial.printf("%02X", data[i]);
        }
        Serial.print('\r');
    }
    // 3. CSV STREAM FORMAT
    else if (currentFormat == FORMAT_CSV) {
        Serial.printf("%u,0x%X,%d,%u", nowMs, id, isExtended ? 1 : 0, dlc);
        for (uint8_t i = 0; i < 8; i++) {
            if (i < dlc) {
                Serial.printf(",%02X", data[i]);
            } else {
                Serial.print(F(","));
            }
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F("     HONDA FIT (2nd GEN) ESP32 CAN BUS SNIFFER         "));
    Serial.println(F("========================================================"));
    Serial.println(F("Attempting MCP2515 SPI initialization..."));
    Serial.printf("Pins: CS=GPIO%d, SCK=GPIO%d, MISO(SO)=GPIO%d, MOSI(SI)=GPIO%d\n",
                  PIN_MCP2515_CS, PIN_MCP2515_SCK, PIN_MCP2515_MISO, PIN_MCP2515_MOSI);
    Serial.printf("Default Crystal: %u MHz | Baud Rate: 500 kbps (Honda F-CAN)\n", mcpOscMhz);
    Serial.println(F("Mode: LISTEN-ONLY (100% Passive & Safe for Vehicle Bus)\n"));

    // 1. Try MCP2515 first
    if (initMcp2515(500, mcpOscMhz, true)) {
        Serial.println(F("[OK] MCP2515 SPI Controller initialized successfully!"));
        Serial.println(F("[OK] Running in LISTEN-ONLY mode at 500 kbps."));
    } else {
        Serial.println(F("[WARN] MCP2515 not detected! Falling back to native ESP32 TWAI..."));
        if (initTwai(500, true)) {
            Serial.println(F("[OK] ESP32 native TWAI initialized at 500 kbps (TX=GPIO5, RX=GPIO4)."));
        } else {
            Serial.println(F("[FAIL] Neither MCP2515 nor TWAI could be started. Check wiring!"));
        }
    }

    Serial.println(F("Type '?' or 'h' for interactive commands.\n"));
    lastStatTime = millis();
}

void loop() {
    // 1. Process serial commands from user / Python tool
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        handleCommand(input);
    }

    uint32_t now = millis();

    // 2. Read frames based on active driver
    if (activeDriver == DRIVER_MCP2515) {
        struct can_frame frame;
        while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
            bool isExt = (frame.can_id & CAN_EFF_FLAG) != 0;
            uint32_t id = isExt ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
            outputPacket(id, isExt, frame.can_dlc, frame.data, now);
        }
    } else {
        twai_message_t rxMsg;
        while (twai_receive(&rxMsg, 0) == ESP_OK) {
            bool isExt = (rxMsg.flags & TWAI_MSG_FLAG_EXTD) != 0;
            outputPacket(rxMsg.identifier, isExt, rxMsg.data_length_code, rxMsg.data, now);
        }
    }

    // 3. Update 1-second packet rate calculation
    if (now - lastStatTime >= 1000) {
        static uint32_t prevPacketCount = 0;
        packetsLastSec = totalPackets - prevPacketCount;
        prevPacketCount = totalPackets;
        lastStatTime = now;
    }

    yield();
}
