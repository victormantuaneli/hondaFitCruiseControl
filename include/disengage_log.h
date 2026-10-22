#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "safety_supervisor.h"

#define MAX_DISENGAGE_LOG_ENTRIES 10

struct DisengageRecord {
    uint32_t uptimeSec;        // Uptime in seconds when disengage occurred
    uint8_t  reason;           // DisengageReason enum value
    float    speedKmh;         // Vehicle speed at disengage
    float    targetSpeedKmh;   // Target set speed
    float    pedalThrottlePct; // Physical driver pedal throttle percentage (0..100)
    float    pedalVolts;       // Physical pedal ADC voltage
    float    cruiseThrottlePct;// Cruise PID throttle demand percentage (0..100)
};

struct DisengageHistory {
    uint32_t magic;            // Validation magic number
    uint32_t totalEvents;      // Lifetime disengage events
    uint8_t  head;             // Circular buffer insertion index (0..9)
    uint8_t  count;            // Number of valid records stored (0..10)
    DisengageRecord records[MAX_DISENGAGE_LOG_ENTRIES];
};

class DisengageLog {
public:
    DisengageLog();

    void begin();
    void logDisengage(DisengageReason reason, float currentSpeed, float targetSpeed, 
                      float pedalThrottle, float pedalVolts, float cruiseThrottle);
    void printHistory() const;
    void clear();

    uint8_t getCount() const { return _history.count; }
    uint32_t getTotalEvents() const { return _history.totalEvents; }

    static const char* getReasonString(DisengageReason reason);

private:
    Preferences _prefs;
    DisengageHistory _history;
    void save();
};
