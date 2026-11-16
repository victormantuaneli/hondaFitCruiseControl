#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include "config.h"

// Forward declarations
class CruiseController;
class CanSpeedReader;
class SafetySupervisor;

class OtaManager {
public:
    OtaManager();

    // Initialize Wi-Fi (AP + optional STA), WebServer and ArduinoOTA
    void begin(CruiseController *cruise, CanSpeedReader *can, SafetySupervisor *safety);

    // Call regularly in main loop
    void update();

    // Check if system is currently safe to accept firmware upload
    bool isSafeToUpdate() const;

    // Returns local IP string (AP or STA)
    String getIpAddress() const;

    // Check if firmware update is currently in progress
    bool isUpdating() const { return _isUpdating; }

private:
    CruiseController *_cruise;
    CanSpeedReader   *_can;
    SafetySupervisor *_safety;

    WebServer _server;
    bool _isUpdating;
    uint32_t _lastDnsCheck;
    bool _staConnected;

    void setupWebServer();
    void setupArduinoOta();
    void handleRoot();
    void handleStatusJson();
};
