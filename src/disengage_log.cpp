#include "disengage_log.h"

#define DISENGAGE_LOG_MAGIC 0x43525553 // 'CRUS'

DisengageLog::DisengageLog() {
    memset(&_history, 0, sizeof(_history));
}

void DisengageLog::begin() {
    _prefs.begin("cruise_log", false);

    size_t bytesRead = _prefs.getBytes("history", &_history, sizeof(DisengageHistory));
    if (bytesRead != sizeof(DisengageHistory) || _history.magic != DISENGAGE_LOG_MAGIC) {
        // First boot or corrupted partition - reinitialize clean struct
        memset(&_history, 0, sizeof(_history));
        _history.magic = DISENGAGE_LOG_MAGIC;
        _history.totalEvents = 0;
        _history.head = 0;
        _history.count = 0;
        save();
        Serial.println(F("[LOG] Memória NVS de diagnóstico inicializada limpa."));
    } else {
        Serial.printf("[LOG] Memória NVS carregada: %u eventos no histórico (Total acumulado: %u)\n", 
                      _history.count, _history.totalEvents);
    }
}

void DisengageLog::save() {
    _prefs.putBytes("history", &_history, sizeof(DisengageHistory));
}

void DisengageLog::clear() {
    memset(&_history, 0, sizeof(_history));
    _history.magic = DISENGAGE_LOG_MAGIC;
    _history.totalEvents = 0;
    _history.head = 0;
    _history.count = 0;
    save();
    Serial.println(F("[LOG] Histórico de desarmes da NVS apagado com sucesso."));
}

const char* DisengageLog::getReasonString(DisengageReason reason) {
    switch (reason) {
        case DisengageReason::BRAKE_PRESSED:   return "PÉ NO FREIO (Brake Pedal)";
        case DisengageReason::CLUTCH_PRESSED:  return "EMBREAGEM (Clutch)";
        case DisengageReason::CANCEL_BUTTON:   return "BOTÃO CANCEL";
        case DisengageReason::MAIN_SWITCH_OFF: return "CHAVE CRUISE MAIN DESLIGADA";
        case DisengageReason::SPEED_TOO_LOW:   return "VELOCIDADE ABAIXO DO MÍNIMO (<20 km/h)";
        case DisengageReason::SPEED_TOO_HIGH:  return "VELOCIDADE ACIMA DO MÁXIMO (>150 km/h)";
        case DisengageReason::CAN_TIMEOUT:     return "CAN TIMEOUT (Sem sinal de velocidade por 1s)";
        case DisengageReason::DRIVER_OVERRIDE: return "DRIVER OVERRIDE";
        case DisengageReason::SENSOR_FAULT:    return "FALHA DE SENSOR";
        default:                               return "OUTRO / DESCONHECIDO";
    }
}

void DisengageLog::logDisengage(DisengageReason reason, float currentSpeed, float targetSpeed, 
                                float pedalThrottle, float pedalVolts, float cruiseThrottle) {
    DisengageRecord rec;
    rec.uptimeSec = millis() / 1000;
    rec.reason = static_cast<uint8_t>(reason);
    rec.speedKmh = currentSpeed;
    rec.targetSpeedKmh = targetSpeed;
    rec.pedalThrottlePct = pedalThrottle * 100.0f;
    rec.pedalVolts = pedalVolts;
    rec.cruiseThrottlePct = cruiseThrottle * 100.0f;

    _history.records[_history.head] = rec;
    _history.head = (_history.head + 1) % MAX_DISENGAGE_LOG_ENTRIES;

    if (_history.count < MAX_DISENGAGE_LOG_ENTRIES) {
        _history.count++;
    }
    _history.totalEvents++;

    save();
    Serial.printf("[LOG] Desarme gravado na NVS (#%u) com sucesso!\n", _history.totalEvents);
}

void DisengageLog::printHistory() const {
    Serial.println(F("\n========================================================================================"));
    Serial.printf(  "            HISTÓRICO DOS ÚLTIMOS DESARMES SALVOS NA MEMÓRIA NVS (FLASH)                \n");
    Serial.printf(  "            Eventos salvos: %u/10  |  Total acumulado na vida útil: %u                \n", 
                    _history.count, _history.totalEvents);
    Serial.println(F("========================================================================================"));

    if (_history.count == 0) {
        Serial.println(F("  Nenhum desarme registrado na memória até o momento (Memória limpa)."));
        Serial.println(F("========================================================================================\n"));
        return;
    }

    for (uint8_t i = 0; i < _history.count; i++) {
        // Access circular buffer from newest to oldest
        int idx = (static_cast<int>(_history.head) - 1 - i + MAX_DISENGAGE_LOG_ENTRIES) % MAX_DISENGAGE_LOG_ENTRIES;
        const DisengageRecord &r = _history.records[idx];

        uint32_t sec = r.uptimeSec;
        uint32_t hrs = sec / 3600;
        uint32_t mins = (sec % 3600) / 60;
        uint32_t s = sec % 60;

        Serial.printf(" [%2u] Motivo: %-42s\n", i + 1, getReasonString(static_cast<DisengageReason>(r.reason)));
        Serial.printf("      Velocidade: %5.1f km/h  (Alvo: %5.1f km/h)  |  Thr Cruise: %4.1f%%\n", 
                      r.speedKmh, r.targetSpeedKmh, r.cruiseThrottlePct);
        Serial.printf("      Pedal Gás : %5.1f%% (%.2fV)                |  Uptime ESP32: %02uh %02um %02us\n",
                      r.pedalThrottlePct, r.pedalVolts, hrs, mins, s);
        Serial.println(F("----------------------------------------------------------------------------------------"));
    }
    Serial.println(F("  (Dica: Pressione 'd' a qualquer momento para ver este histórico, ou 'k' para limpar)\n"));
}
