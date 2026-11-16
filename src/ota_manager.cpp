#include "ota_manager.h"
#include "cruise_controller.h"
#include "can_speed.h"
#include "safety_supervisor.h"

// Embedded HTML Dashboard & OTA Upload UI
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32 Cruise Control - Painel & OTA</title>
  <style>
    :root {
      --bg: #121418;
      --card-bg: #1e222b;
      --accent: #00bcd4;
      --accent-hover: #00acc1;
      --text: #e0e6ed;
      --text-muted: #8c9ba5;
      --danger: #ff5252;
      --success: #4caf50;
      --warning: #ffb74d;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background-color: var(--bg); color: var(--text); padding: 16px; max-width: 600px; margin: 0 auto; }
    h1 { font-size: 1.5rem; text-align: center; margin-bottom: 4px; color: #fff; }
    p.subtitle { text-align: center; color: var(--text-muted); font-size: 0.85rem; margin-bottom: 20px; }
    .card { background: var(--card-bg); border-radius: 12px; padding: 18px; margin-bottom: 16px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); border: 1px solid #2d3340; }
    .card h2 { font-size: 1.1rem; margin-bottom: 14px; border-bottom: 1px solid #2d3340; padding-bottom: 8px; color: var(--accent); }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .metric { background: #15181f; padding: 12px; border-radius: 8px; text-align: center; border: 1px solid #252b36; }
    .metric .label { font-size: 0.75rem; color: var(--text-muted); text-transform: uppercase; margin-bottom: 4px; }
    .metric .val { font-size: 1.35rem; font-weight: bold; color: #fff; }
    .badge { display: inline-block; padding: 4px 10px; border-radius: 20px; font-size: 0.8rem; font-weight: bold; }
    .badge-active { background: rgba(76,175,80,0.2); color: var(--success); border: 1px solid var(--success); }
    .badge-standby { background: rgba(255,183,77,0.2); color: var(--warning); border: 1px solid var(--warning); }
    .badge-off { background: rgba(140,155,165,0.2); color: var(--text-muted); border: 1px solid var(--text-muted); }
    .badge-danger { background: rgba(255,82,82,0.2); color: var(--danger); border: 1px solid var(--danger); }
    .alert { padding: 12px; border-radius: 8px; font-size: 0.85rem; margin-bottom: 16px; line-height: 1.4; display: none; }
    .alert-danger { background: rgba(255,82,82,0.15); border: 1px solid var(--danger); color: #ff8a80; }
    .alert-success { background: rgba(76,175,80,0.15); border: 1px solid var(--success); color: #b9f6ca; }
    .form-group { margin-bottom: 16px; }
    label { display: block; font-size: 0.85rem; margin-bottom: 6px; color: var(--text-muted); }
    input[type="file"] { width: 100%; background: #15181f; padding: 10px; border-radius: 8px; border: 1px dashed #3a4252; color: var(--text); font-size: 0.85rem; }
    input[type="file"]:disabled { opacity: 0.5; cursor: not-allowed; }
    button.btn { width: 100%; padding: 14px; background: var(--accent); color: #000; border: none; border-radius: 8px; font-size: 1rem; font-weight: bold; cursor: pointer; transition: 0.2s; }
    button.btn:hover:not(:disabled) { background: var(--accent-hover); }
    button.btn:disabled { background: #333a47; color: #667285; cursor: not-allowed; }
    .progress-bar-container { width: 100%; height: 12px; background: #15181f; border-radius: 6px; overflow: hidden; margin-top: 14px; display: none; border: 1px solid #2a313e; }
    .progress-bar { width: 0%; height: 100%; background: var(--accent); transition: width 0.1s ease; }
    #progressText { text-align: center; font-size: 0.8rem; margin-top: 6px; color: var(--text-muted); display: none; }
    footer { text-align: center; font-size: 0.75rem; color: #5a6675; margin-top: 24px; }
  </style>
</head>
<body>
  <h1>Cruise Control ESP32</h1>
  <p class="subtitle">Honda Fit / City Drive-by-Wire &bull; Atualização Remota OTA</p>

  <div class="card">
    <h2>Status em Tempo Real</h2>
    <div class="grid">
      <div class="metric">
        <div class="label">Estado</div>
        <div id="mState" class="val"><span class="badge badge-off">...</span></div>
      </div>
      <div class="metric">
        <div class="label">Velocidade Atual</div>
        <div id="mSpeed" class="val">-- km/h</div>
      </div>
      <div class="metric">
        <div class="label">Velocidade Alvo</div>
        <div id="mTarget" class="val">-- km/h</div>
      </div>
      <div class="metric">
        <div class="label">Acelerador (DAC)</div>
        <div id="mThrottle" class="val">-- %</div>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>Atualização de Firmware (OTA)</h2>
    <div id="alertBox" class="alert"></div>

    <form id="uploadForm">
      <div class="form-group">
        <label for="fwFile">Selecione o arquivo firmware.bin compilado:</label>
        <input type="file" id="fwFile" name="update" accept=".bin" required>
      </div>
      <button type="submit" id="btnUpload" class="btn">Atualizar Firmware</button>
      <div id="progressContainer" class="progress-bar-container">
        <div id="progressBar" class="progress-bar"></div>
      </div>
      <div id="progressText">0%</div>
    </form>
  </div>

  <footer>ESP32 Cruise Control &bull; Travamento de Segurança Ativo em Movimento</footer>

  <script>
    let isUploading = false;

    function fetchStatus() {
      if (isUploading) return;
      fetch('/api/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('mSpeed').innerText = data.speed.toFixed(1) + ' km/h';
          document.getElementById('mTarget').innerText = data.target.toFixed(1) + ' km/h';
          document.getElementById('mThrottle').innerText = (data.throttle * 100).toFixed(1) + ' %';
          
          let stateEl = document.getElementById('mState');
          let badgeClass = 'badge-off';
          if (data.state === 'ACTIVE') badgeClass = 'badge-active';
          else if (data.state === 'STANDBY') badgeClass = 'badge-standby';
          else if (data.state === 'FAULT') badgeClass = 'badge-danger';
          stateEl.innerHTML = `<span class="badge ${badgeClass}">${data.state}</span>`;

          let alertBox = document.getElementById('alertBox');
          let fileInput = document.getElementById('fwFile');
          let btnUpload = document.getElementById('btnUpload');

          alertBox.className = 'alert alert-success';
          alertBox.style.display = 'block';
          alertBox.innerHTML = '✅ <b>Pronto para Atualização.</b> Selecione o arquivo .bin abaixo.';
          fileInput.disabled = false;
          btnUpload.disabled = false;
        })
        .catch(err => console.log('Erro ao buscar status', err));
    }

    setInterval(fetchStatus, 1000);
    fetchStatus();

    document.getElementById('uploadForm').addEventListener('submit', function(e) {
      e.preventDefault();
      let fileInput = document.getElementById('fwFile');
      if (fileInput.files.length === 0) {
        alert('Por favor selecione um arquivo .bin');
        return;
      }

      if (!confirm('Deseja realmente iniciar a gravação do novo firmware?')) return;

      isUploading = true;
      let file = fileInput.files[0];
      let formData = new FormData();
      formData.append('update', file);

      let pContainer = document.getElementById('progressContainer');
      let pBar = document.getElementById('progressBar');
      let pText = document.getElementById('progressText');
      let btnUpload = document.getElementById('btnUpload');
      let alertBox = document.getElementById('alertBox');

      pContainer.style.display = 'block';
      pText.style.display = 'block';
      btnUpload.disabled = true;
      fileInput.disabled = true;

      let xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);

      xhr.upload.onprogress = function(e) {
        if (e.lengthComputable) {
          let pct = Math.round((e.loaded / e.total) * 100);
          pBar.style.width = pct + '%';
          pText.innerText = 'Enviando firmware: ' + pct + '%';
        }
      };

      xhr.onload = function() {
        if (xhr.status === 200) {
          alertBox.className = 'alert alert-success';
          alertBox.style.display = 'block';
          alertBox.innerHTML = '🎉 <b>Atualização Concluída com Sucesso!</b><br>O ESP32 está reiniciando agora. Reconecte em 10 segundos.';
          pText.innerText = 'Concluído! Reiniciando...';
        } else {
          alertBox.className = 'alert alert-danger';
          alertBox.style.display = 'block';
          alertBox.innerHTML = '❌ <b>Falha na Atualização:</b> ' + xhr.responseText;
          btnUpload.disabled = false;
          fileInput.disabled = false;
          isUploading = false;
        }
      };

      xhr.onerror = function() {
        alertBox.className = 'alert alert-danger';
        alertBox.style.display = 'block';
        alertBox.innerHTML = '❌ Erro de conexão durante o upload.';
        btnUpload.disabled = false;
        fileInput.disabled = false;
        isUploading = false;
      };

      xhr.send(formData);
    });
  </script>
</body>
</html>
)rawliteral";

OtaManager::OtaManager()
    : _cruise(nullptr),
      _can(nullptr),
      _safety(nullptr),
      _server(OTA_WEB_PORT),
      _isUpdating(false),
      _lastDnsCheck(0),
      _staConnected(false) {}

void OtaManager::begin(CruiseController *cruise, CanSpeedReader *can, SafetySupervisor *safety) {
    _cruise = cruise;
    _can = can;
    _safety = safety;

    Serial.println(F("\n[OTA] Inicializando subsistema Wi-Fi & OTA..."));

    // Set Wi-Fi mode: SoftAP (hotspot) always available for in-car connection,
    // plus STA if configured to connect to home Wi-Fi when in garage.
    WiFi.mode(WIFI_AP_STA);

    // 1. SoftAP Setup
    WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD);
    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("[OTA] SoftAP Ativo: SSID '%s' | Senha: '%s'\n", OTA_AP_SSID, OTA_AP_PASSWORD);
    Serial.printf("[OTA] Endereço Web SoftAP: http://%s/\n", apIP.toString().c_str());

    // 2. Optional Station mode (home Wi-Fi)
    #ifdef OTA_STA_SSID
    if (strlen(OTA_STA_SSID) > 0) {
        Serial.printf("[OTA] Conectando à rede Wi-Fi '%s'...\n", OTA_STA_SSID);
        WiFi.begin(OTA_STA_SSID, OTA_STA_PASSWORD);
        uint32_t startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < 3000) {
            delay(100);
        }
        if (WiFi.status() == WL_CONNECTED) {
            _staConnected = true;
            Serial.printf("[OTA] Conectado ao Wi-Fi residencial! IP: http://%s/\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println(F("[OTA] Wi-Fi residencial indisponível. Continuando via SoftAP do carro."));
        }
    }
    #endif

    // 3. WebServer Setup
    setupWebServer();

    // 4. ArduinoOTA Setup (for direct IDE / PlatformIO flashing)
    setupArduinoOta();
}

bool OtaManager::isSafeToUpdate() const {
    return true; // Trava desabilitada a pedido do usuario
}

String OtaManager::getIpAddress() const {
    if (_staConnected && WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    }
    return WiFi.softAPIP().toString();
}

void OtaManager::setupWebServer() {
    // 1. Dashboard UI
    _server.on("/", HTTP_GET, [this]() {
        handleRoot();
    });

    _server.on("/update", HTTP_GET, [this]() {
        handleRoot();
    });

    // 2. Real-time Status API
    _server.on("/api/status", HTTP_GET, [this]() {
        handleStatusJson();
    });

    // 3. Firmware Upload Endpoint (multipart upload)
    _server.on("/update", HTTP_POST, [this]() {
        if (!_isUpdating) {
            _server.send(400, "text/plain", "Nenhum arquivo enviado.");
            return;
        }

        if (Update.hasError()) {
            _server.send(500, "text/plain", "Falha na gravação do firmware!");
        } else {
            _server.send(200, "text/plain", "Atualização realizada com sucesso! Reiniciando...");
            delay(500);
            ESP.restart();
        }
    }, [this]() {
        // Upload handler callback
        HTTPUpload& upload = _server.upload();

        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("\n[OTA] Recebendo firmware: %s\n", upload.filename.c_str());

            // Immediately disconnect cruise control relay for safety
            if (_safety) {
                _safety->setRelayEngaged(false);
            }

            _isUpdating = true;

            // Start ESP32 flash write
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (_isUpdating) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("[OTA] Sucesso: %u bytes gravados na Flash!\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
            _isUpdating = false;
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
            Update.end();
            _isUpdating = false;
            Serial.println(F("[OTA] Gravação abortada pelo cliente."));
        }
    });

    _server.begin();
    Serial.println(F("[OTA] Servidor Web ativo na porta 80."));
}

void OtaManager::setupArduinoOta() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.setPort(OTA_PORT);

    ArduinoOTA.onStart([this]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        Serial.printf("[ArduinoOTA] Início da atualização de %s\n", type.c_str());

        // Disengage relay
        if (_safety) {
            _safety->setRelayEngaged(false);
        }
        _isUpdating = true;
    });

    ArduinoOTA.onEnd([this]() {
        Serial.println(F("\n[ArduinoOTA] Atualização concluída com sucesso!"));
        _isUpdating = false;
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[ArduinoOTA] Progresso: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[ArduinoOTA] Erro [%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println(F("Falha de autenticação"));
        else if (error == OTA_BEGIN_ERROR) Serial.println(F("Falha na inicialização"));
        else if (error == OTA_CONNECT_ERROR) Serial.println(F("Falha na conexão"));
        else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Falha na recepção"));
        else if (error == OTA_END_ERROR) Serial.println(F("Falha no encerramento"));
    });

    ArduinoOTA.begin();
    Serial.printf("[OTA] ArduinoOTA pronto no hostname '%s' porta %d.\n", OTA_HOSTNAME, OTA_PORT);
}

void OtaManager::handleRoot() {
    _server.send_P(200, "text/html", INDEX_HTML);
}

void OtaManager::handleStatusJson() {
    float speed = _can ? _can->getSpeedKmh() : 0.0f;
    float target = _cruise ? _cruise->getTargetSpeed() : 0.0f;
    float throttle = _cruise ? _cruise->getCurrentThrottle() : 0.0f;
    const char *state = _cruise ? _cruise->getStateString() : "OFF";
    bool safe = isSafeToUpdate();

    char jsonBuf[256];
    snprintf(jsonBuf, sizeof(jsonBuf),
        "{\"state\":\"%s\",\"speed\":%.1f,\"target\":%.1f,\"throttle\":%.3f,\"safeToUpdate\":%s}",
        state, speed, target, throttle, safe ? "true" : "false");

    _server.send(200, "application/json", jsonBuf);
}

void OtaManager::update() {
    _server.handleClient();
    ArduinoOTA.handle();
}
