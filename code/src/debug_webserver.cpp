#include "debug_webserver.hpp"

DebugWebServer::DebugWebServer(uint16_t port) : server(port), ws("/ws") {}

void DebugWebServer::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  // For this example, we only broadcast debug messages.
  // You can extend this to handle incoming messages if needed.
  if (type == WS_EVT_CONNECT) {
    log_i("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    log_w("WebSocket client disconnected");
  }
}

void DebugWebServer::begin() {
  // Attach our event handler.
  ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                    void *arg, uint8_t *data,
                    size_t len) { this->onWsEvent(server, client, type, arg, data, len); });
  server.addHandler(&ws);

  // Serve a simple HTML page that displays debug logs.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <meta charset="utf-8">
    <title>ESP32 Debug Logs</title>
    <style>
      body { background: #000; color: #0f0; font-family: monospace; }
      #log { width: 100%; height: 90vh; overflow-y: auto; padding: 10px; }
    </style>
    <script>
      var gateway = `ws://${window.location.hostname}/ws`;
      var websocket;
      window.addEventListener('load', initWebSocket);
      function initWebSocket() {
        websocket = new WebSocket(gateway);
        websocket.onmessage = function(event) {
          var log = document.getElementById('log');
          log.innerHTML += event.data + "<br>";
          log.scrollTop = log.scrollHeight;
        };
        websocket.onclose = function(event) {
          setTimeout(initWebSocket, 2000);
        };
      }
    </script>
  </head>
  <body>
    <h1>ESP32 Debug Logs</h1>
    <div id="log"></div>
  </body>
</html>
    )rawliteral");
  });

  server.begin();
  Serial.println("Debug WebServer started");
}

void DebugWebServer::broadcast(const String &message) { ws.textAll(message); }
