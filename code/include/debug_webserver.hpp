#ifndef DEBUG_WEBSERVER_H
#define DEBUG_WEBSERVER_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class DebugWebServer {
public:
  // Constructor; default port is 80.
  DebugWebServer(uint16_t port = 80);

  // Initializes the webserver and websocket.
  void begin();

  // Broadcasts a message to all connected websocket clients.
  void broadcast(const String &message);

private:
  AsyncWebServer server;
  AsyncWebSocket ws;

  // Internal websocket event handler.
  void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg,
                 uint8_t *data, size_t len);
};

#endif // DEBUG_WEBSERVER_H
