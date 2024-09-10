#ifdef WEBSOCKET
void notifyClients() {
  ws.textAll("notify");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      Serial.printf("handleWSmessage: %s\n", (char *)data);
      notifyClients();
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
  Serial.printf("ws client #%u\n", client->id());
}

void initWebSocket() {
  if (serverStarted) {
    ws.onEvent(onEvent);
    server.addHandler(&ws);
  } else Serial.printf("initWebSocket: server not started\n");
}

void loopWS() {
  const char message[]{"Hello from compass"};
  Serial.printf("WS: send %s\n", message);
  ws.textAll(message);
  ws.cleanupClients();
}
#endif