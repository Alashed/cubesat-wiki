#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>

// nRF24 (как в main_esp32_nrf_rx): CE=2, CSN=4, VSPI 18/19/23
const uint8_t NRF_CE = 2;
const uint8_t NRF_CSN = 4;
const byte NRF_PIPE[6] = "TBOY1";

RF24 radio(NRF_CE, NRF_CSN);
WebServer server(80);

struct __attribute__((packed)) Telemetry {
  uint32_t ms;
  int16_t temp_c10;
  uint16_t press_hpa10;
  uint8_t hum_x2;
  uint16_t eco2_ppm;
  uint16_t tvoc_ppb;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint8_t flags; // bit0=BME, bit1=CCS, bit2=MPU
};

Telemetry lastPkt = {};
bool hasPacket = false;
uint32_t rxCount = 0;
uint32_t lastRxMillis = 0;

static void handleRoot() {
  String html =
    "<!doctype html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>tenderboy base station</title>"
    "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f6f8fa;color:#1f2328}"
    ".card{background:#fff;border:1px solid #d0d7de;border-radius:10px;padding:14px;max-width:700px}"
    "h1{margin:0 0 10px}.muted{color:#59636e}pre{background:#f6f8fa;padding:10px;border-radius:8px;border:1px solid #d0d7de}"
    "</style></head><body><div class='card'>"
    "<h1>Base station ESP32</h1>"
    "<div class='muted'>nRF24 ch=76, pipe=TBOY1</div>"
    "<pre id='out'>waiting...</pre>"
    "</div><script>"
    "async function tick(){"
    "try{const r=await fetch('/api'); const j=await r.json();"
    "document.getElementById('out').textContent=JSON.stringify(j,null,2);}catch(e){"
    "document.getElementById('out').textContent='api error';}"
    "setTimeout(tick,1000);}"
    "tick();"
    "</script></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleApi() {
  String json = "{";
  json += "\"rx_count\":" + String(rxCount) + ",";
  json += "\"last_rx_ms_ago\":" + String(hasPacket ? (millis() - lastRxMillis) : 0) + ",";
  json += "\"has_packet\":" + String(hasPacket ? "true" : "false") + ",";

  json += "\"bme_ok\":" + String((lastPkt.flags & 0x01) ? "true" : "false") + ",";
  json += "\"ccs_ok\":" + String((lastPkt.flags & 0x02) ? "true" : "false") + ",";
  json += "\"mpu_ok\":" + String((lastPkt.flags & 0x04) ? "true" : "false") + ",";

  json += "\"temp_c\":" + String(lastPkt.temp_c10 / 10.0f, 1) + ",";
  json += "\"press_hpa\":" + String(lastPkt.press_hpa10 / 10.0f, 1) + ",";
  json += "\"hum_pct\":" + String(lastPkt.hum_x2 / 2.0f, 1) + ",";
  json += "\"eco2_ppm\":" + String(lastPkt.eco2_ppm) + ",";
  json += "\"tvoc_ppb\":" + String(lastPkt.tvoc_ppb) + ",";

  json += "\"ax\":" + String(lastPkt.ax) + ",";
  json += "\"ay\":" + String(lastPkt.ay) + ",";
  json += "\"az\":" + String(lastPkt.az) + ",";
  json += "\"gx\":" + String(lastPkt.gx) + ",";
  json += "\"gy\":" + String(lastPkt.gy) + ",";
  json += "\"gz\":" + String(lastPkt.gz);
  json += "}";

  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("tenderboy-base", "12345678");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (!radio.begin()) {
    Serial.println("nRF24 begin FAILED");
  } else {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setAutoAck(false);
    radio.setPayloadSize(sizeof(Telemetry));
    radio.openReadingPipe(1, NRF_PIPE);
    radio.startListening();
    Serial.println("nRF24 RX ready");
  }

  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.begin();
  Serial.println("Web server ready");
}

void loop() {
  server.handleClient();

  while (radio.available()) {
    radio.read(&lastPkt, sizeof(lastPkt));
    hasPacket = true;
    rxCount++;
    lastRxMillis = millis();
  }
}
