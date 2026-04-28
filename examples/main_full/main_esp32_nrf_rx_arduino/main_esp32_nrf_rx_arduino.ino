/**
 * ESP32 + nRF24L01 приёмник + веб: кадр TboyAirPkt 32 B (ver=2) с CubeSat main_full.
 *
 * Пины nRF (как в проекте): CE=GPIO2, CSN=GPIO4, SPI VSPI 18/19/23.
 *
 * Сборка: pio run -e esp32_nrf_rx
 * Заливка: pio run -e esp32_nrf_rx -t upload
 * После подключения к WiFi в Serial будет IP — открой в браузере http://IP/
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>

#include <stddef.h>
#include <stdint.h>

#define TBOY_AIR_VER 2
struct __attribute__((packed)) TboyAirPkt {
  uint8_t  ver;
  uint8_t  fix;
  uint8_t  sats;
  uint8_t  hdop_x10;
  int32_t  lat_e7;
  int32_t  lon_e7;
  uint32_t utc_hhmmss;
  int16_t  temp_c10;
  uint16_t press_hpa10;
  int16_t  alt_m;
  uint8_t  res[10];
};
static_assert(sizeof(TboyAirPkt) == 32, "TboyAirPkt");

// ---------- WiFi (замени при необходимости) ----------
static const char *WIFI_SSID = "KazybekAP";
static const char *WIFI_PASS = "kazybeek";

// ---------- nRF24 ----------
static const uint8_t PIN_NRF_CE  = 2;
static const uint8_t PIN_NRF_CS = 4;
/** Канал как на Uno main_full: setChannel(76) */
static const uint8_t NRF_CHANNEL = 76;
static const uint8_t NRF_PIPE[5] = {'T', 'B', 'O', 'Y', '1'};
static const uint32_t RX_STALE_MS = 2500;

static RF24 radio(PIN_NRF_CE, PIN_NRF_CS);
static WebServer server(80);

static TboyAirPkt lastPkt;
static uint32_t rxCount;
static uint32_t lastRxMs;
static bool everRx;

static void handleRoot();
static void handleData();

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== ESP32 nRF RX + Web (TboyAirPkt 32 B) ==="));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("WiFi "));
  uint8_t n = 0;
  while (WiFi.status() != WL_CONNECTED && n < 60) {
    delay(500);
    Serial.print('.');
    n++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("OK IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi FAIL — проверь SSID/пароль"));
  }

  SPI.begin();
  if (!radio.begin()) {
    Serial.println(F("nRF24: begin FAIL — проводка SPI/питание"));
  } else {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(NRF_CHANNEL);
    radio.setAutoAck(false);
    radio.setPayloadSize(sizeof(TboyAirPkt));
    radio.openReadingPipe(1, NRF_PIPE);
    radio.startListening();
    Serial.printf("nRF24: ch %u, pipe TBOY1, payload %u B (TboyAirPkt ver=%u)\n",
                  (unsigned)NRF_CHANNEL, (unsigned)sizeof(TboyAirPkt), (unsigned)TBOY_AIR_VER);
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println(F("HTTP :80"));
}

void loop() {
  server.handleClient();

  if (radio.available()) {
    TboyAirPkt p;
    radio.read(&p, sizeof(p));
    lastPkt = p;
    everRx = true;
    rxCount++;
    lastRxMs = millis();
    Serial.printf("[RX] #%lu ver=%u fix=%u sats=%u lat_e7=%ld lon_e7=%ld T*10=%d\n",
                  (unsigned long)rxCount, (unsigned)p.ver, (unsigned)p.fix, (unsigned)p.sats,
                  (long)p.lat_e7, (long)p.lon_e7, (int)p.temp_c10);
  }
}

static void handleRoot() {
  static const char PAGE[] = R"HTML(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>tenderboy RX</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#f4f4f5;color:#111}
h1{font-size:1.15rem;margin:0 0 8px;font-weight:600}
.sub{font-size:.8rem;color:#555;margin-bottom:12px}
#st{font-size:.9rem;margin-bottom:12px;padding:10px 12px;border-radius:8px;border:1px solid #ccc;background:#fff}
#st.ok{border-color:#2a7;border-left:4px solid #2a7}
#st.bad{border-color:#c33;border-left:4px solid #c33}
.page{max-width:1100px;margin:0 auto}
.grid{display:grid;gap:14px}
@media(min-width:820px){.grid{grid-template-columns:1fr 1fr}}
#map{height:300px;border-radius:8px;border:1px solid #ccc}
.mapHint{font-size:11px;color:#666;margin-top:6px}
table{border-collapse:collapse;width:100%;background:#fff;border:1px solid #ddd;font-size:13px}
td,th{border:1px solid #e0e0e0;padding:8px;text-align:left}
th{background:#eee}
.num{font-variant-numeric:tabular-nums;text-align:right}
.termRow{margin-top:16px}
.termRow h2{font-size:.95rem;margin:0 0 8px}
#term{display:block;width:100%;min-height:200px;height:260px;max-height:40vh;overflow:auto;padding:12px;background:#fff;border:1px solid #ccc;border-radius:8px;font:12px/1.45 ui-monospace,Consolas,monospace;white-space:pre-wrap;word-break:break-all}
</style>
</head>
<body>
<h1>tenderboy — приём nRF24 (CubeSat)</h1>
<p class="sub">Кадр <strong>32 байта</strong>, <code>ver=2</code> (<code>TboyAirPkt</code>), канал <strong>76</strong>, пайп <code>TBOY1</code>, 250 кбит/с, без ACK — как <code>main_full</code> на Uno.</p>
<div class="page">
<div id="st">загрузка…</div>
<div class="grid">
<div>
<table>
<tr><th>Поле</th><th>Значение</th></tr>
<tr><td>Эфир «живой»</td><td class="num" id="live">—</td></tr>
<tr><td>Возраст пакета, с</td><td class="num" id="age">—</td></tr>
<tr><td>Пакетов всего</td><td class="num" id="cnt">0</td></tr>
<tr><td>Версия кадра</td><td class="num" id="ver">—</td></tr>
<tr><td>GPS fix</td><td class="num" id="fix">—</td></tr>
<tr><td>Координаты</td><td class="num" id="ll">—</td></tr>
<tr><td>T °C (BME по эфиру)</td><td class="num" id="t">—</td></tr>
<tr><td>P гПа</td><td class="num" id="p">—</td></tr>
<tr><td>Влажность %</td><td class="num" id="h">—</td></tr>
<tr><td>Высота MSL м</td><td class="num" id="alt">—</td></tr>
<tr><td>Спутники</td><td class="num" id="sats">—</td></tr>
<tr><td>HDOP</td><td class="num" id="hdop">—</td></tr>
<tr><td>UTC (ЧЧММСС)</td><td class="num" id="utc">—</td></tr>
<tr><td>Канал nRF</td><td class="num" id="ch">—</td></tr>
</table>
</div>
<div>
<div id="map"></div>
<p class="mapHint" id="mapHint">Карта: при валидном фиксе и координатах — маркер. Иначе подсказка здесь.</p>
</div>
</div>
<div class="termRow">
<h2>Лог /data</h2>
<pre id="term"></pre>
</div>
</div>
<script>
const term=document.getElementById("term");
const TMAX=280;
function tlog(line){
  term.textContent+=line+"\n";
  const L=term.textContent.split("\n");
  if(L.length>TMAX)term.textContent=L.slice(-TMAX).join("\n");
  term.scrollTop=term.scrollHeight;
}
let map,mk;
function bootMap(lat,lon){
  if(map)return;
  map=L.map("map").setView([lat,lon],14);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",{maxZoom:19,attribution:"&copy; OSM"}).addTo(map);
  mk=L.marker([lat,lon]).addTo(map);
}
function fmtUtc(u){
  if(!u||u<100)return "—";
  const h=Math.floor(u/10000),m=Math.floor((u%10000)/100),s=u%100;
  return String(h).padStart(2,"0")+":"+String(m).padStart(2,"0")+":"+String(s).padStart(2,"0");
}
async function tick(){
  try{
    const r=await fetch("/data");
    const j=await r.json();
    const st=document.getElementById("st");
    const live=j.rx_live===true;
    const wifi=j.wifi||"";
    st.textContent=(wifi?("WiFi: "+wifi+" | "):"")+"Эфир: "+(live?("да (свежий пакет < "+j.stale_ms+" мс)"):"нет (нет нового кадра или TX выкл.)");
    st.className=live?"ok":"bad";
    document.getElementById("live").textContent=live?"да":"нет";
    document.getElementById("age").textContent=(j.age_s>=0)?j.age_s:"никогда";
    document.getElementById("cnt").textContent=j.cnt|0;
    document.getElementById("ver").textContent=j.ver;
    document.getElementById("fix").textContent=j.fix?"да":"нет";
    document.getElementById("t").textContent=j.temp_c;
    document.getElementById("p").textContent=j.press_hpa;
    document.getElementById("h").textContent=(j.humid===null||j.humid===undefined)?"— (нет в эфире)":j.humid;
    document.getElementById("alt").textContent=j.alt_m;
    document.getElementById("sats").textContent=j.sats;
    document.getElementById("hdop").textContent=j.hdop;
    document.getElementById("utc").textContent=fmtUtc(j.utc_t|0);
    document.getElementById("ch").textContent="ch="+j.ch+" — "+j.ch_note;
    const lat=j.lat,lng=j.lng;
    const hasFix=j.fix&&typeof lat==="number"&&typeof lng==="number"&&!isNaN(lat)&&!isNaN(lng)&&Math.abs(lat)<=90&&Math.abs(lng)<=180;
    document.getElementById("ll").textContent=hasFix?(lat.toFixed(6)+", "+lng.toFixed(6)):"нет фикса";
    const mh=document.getElementById("mapHint");
    if(hasFix){
      mh.textContent="OSM: маркер по последнему кадру.";
      if(!map)bootMap(lat,lng);
      else{if(!mk)mk=L.marker([lat,lng]).addTo(map);else{mk.setLatLng([lat,lng]);map.panTo([lat,lng]);}}
    }else{
      mh.textContent="Нет фикса или координат — маркер не ставится. Дождитесь GPS на передатчике.";
      if(map&&mk){map.removeLayer(mk);mk=null;}
    }
    tlog(new Date().toISOString()+" "+JSON.stringify(j));
  }catch(e){
    document.getElementById("st").textContent="Ошибка запроса /data";
    document.getElementById("st").className="bad";
    tlog(new Date().toISOString()+" ERROR "+e);
  }
}
setInterval(tick,500);
tick();
</script>
</body>
</html>
)HTML";

  server.send(200, "text/html; charset=utf-8", PAGE);
}

static void handleData() {
  char buf[1024];
  char ipstr[20] = "";
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.localIP().toString().toCharArray(ipstr, sizeof(ipstr));
  }

  const uint32_t now = millis();
  const bool rxLive = everRx && ((now - lastRxMs) <= RX_STALE_MS);
  const float ageS = everRx ? (now - lastRxMs) / 1000.0f : -1.0f;

  const TboyAirPkt &q = lastPkt;
  const double lat = (double)q.lat_e7 / 1e7;
  const double lng = (double)q.lon_e7 / 1e7;
  const float temp_c = q.temp_c10 / 10.0f;
  const float press_hpa = q.press_hpa10 / 10.0f;
  const float hdop = q.hdop_x10 / 10.0f;

  snprintf(buf, sizeof(buf),
           "{"
           "\"rx_live\":%s,"
           "\"age_s\":%.2f,"
           "\"stale_ms\":%u,"
           "\"cnt\":%lu,"
           "\"ch\":%u,"
           "\"ch_note\":\"RX=76, TX Uno main_full TboyAirPkt 32B\","
           "\"ver\":%u,"
           "\"fix\":%u,"
           "\"lat\":%.7f,"
           "\"lng\":%.7f,"
           "\"alt_m\":%d,"
           "\"sats\":%u,"
           "\"hdop\":%.1f,"
           "\"utc_t\":%lu,"
           "\"temp_c\":%.1f,"
           "\"press_hpa\":%.1f,"
           "\"humid\":null,"
           "\"wifi\":\"%s\""
           "}",
           rxLive ? "true" : "false",
           (double)ageS,
           (unsigned)RX_STALE_MS,
           (unsigned long)rxCount,
           (unsigned)NRF_CHANNEL,
           (unsigned)q.ver,
           (unsigned)q.fix,
           lat,
           lng,
           (int)q.alt_m,
           (unsigned)q.sats,
           (double)hdop,
           (unsigned long)q.utc_hhmmss,
           (double)temp_c,
           (double)press_hpa,
           ipstr);

  server.send(200, "application/json; charset=utf-8", buf);
}
