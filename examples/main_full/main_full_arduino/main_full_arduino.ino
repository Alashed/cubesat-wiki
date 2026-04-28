/**
 * tenderboy — полный цикл:
 *   MPU-9250/9255 (аксель/гиро) + BME280 + CCS811 + GPS NMEA
 *   → CSV на SD + эфир TboyAirPkt 32 B по nRF24 раз в 1 с; зуммер D3 — раз в минуту;
 *   лента WS2812: как main_gps_nrf_tx — min(sats,8) пикселей цветом по HDOP, остальное чёрное; sats=0 — всё чёрное;
 *   при старте: самотест (лента белая) — nRF / GPS / SD: 1 писк=OK, 2=нет; затем 3 писка «готово».
 *
 * SPI: MOSI=D11, MISO=D12, SCK=D13  (470Ω на MISO SD — обязательно при SD+nRF)
 * I²C: A4=SDA, A5=SCL
 * GPS NEO-6M: TX модуля → D0 (RX0), 9600 — один UART с Serial (без SoftwareSerial, меньше Flash).
 * Монитор: 9600; подробный лог в Serial — только сборка с TBOY_SERIAL_DIAG=1 (env full_diag).
 *
 * Сборка: pio run -e full -t upload
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <string.h>

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

#ifndef TBOY_SERIAL_DIAG
#define TBOY_SERIAL_DIAG 0
#endif

/* ---- пины ---- */
static const uint8_t PIN_SD_CS   = 4;
static const uint8_t PIN_NRF_CE  = 9;
static const uint8_t PIN_NRF_CS  = 10;
static const uint8_t PIN_LED     = 6;   // WS2812, 8 пикселей
static const uint8_t PIN_BUZZ    = 3;   // активный, LOW = звук

static const uint8_t NUM_LEDS    = 8;
static const uint8_t BME_ADDR    = 0x76;
static const uint8_t CCS_ADDR    = 0x5A;

static const uint32_t DT_LED_MS   = 200;      // обновление ленты по GPS
static const uint32_t DT_LOG_MS   = 1000;     // 1 Гц: датчики + SD + nRF
static const uint32_t DT_BEEP_MS  = 60000UL;  // зуммер: раз в минуту

static const char LOG_FILE[] = "TBOY.CSV";
static const uint8_t NRF_PIPE[5] = {'T', 'B', 'O', 'Y', '1'};

/**
 * Пакет для SD (только лог CSV). По эфиру — TboyAirPkt 32 B.
 */
struct __attribute__((packed)) Packet {
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  int16_t  temp_c10;    // °C × 10
  uint16_t press_hpa10; // гПа × 10
  uint8_t  humid_x2;    // % × 2
  uint16_t co2_ppm;
  uint16_t tvoc_ppb;
};

static_assert(sizeof(Packet) == 21, "SD row binary snapshot");

static Packet pkt;
static TboyAirPkt air;
static RF24       radio(PIN_NRF_CE, PIN_NRF_CS);
static TinyGPSPlus gps;

static uint32_t tLed  = 0;
static uint32_t tLog  = 0;
static uint32_t tBeep = 0;

static uint8_t  mpuAddr = 0;
static uint8_t  mpuWhoAmI = 0;
static bool okMpu = false, okBme = false, okCcs = false;
static bool okSd  = false, okNrf = false;

static uint16_t lastCo2 = 400, lastTvoc = 0;

/* ================================================================== */
/* I²C вспомогательные                                                 */
/* ================================================================== */
static uint8_t i2cR8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}
static void i2cRN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, n);
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.available() ? Wire.read() : 0;
}
static void i2cW(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

/* ================================================================== */
/* MPU                                                                 */
/* ================================================================== */
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuInit() {
  const uint8_t addrs[] = {0x68, 0x69};
  uint8_t who = 0;
  mpuAddr = 0;
  for (uint8_t i = 0; i < 2; i++) {
    who = i2cR8(addrs[i], 0x75);
    if (who == 0x71 || who == 0x73 || who == 0x68 || who == 0x70) {
      mpuAddr = addrs[i];
      break;
    }
  }
  if (!mpuAddr) return false;

  mpuWhoAmI = who;
  mpuWrite(0x6B, 0x80);
  delay(100);
  mpuWrite(0x6B, 0x01);
  delay(50);
  mpuWrite(0x1C, 0x00);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1A, 0x03);   // DLPF ~41 Гц (как в тесте MPU-9250)
  return true;
}

static void mpuRead() {
  if (!okMpu) return;
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return;
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
  pkt.ax = (int16_t)((b[0]  << 8) | b[1]);
  pkt.ay = (int16_t)((b[2]  << 8) | b[3]);
  pkt.az = (int16_t)((b[4]  << 8) | b[5]);
  pkt.gx = (int16_t)((b[8]  << 8) | b[9]);
  pkt.gy = (int16_t)((b[10] << 8) | b[11]);
  pkt.gz = (int16_t)((b[12] << 8) | b[13]);
}

/* ================================================================== */
/* BME280                                                              */
/* ================================================================== */
static struct {
  uint16_t T1;
  int16_t T2, T3;
  uint16_t P1;
  int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  uint8_t H1, H3;
  int16_t H2, H4, H5;
  int8_t H6;
} bCal;
static int32_t bTFine;

static bool bmeInit() {
  if (i2cR8(BME_ADDR, 0xD0) != 0x60) return false;
  i2cW(BME_ADDR, 0xE0, 0xB6);
  delay(15);
  uint8_t t[24];
  i2cRN(BME_ADDR, 0x88, t, 24);
  bCal.T1 = (uint16_t)(t[1] << 8 | t[0]);
  bCal.T2 = (int16_t)(t[3] << 8 | t[2]);
  bCal.T3 = (int16_t)(t[5] << 8 | t[4]);
  bCal.P1 = (uint16_t)(t[7] << 8 | t[6]);
  bCal.P2 = (int16_t)(t[9] << 8 | t[8]);
  bCal.P3 = (int16_t)(t[11] << 8 | t[10]);
  bCal.P4 = (int16_t)(t[13] << 8 | t[12]);
  bCal.P5 = (int16_t)(t[15] << 8 | t[14]);
  bCal.P6 = (int16_t)(t[17] << 8 | t[16]);
  bCal.P7 = (int16_t)(t[19] << 8 | t[18]);
  bCal.P8 = (int16_t)(t[21] << 8 | t[20]);
  bCal.P9 = (int16_t)(t[23] << 8 | t[22]);
  bCal.H1 = i2cR8(BME_ADDR, 0xA1);
  uint8_t h[7];
  i2cRN(BME_ADDR, 0xE1, h, 7);
  bCal.H2 = (int16_t)(h[1] << 8 | h[0]);
  bCal.H3 = h[2];
  bCal.H4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
  bCal.H5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
  bCal.H6 = (int8_t)h[6];
  i2cW(BME_ADDR, 0xF2, 0x01);
  i2cW(BME_ADDR, 0xF4, 0x27);
  i2cW(BME_ADDR, 0xF5, 0xA0);
  delay(100);
  return true;
}

static void bmeReadToPacket() {
  if (!okBme) {
    pkt.temp_c10 = 0;
    pkt.press_hpa10 = 0;
    pkt.humid_x2 = 0;
    return;
  }
  uint8_t b[8];
  i2cRN(BME_ADDR, 0xF7, b, 8);
  int32_t rP = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  int32_t rT = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
  int32_t rH = ((int32_t)b[6] << 8) | b[7];

  int32_t v1 = (((rT >> 3) - ((int32_t)bCal.T1 << 1)) * (int32_t)bCal.T2) >> 11;
  int32_t v2 = ((((rT >> 4) - (int32_t)bCal.T1) * ((rT >> 4) - (int32_t)bCal.T1)) >> 12);
  v2 = (v2 * (int32_t)bCal.T3) >> 14;
  bTFine = v1 + v2;
  float T = ((bTFine * 5 + 128) >> 8) / 100.0f;

  float P = 0;
  int64_t w1 = (int64_t)bTFine - 128000;
  int64_t w2 = w1 * w1 * (int64_t)bCal.P6 + ((w1 * (int64_t)bCal.P5) << 17) + (((int64_t)bCal.P4) << 35);
  w1 = ((w1 * w1 * (int64_t)bCal.P3) >> 8) + ((w1 * (int64_t)bCal.P2) << 12);
  w1 = ((((int64_t)1 << 47) + w1) * (int64_t)bCal.P1) >> 33;
  if (w1) {
    int64_t p = 1048576 - rP;
    p = (((p << 31) - w2) * 3125) / w1;
    w1 = ((int64_t)bCal.P9 * (p >> 13) * (p >> 13)) >> 25;
    w2 = ((int64_t)bCal.P8 * p) >> 19;
    p = ((p + w1 + w2) >> 8) + ((int64_t)bCal.P7 << 4);
    P = (float)((uint32_t)p) / 25600.0f;
  }

  int32_t x = bTFine - 76800;
  x = ((((rH << 14) - ((int32_t)bCal.H4 << 20) - ((int32_t)bCal.H5 * x)) + 16384) >> 15) *
      (((((((x * (int32_t)bCal.H6) >> 10) * (((x * (int32_t)bCal.H3) >> 11) + 32768)) >> 10) + 2097152) *
        (int32_t)bCal.H2 + 8192) >>
       14);
  x -= (((((x >> 15) * (x >> 15)) >> 7) * (int32_t)bCal.H1) >> 4);
  if (x < 0) x = 0;
  if (x > 419430400) x = 419430400;
  float H = (float)((uint32_t)(x >> 12)) / 1024.0f;

  int16_t tc = (int16_t)(T * 10.0f + (T >= 0 ? 0.5f : -0.5f));
  if (tc < -300) tc = -300;
  if (tc > 1200) tc = 1200;
  pkt.temp_c10 = tc;
  uint32_t ph = (uint32_t)(P * 10.0f + 0.5f);
  if (ph > 65535) ph = 65535;
  pkt.press_hpa10 = (uint16_t)ph;
  uint8_t hx = (uint8_t)(H * 2.0f + 0.5f);
  if (hx > 200) hx = 200;
  pkt.humid_x2 = hx;
}

/* ================================================================== */
/* CCS811                                                              */
/* ================================================================== */
static bool ccsInit() {
  if (i2cR8(CCS_ADDR, 0x20) != 0x81) return false;
  Wire.beginTransmission(CCS_ADDR);
  Wire.write(0xF4);
  Wire.endTransmission();
  delay(100);
  if (!(i2cR8(CCS_ADDR, 0x00) & 0x80)) return false;
  i2cW(CCS_ADDR, 0x01, 0x10);
  return true;
}

static void ccsSetEnv(float temp, float humid) {
  uint16_t h = (uint16_t)(humid * 512.0f + 0.5f);
  uint16_t t = (uint16_t)((temp + 25.0f) * 512.0f + 0.5f);
  Wire.beginTransmission(CCS_ADDR);
  Wire.write(0x05);
  Wire.write(h >> 8);
  Wire.write(h & 0xFF);
  Wire.write(t >> 8);
  Wire.write(t & 0xFF);
  Wire.endTransmission();
}

static void ccsReadToPacket() {
  if (!okCcs) {
    pkt.co2_ppm = lastCo2;
    pkt.tvoc_ppb = lastTvoc;
    return;
  }
  if (!(i2cR8(CCS_ADDR, 0x00) & 0x08)) {
    pkt.co2_ppm = lastCo2;
    pkt.tvoc_ppb = lastTvoc;
    return;
  }
  Wire.beginTransmission(CCS_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CCS_ADDR, (uint8_t)4);
  uint8_t b[4] = {};
  for (uint8_t i = 0; i < 4; i++)
    if (Wire.available()) b[i] = Wire.read();
  lastCo2 = (uint16_t)((b[0] << 8) | b[1]);
  lastTvoc = (uint16_t)((b[2] << 8) | b[3]);
  pkt.co2_ppm = lastCo2;
  pkt.tvoc_ppb = lastTvoc;
}

/* ================================================================== */
/* WS2812 (D6 = PD6)                                                    */
/* ================================================================== */
static inline void ws2812Byte(uint8_t b) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (b & mask) {
      PORTD |= _BV(PIN_LED);
      __asm__ volatile("nop\nnop\nnop\nnop\nnop\nnop\n");
      PORTD &= ~_BV(PIN_LED);
      __asm__ volatile("nop\n");
    } else {
      PORTD |= _BV(PIN_LED);
      __asm__ volatile("nop\n");
      PORTD &= ~_BV(PIN_LED);
      __asm__ volatile("nop\nnop\nnop\nnop\nnop\n");
    }
  }
}

static inline void ws2812Pixel(uint8_t r, uint8_t g, uint8_t b) {
  ws2812Byte(g);
  ws2812Byte(r);
  ws2812Byte(b);
}

/** HDOP×10: меньше — лучше. 0 — нет данных (тускло-синий). */
/** Все пиксели одним цветом (WS2812, D6 = PD6). */
static void ws2812FillSolid(uint8_t r, uint8_t g, uint8_t b) {
  noInterrupts();
  for (uint16_t i = 0; i < NUM_LEDS; i++)
    ws2812Pixel(r, g, b);
  interrupts();
  delayMicroseconds(60);
}

static void hdopToRgb(uint8_t hdop_x10, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (hdop_x10 == 0) {
    r = 0;
    g = 0;
    b = 48;
    return;
  }
  if (hdop_x10 >= 50) {
    r = 220;
    g = 0;
    b = 0;
    return;
  }
  if (hdop_x10 >= 38) {
    r = 255;
    g = 40;
    b = 0;
    return;
  }
  if (hdop_x10 >= 30) {
    r = 255;
    g = 120;
    b = 0;
    return;
  }
  if (hdop_x10 >= 25) {
    r = 255;
    g = 200;
    b = 0;
    return;
  }
  if (hdop_x10 >= 20) {
    r = 120;
    g = 255;
    b = 0;
    return;
  }
  r = 0;
  g = 220;
  b = 30;
}

static void refreshLedStripFromGps() {
  uint8_t sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  uint8_t hx = 0;
  if (gps.hdop.isValid()) {
    double h = gps.hdop.hdop();
    if (h > 25.5)
      h = 25.5;
    hx = (uint8_t)(h * 10.0 + 0.5);
  }
  uint8_t r, g, b;
  hdopToRgb(hx, r, g, b);

  uint16_t n = sats;
  if (n > NUM_LEDS)
    n = NUM_LEDS;
  noInterrupts();
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if (i < n)
      ws2812Pixel(r, g, b);
    else
      ws2812Pixel(0, 0, 0);
  }
  interrupts();
  delayMicroseconds(60);
}

/* ================================================================== */
/* GPS: во время длинного setup UART буфер 64 B — без encode() теряется NMEA. */
/* ================================================================== */
static inline void pollGps() {
  while (Serial.available())
    gps.encode(Serial.read());
}

static void delayPollGps(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    pollGps();
    delay(3);
  }
}

/* ================================================================== */
/* Зуммер: активный, «инверсия» к тишине — LOW = звук, HIGH = выкл.   */
/* Во время писка крутим pollGps — иначе теряется поток с модуля.     */
/* ================================================================== */
static void beep(uint16_t ms = 80) {
  uint32_t t0 = millis();
  digitalWrite(PIN_BUZZ, LOW);
  while ((uint32_t)(millis() - t0) < (uint32_t)ms) {
    pollGps();
    delay(2);
  }
  digitalWrite(PIN_BUZZ, HIGH);
  t0 = millis();
  while ((uint32_t)(millis() - t0) < 20u) {
    pollGps();
    delay(2);
  }
}

static void selfTestBuzzOk() {
  beep(85);
}

static void selfTestBuzzFail() {
  beep(60);
  delayPollGps(55);
  beep(60);
}

static void selfTestBuzzTripleDone() {
  for (uint8_t i = 0; i < 3; i++) {
    beep(52);
    delayPollGps(88);
  }
}

/** Есть валидная NMEA-строка с CRC за timeout (поток с GPS на Serial). */
static bool selfTestGpsNmea(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeout_ms) {
    pollGps();
    if (gps.passedChecksum() >= 1)
      return true;
    delay(3);
  }
  return false;
}

/** Пауза между этапами самотеста (секунда) + опрос GPS. */
static const uint32_t ST_GAP_MS = 1000;
/** Холодный старт GPS после setup — дольше, чем в коротком gps_nrf_tx. */
static const uint32_t ST_GPS_NMEA_MS = 8000;

/**
 * Самотест при каждом включении: лента на полную белую на всё время.
 * Порядок: nRF → GPS → SD: один писк = OK, два = нет.
 * Затем три писка — самотест окончен.
 */
static void runPowerOnSelfTest() {
  delayPollGps(800);

  ws2812FillSolid(255, 255, 255);

  if (okNrf)
    selfTestBuzzOk();
  else
    selfTestBuzzFail();
  delayPollGps(ST_GAP_MS);

  bool gpsOk = selfTestGpsNmea(ST_GPS_NMEA_MS);
  if (gpsOk)
    selfTestBuzzOk();
  else
    selfTestBuzzFail();
  delayPollGps(ST_GAP_MS);

  if (okSd)
    selfTestBuzzOk();
  else
    selfTestBuzzFail();
  delayPollGps(ST_GAP_MS);

  selfTestBuzzTripleDone();

  ws2812FillSolid(0, 0, 0);
}

/* ================================================================== */
/* SD                                                                   */
/* ================================================================== */
static void sdRelease() {
  digitalWrite(PIN_SD_CS, HIGH);
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  SPI.transfer(0xFF);
  SPI.endTransaction();
}

static void sdWriteHeader() {
  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) return;
  f.println(F("ms,ax,ay,az,gx,gy,gz,T_C,P_hPa,H_%,CO2,TVOC,"
              "gps_fix,gps_sats,gps_hdop,lat,lon,gps_alt_m,utc_hhmmss,gps_date_ddmmyy,"
              "gps_chars,gps_crc_ok,gps_crc_bad,gps_sents_w_fix"));
  f.close();
  sdRelease();
}

static void sdLogRow() {
  if (!okSd) return;
  digitalWrite(PIN_NRF_CS, HIGH);

  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) {
#if TBOY_SERIAL_DIAG
    // Serial.println(F("SD: open fail"));
#endif
    sdRelease();
    return;
  }

  char buf[12];
  char lbuf[16];
  f.print(millis());
  f.print(',');
  f.print(pkt.ax);
  f.print(',');
  f.print(pkt.ay);
  f.print(',');
  f.print(pkt.az);
  f.print(',');
  f.print(pkt.gx);
  f.print(',');
  f.print(pkt.gy);
  f.print(',');
  f.print(pkt.gz);
  f.print(',');
  dtostrf(pkt.temp_c10 / 10.0f, 1, 1, buf);
  f.print(buf);
  f.print(',');
  dtostrf(pkt.press_hpa10 / 10.0f, 1, 2, buf);
  f.print(buf);
  f.print(',');
  dtostrf(pkt.humid_x2 / 2.0f, 1, 1, buf);
  f.print(buf);
  f.print(',');
  f.print(pkt.co2_ppm);
  f.print(',');
  f.print(pkt.tvoc_ppb);
  f.print(',');
  f.print(gps.location.isValid() ? 1 : 0);
  f.print(',');
  f.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  f.print(',');
  if (gps.hdop.isValid()) {
    dtostrf(gps.hdop.hdop(), 1, 1, buf);
    f.print(buf);
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.location.isValid()) {
    dtostrf(gps.location.lat(), 2, 6, lbuf);
    f.print(lbuf);
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.location.isValid()) {
    dtostrf(gps.location.lng(), 2, 6, lbuf);
    f.print(lbuf);
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.altitude.isValid()) {
    dtostrf(gps.altitude.meters(), 1, 1, lbuf);
    f.print(lbuf);
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.time.isValid()) {
    f.print((unsigned long)(gps.time.hour() * 10000UL + gps.time.minute() * 100UL + gps.time.second()));
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.date.isValid()) {
    f.print((unsigned long)(gps.date.day() * 1000000UL + gps.date.month() * 10000UL + gps.date.year()));
  } else {
    f.print('0');
  }
  f.print(',');
  f.print((unsigned long)gps.charsProcessed());
  f.print(',');
  f.print((unsigned long)gps.passedChecksum());
  f.print(',');
  f.print((unsigned long)gps.failedChecksum());
  f.print(',');
  f.print((unsigned long)gps.sentencesWithFix());
  f.println();

  f.close();
  sdRelease();
}

/* ================================================================== */
/* Эфир TboyAirPkt (32 B)                                               */
/* ================================================================== */
static void fillTboyAirPkt() {
  memset(&air, 0, sizeof(air));
  air.ver = TBOY_AIR_VER;
  air.temp_c10 = pkt.temp_c10;
  air.press_hpa10 = pkt.press_hpa10;

  if (gps.location.isValid()) {
    air.fix = 1;
    double la = gps.location.lat();
    double lo = gps.location.lng();
    air.lat_e7 = (int32_t)(la * 1e7 + (la >= 0 ? 0.5 : -0.5));
    air.lon_e7 = (int32_t)(lo * 1e7 + (lo >= 0 ? 0.5 : -0.5));
  }

  if (gps.satellites.isValid())
    air.sats = (uint8_t)gps.satellites.value();

  if (gps.hdop.isValid()) {
    double h = gps.hdop.hdop();
    if (h > 25.5)
      h = 25.5;
    air.hdop_x10 = (uint8_t)(h * 10.0 + 0.5);
  }

  if (gps.time.isValid())
    air.utc_hhmmss = (uint32_t)gps.time.hour() * 10000UL + (uint32_t)gps.time.minute() * 100UL +
                     (uint32_t)gps.time.second();

  if (air.fix && gps.altitude.isValid()) {
    double m = gps.altitude.meters();
    int32_t im = (int32_t)(m + (m >= 0 ? 0.5 : -0.5));
    if (im > 32767)
      im = 32767;
    if (im < -32768)
      im = -32768;
    air.alt_m = (int16_t)im;
  } else {
    air.alt_m = 0;
  }
}

/* ================================================================== */
/* nRF24                                                                */
/* ================================================================== */
static void nrfSend() {
  if (!okNrf) return;
  digitalWrite(PIN_SD_CS, HIGH);
  radio.write(&air, sizeof(air));
}

/* ================================================================== */
static void printRow() {
#if TBOY_SERIAL_DIAG
  // char buf[10];
  // Serial.print(F("ax="));
  // Serial.print(pkt.ax);
  // Serial.print(F(" ay="));
  // Serial.print(pkt.ay);
  // Serial.print(F(" az="));
  // Serial.print(pkt.az);
  // Serial.print(F(" | T="));
  // Serial.print(dtostrf(pkt.temp_c10 / 10.0f, 1, 1, buf));
  // Serial.print(F(" P="));
  // Serial.print(dtostrf(pkt.press_hpa10 / 10.0f, 1, 1, buf));
  // Serial.print(F(" CO2="));
  // Serial.print(pkt.co2_ppm);
  // Serial.print(F(" | SD="));
  // Serial.print(okSd ? F("OK") : F("--"));
  // Serial.print(F(" nRF="));
  // Serial.print(okNrf ? F("OK") : F("--"));
  // Serial.print(F(" | GPS fix="));
  // Serial.print(gps.location.isValid() ? 1 : 0);
  // Serial.print(F(" sats="));
  // Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  // if (gps.location.isValid()) {
  //   char la[16], lo[16];
  //   dtostrf(gps.location.lat(), 2, 5, la);
  //   dtostrf(gps.location.lng(), 2, 5, lo);
  //   Serial.print(F(" lat="));
  //   Serial.print(la);
  //   Serial.print(F(" lon="));
  //   Serial.println(lo);
  // } else {
  //   Serial.println();
  // }
#endif
}

static void sampleAllAndLog() {
  mpuRead();
  bmeReadToPacket();
  if (okBme && okCcs)
    ccsSetEnv(pkt.temp_c10 / 10.0f, pkt.humid_x2 / 2.0f);
  ccsReadToPacket();

  sdLogRow();
  fillTboyAirPkt();
  nrfSend();
  printRow();
}

void setup() {
  Serial.begin(9600);
#if TBOY_SERIAL_DIAG
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  // Serial.println(F("\n=== tenderboy full: MPU+BME+CCS+GPS → SD + TboyAirPkt 32B nRF; бип 1/мин; LED sats/HDOP ==="));
#endif

  pinMode(PIN_LED, OUTPUT);
  TCCR2A = 0;
  TCCR2B = 0;
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, HIGH);
  delayPollGps(50);

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_NRF_CS, OUTPUT);
  digitalWrite(PIN_NRF_CS, HIGH);
  SPI.begin();

  Wire.begin();
  Wire.setClock(400000);
  delayPollGps(50);

  okMpu = mpuInit();
#if TBOY_SERIAL_DIAG
  // Serial.print(F("MPU:  "));
  // if (okMpu) {
  //   Serial.print(F("OK WHO=0x"));
  //   Serial.println(mpuWhoAmI, HEX);
  // } else {
  //   Serial.println(F("FAIL"));
  // }
#endif

  okBme = bmeInit();
#if TBOY_SERIAL_DIAG
  // Serial.print(F("BME:  "));
  // Serial.println(okBme ? F("OK") : F("FAIL"));
#endif

  okCcs = ccsInit();
#if TBOY_SERIAL_DIAG
  // Serial.print(F("CCS:  "));
  // Serial.println(okCcs ? F("OK") : F("FAIL"));
#endif

  okSd = SD.begin(PIN_SD_CS);
#if TBOY_SERIAL_DIAG
  // Serial.print(F("SD:   "));
  // Serial.println(okSd ? F("OK") : F("FAIL"));
#endif
  if (okSd) {
    sdRelease();
    if (!SD.exists(LOG_FILE))
      sdWriteHeader();
#if TBOY_SERIAL_DIAG
    // Serial.print(F("Лог:  "));
    // Serial.println(LOG_FILE);
#endif
  }

  okNrf = radio.begin();
#if TBOY_SERIAL_DIAG
  // Serial.print(F("nRF:  "));
  // Serial.println(okNrf ? F("OK") : F("FAIL"));
#endif
  if (okNrf) {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setAutoAck(false);
    radio.setPayloadSize(sizeof(TboyAirPkt));
    radio.openWritingPipe(NRF_PIPE);
    radio.stopListening();
  }

  pkt = Packet();
  lastCo2 = 400;
  lastTvoc = 0;

  runPowerOnSelfTest();
  tBeep = millis();
  tLog = millis() - DT_LOG_MS;
  tLed = millis() - DT_LED_MS;
#if TBOY_SERIAL_DIAG
  // Serial.println(F("1 с: датчики → SD → nRF TboyAirPkt; зуммер раз в мин; лента — спутники/HDOP.\n"));
#endif
}

void loop() {
  uint32_t now = millis();

  while (Serial.available())
    gps.encode(Serial.read());

  if (now - tLed >= DT_LED_MS) {
    tLed = now;
    refreshLedStripFromGps();
  }

  if (now - tLog >= DT_LOG_MS) {
    tLog = now;
    sampleAllAndLog();
  }

  if (now - tBeep >= DT_BEEP_MS) {
    tBeep = now;
    beep(80);
  }
}
