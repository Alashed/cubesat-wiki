#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>

// ВАЖНО: на Nano GPS TX -> D0 (RX0), поэтому Serial занят GPS.
// Во время прошивки лучше отключать TX GPS от D0.

const uint8_t SD_CS = 4;
const char* LOG_FILE = "GPSLOG.CSV";
const unsigned long LOG_PERIOD_MS = 1000;

TinyGPSPlus gps;
unsigned long lastLogMs = 0;

void writeHeaderIfNeeded() {
  if (SD.exists(LOG_FILE)) return;
  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) return;
  f.println("ms,fix,sats,hdop,lat,lon,alt_m,utc_hhmmss,date_ddmmyyyy,chars,crc_ok,crc_bad");
  f.close();
}

void logRow() {
  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) return;

  f.print(millis());
  f.print(',');
  f.print(gps.location.isValid() ? 1 : 0);
  f.print(',');
  f.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  f.print(',');
  if (gps.hdop.isValid()) f.print(gps.hdop.hdop(), 1); else f.print('0');
  f.print(',');
  if (gps.location.isValid()) f.print(gps.location.lat(), 6); else f.print('0');
  f.print(',');
  if (gps.location.isValid()) f.print(gps.location.lng(), 6); else f.print('0');
  f.print(',');
  if (gps.altitude.isValid()) f.print(gps.altitude.meters(), 1); else f.print('0');
  f.print(',');
  if (gps.time.isValid()) {
    unsigned long t = gps.time.hour() * 10000UL + gps.time.minute() * 100UL + gps.time.second();
    f.print(t);
  } else {
    f.print('0');
  }
  f.print(',');
  if (gps.date.isValid()) {
    unsigned long d = gps.date.day() * 1000000UL + gps.date.month() * 10000UL + gps.date.year();
    f.print(d);
  } else {
    f.print('0');
  }
  f.print(',');
  f.print((unsigned long)gps.charsProcessed());
  f.print(',');
  f.print((unsigned long)gps.passedChecksum());
  f.print(',');
  f.print((unsigned long)gps.failedChecksum());
  f.println();

  f.close();
}

void setup() {
  // Serial используется только как вход GPS @9600
  Serial.begin(9600);

  SD.begin(SD_CS);
  writeHeaderIfNeeded();
  lastLogMs = millis();
}

void loop() {
  while (Serial.available()) {
    gps.encode(Serial.read());
  }

  unsigned long now = millis();
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    logRow();
  }
}
