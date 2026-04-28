#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>

// nRF24 (как в проекте)
const uint8_t NRF_CE = 9;
const uint8_t NRF_CSN = 10;
const byte NRF_PIPE[6] = "TBOY1";

// I2C адреса
const uint8_t MPU_ADDR = 0x68;

RF24 radio(NRF_CE, NRF_CSN);
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

struct __attribute__((packed)) Telemetry {
  uint32_t ms;
  int16_t temp_c10;
  uint16_t press_hpa10;
  uint8_t hum_x2;
  uint16_t eco2_ppm;
  uint16_t tvoc_ppb;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint8_t flags; // bit0=BME ok, bit1=CCS ok, bit2=MPU ok
};

Telemetry pkt;

static uint8_t readReg8(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(dev, (uint8_t)1);
  if (!Wire.available()) return 0;
  return Wire.read();
}

static bool readRegs(uint8_t dev, uint8_t reg, uint8_t* dst, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) dst[i] = Wire.read();
  return true;
}

static void writeReg8(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool initMpu() {
  uint8_t who = readReg8(MPU_ADDR, 0x75);
  if (who != 0x71 && who != 0x73) return false;
  writeReg8(MPU_ADDR, 0x6B, 0x00); // wake
  delay(30);
  writeReg8(MPU_ADDR, 0x1B, 0x00); // gyro +-250 dps
  writeReg8(MPU_ADDR, 0x1C, 0x00); // accel +-2g
  return true;
}

static bool readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[14];
  if (!readRegs(MPU_ADDR, 0x3B, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

bool okBme = false;
bool okCcs = false;
bool okMpu = false;

void setup() {
  Wire.begin();
  delay(20);

  okBme = bme.begin(0x76) || bme.begin(0x77);
  okCcs = ccs.begin(0x5A);
  okMpu = initMpu();

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(Telemetry));
  radio.openWritingPipe(NRF_PIPE);
  radio.stopListening();
}

void loop() {
  memset(&pkt, 0, sizeof(pkt));
  pkt.ms = millis();

  if (okBme) {
    pkt.flags |= 0x01;
    pkt.temp_c10 = (int16_t)(bme.readTemperature() * 10.0f);
    float p = bme.readPressure() / 100.0f;
    if (p < 0) p = 0;
    if (p > 6553.5f) p = 6553.5f;
    pkt.press_hpa10 = (uint16_t)(p * 10.0f);
    float h = bme.readHumidity();
    if (h < 0) h = 0;
    if (h > 100) h = 100;
    pkt.hum_x2 = (uint8_t)(h * 2.0f);
  }

  if (okCcs && ccs.available() && !ccs.readData()) {
    pkt.flags |= 0x02;
    pkt.eco2_ppm = ccs.geteCO2();
    pkt.tvoc_ppb = ccs.getTVOC();
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (okMpu && readMpuRaw(ax, ay, az, gx, gy, gz)) {
    pkt.flags |= 0x04;
    pkt.ax = ax; pkt.ay = ay; pkt.az = az;
    pkt.gx = gx; pkt.gy = gy; pkt.gz = gz;
  }

  radio.write(&pkt, sizeof(pkt));
  delay(1000);
}
