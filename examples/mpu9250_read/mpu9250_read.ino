#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;   // MPU9250
const uint8_t MAG_ADDR = 0x0C;   // AK8963 inside MPU9250

uint8_t readReg8(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(dev, (uint8_t)1);
  if (!Wire.available()) return 0;
  return Wire.read();
}

bool readRegs(uint8_t dev, uint8_t reg, uint8_t* dst, uint8_t len) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) dst[i] = Wire.read();
  return true;
}

void writeReg8(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool initMpu9250() {
  // WHO_AM_I для MPU9250 обычно 0x71 (иногда 0x73)
  uint8_t who = readReg8(MPU_ADDR, 0x75);
  if (who != 0x71 && who != 0x73) return false;

  writeReg8(MPU_ADDR, 0x6B, 0x00); // PWR_MGMT_1: wake up
  delay(50);
  writeReg8(MPU_ADDR, 0x1B, 0x00); // GYRO_CONFIG: +-250 dps
  writeReg8(MPU_ADDR, 0x1C, 0x00); // ACCEL_CONFIG: +-2g
  writeReg8(MPU_ADDR, 0x37, 0x02); // INT_PIN_CFG: bypass on (доступ к AK8963)
  return true;
}

bool initMag() {
  uint8_t who = readReg8(MAG_ADDR, 0x00); // WIA
  if (who != 0x48) return false;

  writeReg8(MAG_ADDR, 0x0A, 0x00); // power down
  delay(10);
  writeReg8(MAG_ADDR, 0x0A, 0x16); // continuous mode 2, 16-bit, 100 Hz
  delay(10);
  return true;
}

bool readAccelGyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
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

bool readMag(int16_t &mx, int16_t &my, int16_t &mz) {
  // ST1 bit0 = data ready
  if ((readReg8(MAG_ADDR, 0x02) & 0x01) == 0) return false;

  uint8_t b[7];
  if (!readRegs(MAG_ADDR, 0x03, b, 7)) return false;
  if (b[6] & 0x08) return false; // overflow

  // У AK8963 порядок little-endian
  mx = (int16_t)((b[1] << 8) | b[0]);
  my = (int16_t)((b[3] << 8) | b[2]);
  mz = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(); // Nano: SDA=A4, SCL=A5

  if (!initMpu9250()) {
    Serial.println("MPU9250 init failed.");
    Serial.println("Check wiring, power, GND, and address 0x68.");
    while (true) delay(1000);
  }

  if (!initMag()) {
    Serial.println("AK8963 (magnetometer) init failed.");
    Serial.println("Accel/Gyro may still work, mag may be unavailable.");
  } else {
    Serial.println("AK8963 OK.");
  }

  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  int16_t mx = 0, my = 0, mz = 0;

  bool okImu = readAccelGyro(ax, ay, az, gx, gy, gz);
  bool okMag = readMag(mx, my, mz);

  if (!okImu) {
    Serial.println("imu_read_error");
    delay(200);
    return;
  }

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");

  if (okMag) {
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.println(mz);
  } else {
    Serial.println("0,0,0");
  }

  delay(200);
}
