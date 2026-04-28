#include <Wire.h>

const unsigned long SCAN_PERIOD_MS = 3000;
unsigned long lastScanMs = 0;

const char* knownDeviceName(uint8_t addr) {
  switch (addr) {
    case 0x76: return "BME280";
    case 0x5A: return "CCS811";
    case 0x68: return "MPU9250 (or compatible at 0x68)";
    case 0x69: return "MPU9250 (or compatible at 0x69)";
    default:   return "Unknown device";
  }
}

void printAddressHex(uint8_t addr) {
  Serial.print("0x");
  if (addr < 16) Serial.print('0');
  Serial.print(addr, HEX);
}

void runI2CScan() {
  uint8_t found = 0;

  Serial.println();
  Serial.println("=== I2C scan start ===");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      found++;
      printAddressHex(addr);
      Serial.print("  ->  ");
      Serial.println(knownDeviceName(addr));
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices found: ");
    Serial.println(found);
  }

  Serial.println("Expected: BME280=0x76, CCS811=0x5A, MPU9250=0x68/0x69");
  Serial.println("=== I2C scan end ===");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Arduino Nano: SDA=A4, SCL=A5
  delay(300);

  Serial.println("I2C Scanner ready.");
  Serial.println("Wiring: SDA->A4, SCL->A5, GND common.");
  runI2CScan();
  lastScanMs = millis();
}

void loop() {
  if (millis() - lastScanMs >= SCAN_PERIOD_MS) {
    lastScanMs = millis();
    runI2CScan();
  }
}
