#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(); // Nano: SDA=A4, SCL=A5

  // Частые адреса BME280: 0x76 или 0x77
  bool ok = bme.begin(0x76);
  if (!ok) ok = bme.begin(0x77);

  if (!ok) {
    Serial.println("BME280 not found. Check wiring and address (0x76/0x77).");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("BME280 OK");
  Serial.println("Temp_C,Pressure_hPa,Humidity_%");
}

void loop() {
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0f; // Pa -> hPa
  float h = bme.readHumidity();

  Serial.print(t, 2);
  Serial.print(",");
  Serial.print(p, 2);
  Serial.print(",");
  Serial.println(h, 2);

  delay(1000);
}
