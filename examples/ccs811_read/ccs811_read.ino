#include <Wire.h>
#include <Adafruit_CCS811.h>

Adafruit_CCS811 ccs;

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(); // Nano: SDA=A4, SCL=A5

  Serial.println("CCS811 init (Adafruit lib)...");
  if (!ccs.begin(0x5A)) {
    Serial.println("CCS811 not found.");
    Serial.println("Check power, GND, SDA(A4), SCL(A5), address 0x5A");
    while (true) delay(1000);
  }

  Serial.println("CCS811 OK");
  Serial.println("Warm-up: wait 2-5 minutes");
  Serial.println("eCO2_ppm,TVOC_ppb");
}

void loop() {
  if (!ccs.available()) {
    delay(100);
    return;
  }

  if (!ccs.readData()) {
    Serial.print(ccs.geteCO2());
    Serial.print(",");
    Serial.println(ccs.getTVOC());
  } else {
    Serial.println("read_error");
  }

  delay(100);
}
