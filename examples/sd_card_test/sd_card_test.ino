#include <SPI.h>
#include <SD.h>

const uint8_t SD_CS = 4;               // CS SD карты в main_full
const char* LOG_FILE = "TEST.CSV";

void printFile(const char* path) {
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println("Read fail: cannot open file");
    return;
  }

  Serial.println("--- File content start ---");
  while (f.available()) {
    Serial.write(f.read());
  }
  Serial.println();
  Serial.println("--- File content end ---");
  f.close();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("microSD test start");
  Serial.println("SPI: D11(MOSI), D12(MISO), D13(SCK), CS=D4");

  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
    Serial.println("Check power, GND, CS=D4, and SPI wires");
    while (true) delay(1000);
  }
  Serial.println("SD init OK");

  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) {
    Serial.println("Create/write failed");
    while (true) delay(1000);
  }

  // Простая строка телеметрии (пример)
  f.println("ms,temp_c,hum_%,co2_ppm");
  f.println("12345,24.7,38.5,420");
  f.close();
  Serial.println("Write OK");

  printFile(LOG_FILE);
  Serial.println("microSD test done");
}

void loop() {
  // Ничего не делаем, тест выполнен в setup()
}
