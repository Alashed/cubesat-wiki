#include <SPI.h>
#include <RF24.h>

// ESP32 (как в main_esp32_nrf_rx): CE=2, CSN=4, VSPI=18/19/23
const uint8_t PIN_NRF_CE = 2;
const uint8_t PIN_NRF_CSN = 4;

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte PIPE[6] = "TBOY1";

struct TelemetryPacket {
  uint32_t ms;
  uint16_t counter;
  int16_t value;
};

void setup() {
  Serial.begin(115200);
  delay(300);

  if (!radio.begin()) {
    Serial.println("nRF24 begin FAILED");
    while (true) delay(1000);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(TelemetryPacket));
  radio.openReadingPipe(1, PIPE);
  radio.startListening();

  Serial.println("ESP32 RX ready (ch=76, pipe=TBOY1)");
  Serial.println("Waiting packets...");
}

void loop() {
  if (!radio.available()) {
    delay(10);
    return;
  }

  TelemetryPacket pkt;
  radio.read(&pkt, sizeof(pkt));

  Serial.print("RX cnt=");
  Serial.print(pkt.counter);
  Serial.print(" tx_ms=");
  Serial.print(pkt.ms);
  Serial.print(" val=");
  Serial.println(pkt.value);
}
