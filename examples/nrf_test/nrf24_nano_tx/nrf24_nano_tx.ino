#include <SPI.h>
#include <RF24.h>

// Nano (как в main_full): CE=D9, CSN=D10
const uint8_t PIN_NRF_CE = 9;
const uint8_t PIN_NRF_CSN = 10;

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte PIPE[6] = "TBOY1";

struct TelemetryPacket {
  uint32_t ms;
  uint16_t counter;
  int16_t value;
};

TelemetryPacket pkt;
uint16_t counterTx = 0;

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
  radio.openWritingPipe(PIPE);
  radio.stopListening();

  Serial.println("Nano TX ready (ch=76, pipe=TBOY1)");
}

void loop() {
  pkt.ms = millis();
  pkt.counter = counterTx++;
  pkt.value = (int16_t)(1000 + (pkt.counter % 200)); // тестовое значение

  bool ok = radio.write(&pkt, sizeof(pkt));

  Serial.print("TX ");
  Serial.print(ok ? "OK " : "FAIL ");
  Serial.print("cnt=");
  Serial.print(pkt.counter);
  Serial.print(" ms=");
  Serial.print(pkt.ms);
  Serial.print(" val=");
  Serial.println(pkt.value);

  delay(1000);
}
