#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte address[6] = "RC001";

struct Data {
  int16_t j1x, j1y, j2x, j2y;
  bool b1, b2, t1, t2;
};
Data data;

void setup() {
  Serial.begin(115200);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  radio.begin();
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(120);
  radio.setPayloadSize(sizeof(Data));
  radio.openWritingPipe(address);
  radio.stopListening();
  Serial.println("TX READY");
}

void loop() {
  data.j1x = analogRead(A0);
  data.j1y = analogRead(A1);
  data.j2x = analogRead(A2);
  data.j2y = analogRead(A3);
  data.b1 = !digitalRead(3);
  data.b2 = !digitalRead(2);
  data.t1 = !digitalRead(6);
  data.t2 = !digitalRead(7);

  bool ok = radio.write(&data, sizeof(data));
  Serial.println(ok ? "TX OK" : "TX FAIL");
  delay(50);
}
