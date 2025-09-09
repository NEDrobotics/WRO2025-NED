#include <SoftwareSerial.h>
#define huskylens Serial2 // RX, TX

void setup() {
  Serial.begin(115200);
  huskylens.begin(9600);
  delay(2000);
  Serial.println("Started.");
}

void sendRequestBlocks() {
  uint8_t frame[] = {0x55, 0xAA, 0x11, 0x00, 0x21, 0x31};
  huskylens.write(frame, sizeof(frame));
}

void loop() {
  sendRequestBlocks();
  delay(50);

  while (huskylens.available() >= 5) { // header + at least cmd
    if (huskylens.peek() != 0x55) { huskylens.read(); continue; }
    huskylens.read(); // eat 0x55
    if (huskylens.read() != 0xAA) continue;

    byte address = huskylens.read();
    byte length = huskylens.read();
    byte cmd = huskylens.read();

    if (cmd == 0x2A && length == 0x0A && huskylens.available() >= 11) {
      byte data[10];
      for (int i = 0; i < 10; i++) data[i] = huskylens.read();
      byte checksum = huskylens.read(); // consume checksum

      uint16_t x  = data[0] | (data[1] << 8);
      uint16_t y  = data[2] | (data[3] << 8);
      uint16_t w  = data[4] | (data[5] << 8);
      uint16_t h  = data[6] | (data[7] << 8);
      uint16_t id = data[8] | (data[9] << 8);

      Serial.print("Detected ID ");
      Serial.print(id);
      Serial.print(" at (");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print("), size: ");
      Serial.print(w);
      Serial.print("x");
      Serial.println(h);
    } else {
      // skip frame if not block
      for (int i = 0; i < length + 1 && huskylens.available(); i++) huskylens.read();
    }
  }
}
