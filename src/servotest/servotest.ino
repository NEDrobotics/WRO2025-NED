#include <Servo.h>

Servo myServo;  // Create servo object

void setup() {
  myServo.attach(9); // Attach to digital pin 9
}

void loop() {
  myServo.write(90);
  delay(1000);
}
