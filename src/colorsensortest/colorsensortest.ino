// Pins
const int s0 = 4;
const int s1 = 5;
const int s2 = 6;
const int s3 = 7;
const int outPin = 8;
const int ledPin = 13; // LED control

void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);

  // Set frequency scaling to 20%
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);

  // Turn on white LED
  digitalWrite(ledPin, HIGH);
}

unsigned int readColor(int s2Val, int s3Val) {
  digitalWrite(s2, s2Val);
  digitalWrite(s3, s3Val);
  delayMicroseconds(300); // shorter delay to speed up but still stable
  unsigned int value = pulseIn(outPin, LOW, 100000); // timeout 100ms
  if (value == 0) value = 10000; // fallback if timeout happens
  return value;
}

void loop() {
  // Read raw color frequencies
  unsigned int redRaw = readColor(LOW, LOW);     // Red filter
  unsigned int greenRaw = readColor(HIGH, HIGH); // Green filter
  unsigned int blueRaw = readColor(LOW, HIGH);   // Blue filter

  // Map values to 0–255 (calibrate if needed)
  int red = constrain(map(redRaw, 20, 600, 255, 0), 0, 255);
  int green = constrain(map(greenRaw, 20, 600, 255, 0), 0, 255);
  int blue = constrain(map(blueRaw, 20, 600, 255, 0), 0, 255);

  // Send as clean CSV for Processing/visualizing
  Serial.print(red); Serial.print(",");
  Serial.print(green); Serial.print(",");
  Serial.println(blue);

  delay(50); // Update every 50ms
}
