// TCS3200 simple mapping (no calibration)
// Pins (your values)
const int s0 = 3; // 28
const int s1 = 4; // 29
const int s2 = 5; // 30
const int s3 = 6; // 31
const int outPin = 7; // 32
const int ledPin = 2; // 33 LED control

// Default "calibrated-like" ranges (adjust if needed)
const unsigned int RAW_MIN = 0;   // when sensor sees very bright -> low pulse count
const unsigned int RAW_MAX = 90; // when sensor sees very dark  -> high pulse count

void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);

  // Frequency scaling 20%
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);

  // Turn on module LED
  digitalWrite(ledPin, HIGH);

  Serial.println(F("TCS3200 simple mapper started (no calibration)."));
}

// read one channel (returns pulse length in microseconds)
unsigned int readColorRaw(int s2Val, int s3Val) {
  digitalWrite(s2, s2Val);
  digitalWrite(s3, s3Val);
  delayMicroseconds(300); // allow filter select to settle
  unsigned int v = pulseIn(outPin, LOW, 100000); // 100 ms timeout
  if (v == 0) v = 10000; // fallback if timeout
  return v;
}

// float map helper
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return (out_min + out_max) / 2.0;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  // raw readings (note: lower raw => brighter)
  unsigned int rawR = readColorRaw(LOW, LOW);    // red
  unsigned int rawG = readColorRaw(HIGH, HIGH); // green
  unsigned int rawB = readColorRaw(LOW, HIGH);  // blue

  // map raw -> 0..255 (invert because lower pulse = brighter)
  int R = constrain((int)round(mapFloat((float)rawR, (float)30, (float)900, 255.0, 0.0)), 0, 255);
  int G = constrain((int)round(mapFloat((float)rawG, (float)30, (float)890, 255.0, 0.0)), 0, 255);
  int B = constrain((int)round(mapFloat((float)rawB, (float)30, (float)880, 255.0, 0.0)), 0, 255);

  // Primary CSV output for Python: mapped RGB (0-255)
  Serial.print(R); Serial.print(","); Serial.print(G); Serial.print(","); Serial.println(B);

  // Also print raw for debugging (optional)
  Serial.print("RAW: ");
  Serial.print(rawR); Serial.print(","); Serial.print(rawG); Serial.print(","); Serial.println(rawB);

  delay(50); // ~20 updates/sec
}
