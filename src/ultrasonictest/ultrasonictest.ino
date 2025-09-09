// Left (original, used for PID)
const int TRIG_LEFT_PIN = 45;
const int ECHO_LEFT_PIN = 44;

// Right (unused for this feature but retained)
const int TRIG_RIGHT_PIN = 47;
const int ECHO_RIGHT_PIN = 46;

// Front (used to trigger the turn)
const int TRIG_FRONT_PIN = 49;
const int ECHO_FRONT_PIN = 48;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_LEFT_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);

  pinMode(TRIG_RIGHT_PIN, OUTPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);

  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);
}

// Function to measure distance from a sensor
float getDistance(int trigPin, int echoPin) {
  // Clear the TRIG pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10µs pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout (~4m max)

  // Calculate distance in cm (speed of sound = 343 m/s)
  float distance = duration * 0.0343 / 2.0;
  return distance;
}

void loop() {
  float leftDist  = getDistance(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
  float rightDist = getDistance(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
  float frontDist = getDistance(TRIG_FRONT_PIN, ECHO_FRONT_PIN);

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | Right: ");
  Serial.print(rightDist);
  Serial.print(" cm | Front: ");
  Serial.print(frontDist);
  Serial.println(" cm");

  delay(200); // adjust as needed
}
