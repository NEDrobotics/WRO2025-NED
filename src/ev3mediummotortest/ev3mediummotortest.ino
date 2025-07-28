// Motor Driver pins
const int IN1 = 9;
const int IN2 = 10;
const int ENA = 11;

// Encoder pins
const int ENC_A = 2;  // Interrupt 0
const int ENC_B = 3;  // Interrupt 1

volatile long encoderCount = 0;

void setup() {
  // Set motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Set encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // Move motor forward for 3 seconds
  moveMotor(120, true);
  delay(3000);



  // Stop and print encoder count
  stopMotor();
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
  delay(2000);
}

// Interrupt Service Routine
void encoderISR() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A == B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Move motor in given direction
void moveMotor(int speed, bool forward) {
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, speed); // 0–255
}

// Stop the motor
void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}
