#include <Servo.h>
#include <math.h>

// ---------------- HW & pins ----------------
Servo servo1;
Servo servo2;

// Left ultrasonic
const int TRIG_LEFT_PIN = 45;
const int ECHO_LEFT_PIN = 44;
// Right ultrasonic
const int TRIG_RIGHT_PIN = 47;
const int ECHO_RIGHT_PIN = 46;
// Front ultrasonic (kept but not used for turning here)
const int TRIG_FRONT_PIN = 49;
const int ECHO_FRONT_PIN = 48;

const int S1PIN = 10;
const int S2PIN = 11;
const int buttonPin = 53;
int buttonState = 0;

// Motor Driver pins
const int IN1_1 = 22;
const int IN2_1 = 23;
const int ENA_1  = 8;
const int IN1_2 = 24;
const int IN2_2 = 25;
const int ENA_2  = 9;

// servo mid positions
const int s1mid = 96;
const int s2mid = 115;

// ---------------- control params ----------------
// wall-following target (left or right depending on active mode)
const int targetdistance = 15; // cm

// PID params (tune)
float Kp = 4.5;
float Ki = 0.0;
float Kd = 0.6;

float integralTerm = 0.0;
float lastError = 0.0;
unsigned long lastPidTime = 0;

const float I_MAX = 100.0;
const float I_MIN = -100.0;

float filteredLeft = (float)targetdistance;
float filteredRight = 1000.0f;
float filteredFront = 1000.0f;
const float alpha = 0.3f;

unsigned long lastServoUpdate = 0;
const unsigned long servoUpdateInterval = 50;

const int MAX_CORRECTION_DEG = 25;

// steering degree used for hard steer (positive is left steer magnitude)
const int TURN_DEGREE = 45;

// Very small steering used during first lap before first turn (3-5 degrees)
const int SMALL_STEER_DEG = 9;

// speeds
const int BASE_SPEED = 120;
const int MIN_SPEED = 80;

// Turning / safety
const int TURN_PWM = 59;
const int TURN_SETTLE_MS = 120;
const float SIDE_TURN_STOP_CM = 20.0f;
const unsigned long TURN_TIMEOUT_MS = 5000UL; // safety timeout

// how much bigger than the previous reading is required to trigger a turn
const float DELTA_TO_TRIGGER = 20.0f; // cm

// short ignore after a rotation to avoid immediate retrigger
const unsigned long IGNORE_AFTER_TURN_MS = 1200UL;
unsigned long ignoreUntil = 0;

// misc
int lastS1pos = s1mid;
int lastS2pos = s2mid;
int turnCount = 0;

// ---------- Choose turn side mode (compile-time preference) ----------
// 0 = auto-detect on first trigger (recommended) -> will lock after first turn
// 1 = left-only from start
// 2 = right-only from start
const int TURN_SIDE_MODE = 0; // set to 0,1,2 as needed (1, 2 debug mode only)

// runtime-active mode (1=left, 2=right). If TURN_SIDE_MODE==0 we pick initial follow side based on closeness.
int activeTurnMode = 0;

// store baseline readings taken at start of runProgram() (kept for diagnostics)
float baselineLeft = 0.0f;
float baselineRight = 0.0f;
bool firstTurnDone = false;

// store previous filtered values to detect sudden increase (current - prev)
float oldLeft = 0.0f;
float oldRight = 0.0f;

// ---------------- helpers ----------------

// Helper: ultrasonic reading (cm) or -1 on timeout
float readUltrasonicCm(int trigPin, int echoPin) {
  unsigned long duration = 0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) return -1.0f;
  return (float)duration * 0.0343f / 2.0f; // cm
}

// Precise averaged ultrasonic sampling to confirm state
void preciseSampleUltrasonics(int samples = 8, int delayMs = 20) {
  float sumL = 0.0f, sumR = 0.0f, sumF = 0.0f;
  int countL = 0, countR = 0, countF = 0;
  for (int i = 0; i < samples; ++i) {
    float vL = readUltrasonicCm(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
    float vR = readUltrasonicCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    float vF = readUltrasonicCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
    if (vL >= 0.0f) { sumL += vL; ++countL; }
    if (vR >= 0.0f) { sumR += vR; ++countR; }
    if (vF >= 0.0f) { sumF += vF; ++countF; }
    delay(delayMs);
  }
  if (countL > 0) filteredLeft = sumL / countL;
  if (countR > 0) filteredRight = sumR / countR;
  if (countF > 0) filteredFront = sumF / countF;
  Serial.print("Precise sample -> L:");
  Serial.print(filteredLeft);
  Serial.print(" R:");
  Serial.print(filteredRight);
  Serial.print(" F:");
  Serial.println(filteredFront);
}

// Stop both motors
void stopMotors() {
  analogWrite(ENA_1, 0);
  analogWrite(ENA_2, 0);
}

// Set both motors forward/backward
void setMotorsForward(bool forward, int speed) {
  // Motor 1
  digitalWrite(IN1_1, forward ? HIGH : LOW);
  digitalWrite(IN2_1, forward ? LOW  : HIGH);
  analogWrite(ENA_1, speed);
  // Motor 2 (reverse wiring)
  digitalWrite(IN1_2, forward ? LOW : HIGH);
  digitalWrite(IN2_2, forward ? HIGH : LOW);
  analogWrite(ENA_2, speed);
}

// ------------------------------------------------------------------
// Steering helpers for a servo-steered vehicle
// ------------------------------------------------------------------

void setSteerLeftMax() {
  int leftPose  = constrain(s1mid + TURN_DEGREE, 0, 180);
  int rightPose = constrain(s2mid - TURN_DEGREE, 0, 180);
  servo1.write(leftPose);
  servo2.write(rightPose);
  lastServoUpdate = millis();
}

void setSteerRightMax() {
  int leftPose  = constrain(s1mid - TURN_DEGREE, 0, 180);
  int rightPose = constrain(s2mid + TURN_DEGREE, 0, 180);
  servo1.write(leftPose);
  servo2.write(rightPose);
  lastServoUpdate = millis();
}

void centerSteering() {
  servo1.write(s1mid);
  servo2.write(s2mid);
  lastServoUpdate = millis();
  delay(10);
}

// Rotate/turn toward a side (1=left, 2=right) by steering + driving forward until that side sensor reads < stopCm
void rotateUntilSideUnder(int side, float stopCm = SIDE_TURN_STOP_CM, unsigned long timeoutMs = TURN_TIMEOUT_MS) {
  turnCount++;
  Serial.print("steering-");
  Serial.print(side == 1 ? "LEFT" : "RIGHT");
  Serial.print(" until side < ");
  Serial.print(stopCm);
  Serial.print(" cm (timeout ");
  Serial.print(timeoutMs);
  Serial.println(" ms)");

  if (side == 1) setSteerLeftMax();
  else setSteerRightMax();
  delay(60);

  setMotorsForward(true, MIN_SPEED);
  delay(150);
  unsigned long start = millis();
  setMotorsForward(true, TURN_PWM);

  const float alphaTurn = 0.45f;
  while (true) {
    float v = -1.0f;
    if (side == 1) v = readUltrasonicCm(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
    else v = readUltrasonicCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);

    if (v >= 0.0f) {
      if (side == 1) filteredLeft  = alphaTurn * v + (1.0f - alphaTurn) * filteredLeft;
      else           filteredRight = alphaTurn * v + (1.0f - alphaTurn) * filteredRight;
    }

    float checkVal = (side == 1) ? filteredLeft : filteredRight;
    if (checkVal < stopCm) {
      Serial.print("steer stop: side reading=");
      Serial.println(checkVal);
      break;
    }

    if ((millis() - start) > timeoutMs) {
      Serial.println("steer timeout reached — stopping");
      break;
    }

    delay(10);
  }

  stopMotors();
  delay(TURN_SETTLE_MS);
  centerSteering();
  preciseSampleUltrasonics(8, 20);

  // ignore re-trigger for a short window
  ignoreUntil = millis() + IGNORE_AFTER_TURN_MS;
}

// Drive straight with servos centered and motor pwm
void driveForwardStraight(int motorPwm) {
  if ((millis() - lastServoUpdate) >= servoUpdateInterval) {
    servo1.write(s1mid);
    servo2.write(s2mid);
    lastServoUpdate = millis();
  }
  setMotorsForward(true, motorPwm);
}

// PID for wall-following: by default follows left unless activeTurnMode==2 (follow right)
float computePidOutput(float measured) {
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  float error = (float)targetdistance - measured;

  integralTerm += error * dt;
  if (integralTerm > I_MAX) integralTerm = I_MAX;
  if (integralTerm < I_MIN) integralTerm = I_MIN;

  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integralTerm + Kd * derivative;

  if (output > MAX_CORRECTION_DEG) output = MAX_CORRECTION_DEG;
  if (output < -MAX_CORRECTION_DEG) output = -MAX_CORRECTION_DEG;

  lastError = error;
  lastPidTime = now;
  return output;
}

// servo write (rate-limited)
void writeServosFromOutput(float output) {
  unsigned long now = millis();
  if ((now - lastServoUpdate) < servoUpdateInterval) return;

  int s1pos = (int)roundf((float)s1mid - output);
  int s2pos = (int)roundf((float)s2mid + output);

  s1pos = constrain(s1pos, 0, 180);
  s2pos = constrain(s2pos, 0, 180);

  servo1.write(s1pos);
  servo2.write(s2pos);

  lastS1pos = s1pos;
  lastS2pos = s2pos;
  lastServoUpdate = now;

  Serial.print("Servos s1:");
  Serial.print(s1pos);
  Serial.print(" s2:");
  Serial.println(s2pos);
}

// ------------------ setup & loop ------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ready (sensor-driven turning)");
  Serial.print("TURN_SIDE_MODE compile-time = ");
  if (TURN_SIDE_MODE == 0) Serial.println("AUTO (0)");
  else if (TURN_SIDE_MODE == 1) Serial.println("LEFT-only (1)");
  else Serial.println("RIGHT-only (2)");

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(TRIG_LEFT_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);
  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);

  servo1.attach(S1PIN);
  servo2.attach(S2PIN);

  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(ENA_1, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(ENA_2, OUTPUT);

  servo1.write(s1mid);
  servo2.write(s2mid);

  lastPidTime = millis();
  lastServoUpdate = millis();
}

void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    testProgram();
  } else {
    delay(20);
  }
}

int turningMode = 0;
int flag = 1;
void testProgram(){
  while(turnCount < 12){
    setMotorsForward(true, 230);
    float leftDist = readUltrasonicCm(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
    float rightDist = readUltrasonicCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    float frontDist = readUltrasonicCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
  
    if (leftDist >= 0.0f) filteredLeft = (alpha * leftDist) + ((1.0f - alpha) * filteredLeft);
    if (rightDist >= 0.0f) filteredRight = (alpha * rightDist) + ((1.0f - alpha) * filteredRight);
    if (frontDist >= 0.0f) filteredFront = (alpha * frontDist) + ((1.0f - alpha) * filteredFront);
    if(filteredFront < 50){
      turnCount++;
      if(flag){
        preciseSampleUltrasonics(6, 20);
        if(filteredLeft > 80) activeTurnMode = 1;
        else activeTurnMode = 2;
        flag = 0;
      }
      servo1.write(s1mid + (42 * ((activeTurnMode == 1) ? 1 : -1)));
      servo2.write(s2mid - (42 * ((activeTurnMode == 1) ? 1 : -1)));
      delay(800);
      servo1.write(s1mid);
      servo2.write(s2mid);
    }
    if(activeTurnMode == 1){
      if(filteredRight < 20){
        servo1.write(s1mid + (5 * ((activeTurnMode == 1) ? 1 : -1)));
        servo2.write(s2mid - (5 * ((activeTurnMode == 1) ? 1 : -1)));
        delay(70);
        servo1.write(s1mid);
        servo2.write(s2mid);
      }
    }
    else if(activeTurnMode == 2){
      if(filteredLeft < 20){
        servo1.write(s1mid + (5 * ((activeTurnMode == 1) ? 1 : -1)));
        servo2.write(s2mid - (5 * ((activeTurnMode == 1) ? 1 : -1)));
        delay(70);
        servo1.write(s1mid);
        servo2.write(s2mid);
      }
    }
  }
  delay(700);
  stopMotors();
}


void runProgram() {
  delay(500);
  ignoreUntil = 0;
  turnCount = 0;
  firstTurnDone = false;

  // take a few precise samples at start to establish baseline and initialize old values
  preciseSampleUltrasonics(10, 20);
  baselineLeft = filteredLeft;
  baselineRight = filteredRight;
  oldLeft = baselineLeft;   // IMPORTANT: initialize previous-reading storage here
  oldRight = baselineRight;
  Serial.print("baselineLeft="); Serial.print(baselineLeft);
  Serial.print(" baselineRight="); Serial.println(baselineRight);

  // detect if we start in the middle (AUTO mode only)
  bool startInMiddle = false;
  if (TURN_SIDE_MODE == 0 && filteredLeft > 35.0f && filteredRight > 35.0f) {
    startInMiddle = true;
    Serial.println("START: robot appears centered between walls -> will drive straight until first turn");
  }

  // activeTurnMode initially:
  // if user forced LEFT-only or RIGHT-only, respect that.
  // if AUTO (0) -> choose the sensor that is *closest* to the wall (smaller distance),
  // unless startInMiddle is true (then we remain unlocked until first trigger and drive straight).
  if (TURN_SIDE_MODE == 1) activeTurnMode = 1;
  else if (TURN_SIDE_MODE == 2) activeTurnMode = 2;
  else {
    if (startInMiddle) activeTurnMode = 0; // unlocked; we'll drive straight until first turn
    else activeTurnMode = (filteredLeft <= filteredRight) ? 1 : 2;
    Serial.print("Initial follow chosen = ");
    if (activeTurnMode == 0) Serial.println("NONE (start in middle)");
    else Serial.println(activeTurnMode == 1 ? "LEFT" : "RIGHT");
  }

  while (turnCount < 11) {
    // read sonars (fast single readings + EMA update)
    float leftDist = readUltrasonicCm(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
    float rightDist = readUltrasonicCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    float frontDist = readUltrasonicCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);

    if (leftDist >= 0.0f) filteredLeft = (alpha * leftDist) + ((1.0f - alpha) * filteredLeft);
    if (rightDist >= 0.0f) filteredRight = (alpha * rightDist) + ((1.0f - alpha) * filteredRight);
    if (frontDist >= 0.0f) filteredFront = (alpha * frontDist) + ((1.0f - alpha) * filteredFront);

    Serial.print("L:"); Serial.print(filteredLeft);
    Serial.print("  R:"); Serial.print(filteredRight);
    Serial.print("  F:"); Serial.println(filteredFront);

    // Decide whether to trigger a first-turn (dynamic > previous + DELTA_TO_TRIGGER)
    bool turned = false;
    // Only allow dynamic auto-detection if compile-time mode = 0 (AUTO) and first turn not done.
    if (!firstTurnDone && TURN_SIDE_MODE == 0 && (millis() > ignoreUntil)) {
      // check both sides; compare against previous filtered reading (oldLeft/oldRight)
      float leftDelta = filteredLeft - oldLeft;
      float rightDelta = filteredRight - oldRight;

      if ((leftDelta >= DELTA_TO_TRIGGER) || (rightDelta >= DELTA_TO_TRIGGER)) {
        // choose the side with bigger delta
        int chosenSide = (leftDelta > rightDelta) ? 1 : 2;
        Serial.print("AUTO first-turn triggered by side ");
        Serial.print(chosenSide == 1 ? "LEFT" : "RIGHT");
        Serial.print(" (delta L="); Serial.print(leftDelta);
        Serial.print(" R="); Serial.print(rightDelta);
        Serial.println(" )");

        // confirm with precise sampling (compare again against previous reading stored in oldLeft/oldRight)
        preciseSampleUltrasonics(6, 20);
        float confirmVal = (chosenSide == 1) ? filteredLeft : filteredRight;
        float confirmPrev = (chosenSide == 1) ? oldLeft : oldRight;
        if (confirmVal - confirmPrev >= DELTA_TO_TRIGGER) {
          // lock the active mode to the chosen side for subsequent turns
          activeTurnMode = chosenSide;
          firstTurnDone = true;
          Serial.print("Locking activeTurnMode to ");
          Serial.println(activeTurnMode == 1 ? "LEFT" : "RIGHT");
          // perform the turn toward chosen side (steering-based)
          // rotateUntilSideUnder(chosenSide, SIDE_TURN_STOP_CM, TURN_TIMEOUT_MS);
          setMotorsForward(true, 90);
          delay(5);
          servo1.write(s1mid + (42 * ((activeTurnMode == 1) ? 1 : -1)));
          servo2.write(s2mid - (42 * ((activeTurnMode == 1) ? 1 : -1)));
          delay(1450);
          servo1.write(s1mid);
          servo2.write(s2mid);
          if(filteredLeft < 30){
            delay(625 - (625 * ((activeTurnMode == 1) ? 1 : -1)));
          }
          else if(filteredRight < 30){
            delay(625 + (625 * ((activeTurnMode == 1) ? 1 : -1)));
          }
          else{
            delay(550);
          }
          integralTerm = 0.0f; lastError = 0.0f; lastPidTime = millis();
          turned = true;
          stopMotors();
          delay(10);
          preciseSampleUltrasonics(6, 20);
        } else {
          Serial.println("Auto-trigger was transient on confirm; continue.");
        }
      }
    }

    // If not turned yet and we already locked to a side (either by compile-time preference or by first auto turn),
    // use that side's trigger logic: turn when THAT side reading exceeds previous reading by DELTA_TO_TRIGGER.
    if (!turned && (millis() > ignoreUntil)) {
      if (activeTurnMode == 1) {
        // left-only active: compare to previous filtered (oldLeft)
        if (filteredLeft - oldLeft >= DELTA_TO_TRIGGER) {
          Serial.println("LEFT active-mode trigger -> turning left");
          preciseSampleUltrasonics(6, 20);
          if (filteredLeft - oldLeft >= DELTA_TO_TRIGGER) {
            rotateUntilSideUnder(1, SIDE_TURN_STOP_CM, TURN_TIMEOUT_MS);
            integralTerm = 0.0f; lastError = 0.0f; lastPidTime = millis();
            firstTurnDone = true;
            turned = true;
          }
        }
      } else if (activeTurnMode == 2) {
        // right-only active: compare to previous filtered (oldRight)
        if (filteredRight - oldRight >= DELTA_TO_TRIGGER) {
          Serial.println("RIGHT active-mode trigger -> turning right");
          preciseSampleUltrasonics(6, 20);
          if (filteredRight - oldRight >= DELTA_TO_TRIGGER) {
            rotateUntilSideUnder(2, SIDE_TURN_STOP_CM, TURN_TIMEOUT_MS);
            integralTerm = 0.0f; lastError = 0.0f; lastPidTime = millis();
            firstTurnDone = true;
            turned = true;
          }
        }
      }
    }

    if (!turned) {
      // Normal wall-follow PID when not turning.
      // Two special cases BEFORE first turn:
      // 1) if we started in the middle (startInMiddle==true), drive STRAIGHT until first turn
      // 2) otherwise, if AUTO mode and before first turn, use very small steering
      bool beforeFirstTurn = !firstTurnDone && (TURN_SIDE_MODE == 0);

      // If we started in the middle and still haven't done first turn, go straight at full speed
      if (beforeFirstTurn && (filteredLeft > 35.0f && filteredRight > 35.0f)) {
        // center and full speed
        centerSteering();
        setMotorsForward(true, 60);
      } else {
        // Choose measured value for wall follow: if locked to right, follow right; otherwise follow left.
        float useMeasured = (activeTurnMode == 2) ? filteredRight : filteredLeft;
        float pidOutput = computePidOutput(useMeasured);

        if (beforeFirstTurn) {
          // clamp pidOutput to a very small steer (SMALL_STEER_DEG)
          if (pidOutput > SMALL_STEER_DEG) pidOutput = SMALL_STEER_DEG;
          if (pidOutput < -SMALL_STEER_DEG) pidOutput = -SMALL_STEER_DEG;
        }

        // If following right side, invert sign so control steering orientation stays intuitive
        float servoOutput = pidOutput * ((activeTurnMode == 2) ? -1.0f : 1.0f);

        writeServosFromOutput(servoOutput);

        int absOut = (int)abs(pidOutput);
        float frac = (float)absOut / (float)MAX_CORRECTION_DEG;
        if (frac > 1.0f) frac = 1.0f;
        float reduction = frac * 0.6f;
        int speed = (int)roundf((float)BASE_SPEED * (1.0f - reduction));
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        if(beforeFirstTurn){ 
          speed = (int)roundf((float)60 * (1.0f - reduction));
          if (speed < 57) speed = 57;
        }
        setMotorsForward(true, speed);
      }
    }

    // save previous filtered values for next-iteration delta checks
    oldLeft = filteredLeft;
    oldRight = filteredRight;

    delay(20);
  }

  // After running until turnCount limit, drive forward then stop
  driveForwardStraight(120);
  servo1.write(s1mid - (7 * ((activeTurnMode == 1) ? 1 : -1)));
  servo2.write(s2mid + (7 * ((activeTurnMode == 1) ? 1 : -1)));
  delay(1000);
  stopMotors();
}
