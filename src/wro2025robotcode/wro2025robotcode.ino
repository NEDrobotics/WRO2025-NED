#include <Servo.h>
#include "mpu6500.h"
#include <math.h>

bfs::Mpu6500 imu;

// Mahony-like gain (used in main IMU integration)
const float gyroKp = 3.0f;
const float sampleInterval = 0.01f;

// gyro bias (rad/s)
float gbx = 0.0f, gby = 0.0f, gbz = 0.0f;

// quaternion (q0 = w, q1 = x, q2 = y, q3 = z)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

unsigned long lastMicros = 0;

// helper: normalize a 3-element vector
void normalize3(float &x, float &y, float &z) {
  float n = sqrtf(x*x + y*y + z*z);
  if (n <= 1e-9f) return;
  x /= n; y /= n; z /= n;
}

// normalize quaternion
void normalizeQuat() {
  float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (n <= 1e-9f) { q0=1; q1=q2=q3=0; return; }
  q0/=n; q1/=n; q2/=n; q3/=n;
}

void calibrateGyroBias(int samples = 800) {
  Serial.println("Calibrating gyro biases — keep sensor still...");
  double sumx=0, sumy=0, sumz=0;
  int got=0;
  unsigned long start = millis();
  while (got < samples && (millis() - start) < 5000) {
    if (imu.Read()) {
      sumx += imu.gyro_x_radps();
      sumy += imu.gyro_y_radps();
      sumz += imu.gyro_z_radps();
      got++;
      delay(2);
    }
  }
  if (got == 0) {
    gbx = gby = gbz = 0;
  } else {
    gbx = sumx / got;
    gby = sumy / got;
    gbz = sumz / got;
  }
  Serial.print("Gyro bias (rad/s): ");
  Serial.print(gbx,6); Serial.print(", ");
  Serial.print(gby,6); Serial.print(", ");
  Serial.println(gbz,6);
}

// ---------------- HW & pins ----------------
Servo servo1;
Servo servo2;

// Left (original, used for PID)
const int TRIG_LEFT_PIN = 37;
const int ECHO_LEFT_PIN = 36;
// Right (unused for this feature but retained)
const int TRIG_RIGHT_PIN = 39;
const int ECHO_RIGHT_PIN = 38;
// Front (used to trigger the turn)
const int TRIG_FRONT_PIN = 41;
const int ECHO_FRONT_PIN = 40;

const int S1PIN = 10;
const int S2PIN = 11;
const int buttonPin = 53;
int buttonState = 0;

// Motor Driver pins
const int IN1_1 = 22;
const int IN2_1 = 23;
const int ENA_1 = 8;
const int IN1_2 = 24;
const int IN2_2 = 25;
const int ENA_2 = 9;

// servo mid positions (your values)
const int s1mid = 99;
const int s2mid = 114;

// PID / control targets
const int targetdistance = 15; // cm (left)
const int targetmargin = 0;    // deadband in cm

// --- PID parameters (tune these) ---
float Kp = 1.5;    // proportional gain
float Ki = 0.6;    // integral gain
float Kd = 0.6;    // derivative gain

// PID state
float integralTerm = 0.0;
float lastError = 0.0;
unsigned long lastPidTime = 0; // ms

// integral windup clamp
const float I_MAX = 100.0;
const float I_MIN = -100.0;

// ultrasonic filtering (exponential moving average)
float filteredLeft = (float)targetdistance;
float filteredRight = 1000.0f;
float filteredFront = 1000.0f;
const float alpha = 0.3f; // EMA smoothing

// servo update / rate limiting
unsigned long lastServoUpdate = 0;
const unsigned long servoUpdateInterval = 50; // ms

// PID output clamp (degrees)
const int MAX_CORRECTION_DEG = 30;

// Drive speed constants
const int BASE_SPEED = 255;      // normal forward speed (0..255)
const int MIN_SPEED = 110;
const int AVOID_SPEED = 255;

// Distances & turn thresholds (tune these)
const int LEFT_TRIGGER_DISTANCE_CM = 50;   // when left > 50 -> special forward
const int FRONT_TRIGGER_DISTANCE_CM = 75;  // when front < 75 -> begin 90° left turn

// Special forward speed as percent of BASE_SPEED
const float SPECIAL_FORWARD_SPEED_PCT = 0.80f; // 80%

// Turning parameters
const int TURN_PWM = 220;               // PWM used for in-place turn (tune)
const float YAW_TOLERANCE_DEG = 3.0f;   // stop when within this tolerance
const int TURN_SETTLE_MS = 120;         // small settle after stop

// IMU yaw globals (we'll use -180..+180)
float currentYawDeg = 0.0f;            // estimated yaw in degrees (-180..+180)
float currentYawRateDeg = 0.0f;        // gz in deg/s

// yaw reference at program start
float yawZero = 0.0f;

// servo positions last known
int lastS1pos = s1mid;
int lastS2pos = s2mid;

// Modes
enum Mode { MODE_NORMAL=0, MODE_SPECIAL_FORWARD=1, MODE_TURNING=2 };
Mode mode = MODE_NORMAL;

// front-ignore cooldown after turning to avoid immediate re-trigger (ms)
const unsigned long FRONT_IGNORE_MS = 1000UL;
unsigned long frontIgnoreUntil = 0;

// ---------- NEW: left-debounce/ignore variables ----------
const unsigned long LEFT_DEBOUNCE_MS = 400UL;          // left must be above threshold for this long
unsigned long leftAboveSince = 0;                     // timestamp when left first crossed threshold
const unsigned long LEFT_IGNORE_AFTER_TURN_MS = 800UL; // don't consider left after performing a rotation
unsigned long leftIgnoreUntil = 0;
// ----------------------------------------------------------------

 // helper: angle difference normalized to [-180,180]
float angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// ------------------ IMU update function ------------------
// Performs one Mahony-like quaternion update (if imu.Read() available) and
// returns current yaw in degrees in range [-180, +180]. It also updates
// global currentYawDeg and currentYawRateDeg.
bool updateIMU(float &outYawDeg, float &outYawRateDeg) {
  if (!imu.Read()) return false; // no new data

  // dt calculation
  unsigned long nowUs = micros();
  float dt = (nowUs - lastMicros) * 1e-6f;
  if (dt <= 0 || dt > 0.1f) dt = sampleInterval;
  lastMicros = nowUs;

  // read sensor values
  float ax = imu.accel_x_mps2();
  float ay = imu.accel_y_mps2();
  float az = imu.accel_z_mps2();

  float gx = imu.gyro_x_radps() - gbx;
  float gy = imu.gyro_y_radps() - gby;
  float gz = imu.gyro_z_radps() - gbz;

  // normalize accel vector
  float vx = ax, vy = ay, vz = az;
  normalize3(vx, vy, vz);

  // Estimated gravity from quaternion
  float v_est_x = 2.0f * (q1*q3 - q0*q2);
  float v_est_y = 2.0f * (q0*q1 + q2*q3);
  float v_est_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // error = v_est x v_meas
  float ex = (v_est_y * vz - v_est_z * vy);
  float ey = (v_est_z * vx - v_est_x * vz);
  float ez = (v_est_x * vy - v_est_y * vx);

  // Apply proportional correction to gyro (Mahony P term)
  gx += gyroKp * ex;
  gy += gyroKp * ey;
  gz += gyroKp * ez;

  // Integrate quaternion: q_dot = 0.5 * q * omega (omega = [0 gx gy gz])
  float dq0 =  0.5f * ( - q1*gx - q2*gy - q3*gz );
  float dq1 =  0.5f * (   q0*gx + q2*gz - q3*gy );
  float dq2 =  0.5f * (   q0*gy - q1*gz + q3*gx );
  float dq3 =  0.5f * (   q0*gz + q1*gy - q2*gx );

  q0 += dq0 * dt;
  q1 += dq1 * dt;
  q2 += dq2 * dt;
  q3 += dq3 * dt;
  normalizeQuat();

  // Extract yaw from quaternion (radians)
  float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
  const float R2D = 57.295779513082320876f;
  float yaw_deg = yaw * R2D; // this is in -180..+180 from atan2

  // yaw rate from gz (rad/s -> deg/s)
  float yawRateDeg = gz * R2D;

  // update globals & outputs
  currentYawDeg = yaw_deg;
  currentYawRateDeg = yawRateDeg;
  outYawDeg = currentYawDeg;
  outYawRateDeg = currentYawRateDeg;
  return true;
}

// ------------------ Utility functions ------------------

void stopMotors() {
  analogWrite(ENA_1, 0);
  analogWrite(ENA_2, 0);
}

// Set both motors for forward/backward (same as old setMotors)
void setMotorsForward(bool forward, int speed) {
  // Motor 1
  digitalWrite(IN1_1, forward ? HIGH : LOW);
  digitalWrite(IN2_1, forward ? LOW : HIGH);
  analogWrite(ENA_1, speed);
  // Motor 2
  digitalWrite(IN1_2, forward ? LOW : HIGH);
  digitalWrite(IN2_2, forward ? HIGH : LOW);
  analogWrite(ENA_2, speed);
}

// Rotate in place LEFT: left wheel backward, right wheel forward
// If your robot turns the wrong way, swap HIGH/LOW for each motor here.
void rotateLeftInPlace(int pwm) {
  digitalWrite(IN1_1, HIGH);  // motor1 backward
  digitalWrite(IN2_1, LOW);
  digitalWrite(IN1_2, LOW);  // motor2 forward
  digitalWrite(IN2_2, HIGH);
  analogWrite(ENA_1, pwm);
  analogWrite(ENA_2, pwm);
}

// small helper to set servos to a steering "output" immediately (bypasses rate-limit)
void setServosImmediateFromOutput(float output) {
  // NOTE: mapping: output < 0 -> RIGHT turn; output > 0 -> LEFT turn
  int s1pos = (int)roundf((float)s1mid - output);
  int s2pos = (int)roundf((float)s2mid + output);
  s1pos = constrain(s1pos, 0, 180);
  s2pos = constrain(s2pos, 0, 180);
  servo1.write(s1pos);
  servo2.write(s2pos);
  lastS1pos = s1pos;
  lastS2pos = s2pos;
  lastServoUpdate = millis();
}

// Blocking rotate left by 90° using yawZero as reference (target = yawZero + 90).
// Uses updateIMU() in the loop so quaternion/mahony output is the single truth.
void rotateLeft90BlockingToStartRef() {
  // Put servos to a left-turn pose for consistency
  int leftServoPose = constrain(s1mid + MAX_CORRECTION_DEG, 0, 180);
  int rightServoPose = constrain(s2mid - MAX_CORRECTION_DEG, 0, 180);
  servo1.write(leftServoPose);
  servo2.write(rightServoPose);
  lastServoUpdate = millis();

  // compute absolute target yaw relative to start (yawZero + 90) and normalize to -180..180
  float targetYaw = yawZero - 90.0f;
  while (targetYaw > 180.0f) targetYaw -= 360.0f;
  while (targetYaw <= -180.0f) targetYaw += 360.0f;

  Serial.print("rotateLeft90ToStartRef: currentYaw=");
  Serial.print(currentYawDeg);
  Serial.print(" targetYaw=");
  Serial.println(targetYaw);

  // start rotation
  rotateLeftInPlace(TURN_PWM);

  // loop until within tolerance using updateIMU()
  while (true) {
    float yawOut, yawRate;
    updateIMU(yawOut, yawRate); // update quaternion and currentYawDeg
    float err = angleDiff(targetYaw, currentYawDeg); // signed shortest difference
    if (fabs(err) <= YAW_TOLERANCE_DEG) break;
    delay(5);
  }

  stopMotors();
  delay(TURN_SETTLE_MS);

  // set currentYawDeg exactly to targetYaw (normalized)
  float norm = yawZero - 90.0f;
  while (norm > 180.0f) norm -= 360.0f;
  while (norm <= -180.0f) norm += 360.0f;
  currentYawDeg = norm;

  // set servos back to neutral
  servo1.write(s1mid);
  servo2.write(s2mid);
  lastServoUpdate = millis();

  // IMPORTANT: prevent immediate re-trigger by ignoring front sonar for a brief time
  filteredFront = 1000.0f;               // clear filtered value (makes it "far")
  frontIgnoreUntil = millis() + FRONT_IGNORE_MS;

  // Also ignore left for a short window after the turn
  leftIgnoreUntil = millis() + LEFT_IGNORE_AFTER_TURN_MS;
  leftAboveSince = 0;

  // adjust yawZero to new heading (we turned left +90)
  yawZero -= 90.0f;
  while (yawZero > 180.0f) yawZero -= 360.0f;
  while (yawZero <= -180.0f) yawZero += 360.0f;

  Serial.print("rotateLeft90ToStartRef: done yaw=");
  Serial.print(currentYawDeg);
  Serial.print("  new yawZero=");
  Serial.println(yawZero);
}

// ---------- Drive forward while maintaining a target yaw ----------
const float YAW_HOLD_KP = 0.6f;     // deg of servo per deg of yaw error (tune)
const float YAW_HOLD_MAX_DEG = 12;  // maximum yaw-based steering output (degrees)

// Drive forward while holding absolute yaw target (non-blocking; call every loop)
void driveForwardMaintainYaw(float targetYawDeg, int motorPwm) {
  float yawErr = angleDiff(targetYawDeg, currentYawDeg); // target - current
  float yawOutput = YAW_HOLD_KP * yawErr;
  if (yawOutput > YAW_HOLD_MAX_DEG) yawOutput = YAW_HOLD_MAX_DEG;
  if (yawOutput < -YAW_HOLD_MAX_DEG) yawOutput = -YAW_HOLD_MAX_DEG;

  setServosImmediateFromOutput(yawOutput);
  setMotorsForward(true, motorPwm);
}

// ------------------ main setup & loop ------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("Error initializing IMU");
    while (1) {}
  }
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while (1) {}
  }

  lastMicros = micros();
  Serial.println("ready");

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
    runProgram();
  } else {
    delay(1);
  }
}

void runProgram() {
  delay(500);
  calibrateGyroBias();

  // Warm IMU and set yawZero using updateIMU()
  for (int i = 0; i < 30; ++i) {
    float y, yr;
    updateIMU(y, yr);
    delay(10);
  }
  // set yawZero to current quaternion-derived yaw (in -180..180)
  yawZero = currentYawDeg;
  Serial.print("yawZero set to ");
  Serial.println(yawZero);

  // ensure left debounce state is clean
  leftAboveSince = 0;
  leftIgnoreUntil = 0;
  frontIgnoreUntil = 0;

  while (true) {
    // update IMU each loop (if new data available)
    float ytmp, yrtmp;
    updateIMU(ytmp, yrtmp);

    // ----- read sonars -----
    unsigned long now = millis();
    float leftDist = readUltrasonicCm(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
    float rightDist = readUltrasonicCm(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);
    float frontDist = readUltrasonicCm(TRIG_FRONT_PIN, ECHO_FRONT_PIN);

    if (leftDist >= 0.0f) filteredLeft = (alpha * leftDist) + ((1.0f - alpha) * filteredLeft);
    if (rightDist >= 0.0f) filteredRight = (alpha * rightDist) + ((1.0f - alpha) * filteredRight);
    if (frontDist >= 0.0f) filteredFront = (alpha * frontDist) + ((1.0f - alpha) * filteredFront);

    Serial.print("Yaw:"); Serial.print(currentYawDeg,2);
    Serial.print("  L:"); Serial.print(filteredLeft);
    Serial.print("  F:"); Serial.println(filteredFront);

    // STATE MACHINE
    if (mode == MODE_NORMAL) {
      // LEFT trigger now uses debounce and ignore-after-turn
      if (millis() > leftIgnoreUntil) {
        if (filteredLeft > (float)LEFT_TRIGGER_DISTANCE_CM) {
          if (leftAboveSince == 0) leftAboveSince = millis();
          else if ((millis() - leftAboveSince) >= LEFT_DEBOUNCE_MS) {
            // persistently above threshold -> enter special forward
            mode = MODE_SPECIAL_FORWARD;
            Serial.println("MODE -> SPECIAL_FORWARD (left > threshold persisted)");
            // reset debounce so when returning it restarts
            leftAboveSince = 0;
          }
        } else {
          // not above threshold -> reset debounce
          leftAboveSince = 0;
        }
      } else {
        // currently ignoring left (shortly after turn)
        leftAboveSince = 0;
      }

      if (mode == MODE_NORMAL) {
        // still normal -> do PID wall following
        float pidOutput = computePidOutput(filteredLeft);
        writeServosFromOutput(pidOutput);

        // speed scaling by steering
        int absOut = (int)abs(pidOutput);
        float frac = (float)absOut / (float)MAX_CORRECTION_DEG;
        if (frac > 1.0f) frac = 1.0f;
        float reduction = frac * 0.6f;
        int speed = (int)roundf((float)BASE_SPEED * (1.0f - reduction));
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        setMotorsForward(true, speed);
      }
    }
    else if (mode == MODE_SPECIAL_FORWARD) {
      // drive straight at SPECIAL_FORWARD_SPEED_PCT of BASE_SPEED while holding yawZero
      int specialSpeed = (int)roundf((float)BASE_SPEED * SPECIAL_FORWARD_SPEED_PCT);
      if (specialSpeed < MIN_SPEED) specialSpeed = MIN_SPEED;

      driveForwardMaintainYaw(yawZero, specialSpeed);

      // if front sees obstacle closer than FRONT_TRIGGER_DISTANCE_CM, begin turning left 90° to yawZero+90
      // IMPORTANT: only trigger if not in the front ignore cooldown
      if ((filteredFront < (float)FRONT_TRIGGER_DISTANCE_CM) && (millis() > frontIgnoreUntil)) {
        mode = MODE_TURNING;
        Serial.println("MODE -> TURNING (front < threshold)");

        // perform blocking left 90° rotation relative to start yawZero
        rotateLeft90BlockingToStartRef();

        // after rotation, reset PID integral & lastError to avoid transients
        integralTerm = 0.0f;
        lastError = 0.0f;
        lastPidTime = millis();

        // return to normal mode (wall-following)
        mode = MODE_NORMAL;
        Serial.println("MODE -> NORMAL (after turn)");
        // after returning, set small left-ignore to avoid immediate re-entry
        leftIgnoreUntil = millis() + LEFT_IGNORE_AFTER_TURN_MS;
        leftAboveSince = 0;
      }
    }
    else { // MODE_TURNING should not persist since rotateLeft90BlockingToStartRef handles rotation
      // fallback to normal
      mode = MODE_NORMAL;
    }

    // small loop pause
    delay(20);
  }
}

// Helper: read ultrasonic sensor (trigPin,echoPin) and return cm or -1 for timeout
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

// computePidOutput: PID using left filtered distance; returns signed degrees (-MAX..+MAX)
float computePidOutput(float leftMeasured) {
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  float error = (float)targetdistance - leftMeasured; // positive => too far

  integralTerm += error * dt;
  if (integralTerm > I_MAX) integralTerm = I_MAX;
  if (integralTerm < I_MIN) integralTerm = I_MIN;

  float derivative = (error - lastError) / dt;

  float output = Kp * error + Ki * integralTerm + Kd * derivative;

  // clamp
  if (output > MAX_CORRECTION_DEG) output = MAX_CORRECTION_DEG;
  if (output < -MAX_CORRECTION_DEG) output = -MAX_CORRECTION_DEG;

  lastError = error;
  lastPidTime = now;

  return output;
}

// writeServosFromOutput: converts pidOutput to servo positions and writes (rate-limited)
void writeServosFromOutput(float output) {
  unsigned long now = millis();
  if ((now - lastServoUpdate) < servoUpdateInterval) return;

  // NOTE: output < 0 -> RIGHT turn; output > 0 -> LEFT turn
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
