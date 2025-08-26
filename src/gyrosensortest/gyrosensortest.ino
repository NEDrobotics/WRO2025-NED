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

// IMU yaw globals
float currentYawDeg = 0.0f;            // estimated yaw in degrees (0..360)
float currentYawRateDeg = 0.0f;        // gz in deg/s

// yaw reference at program start
float yawZero = 0.0f;

// servo positions last known
int lastS1pos = s1mid;
int lastS2pos = s2mid;

// Modes
enum Mode { MODE_NORMAL=0, MODE_SPECIAL_FORWARD=1, MODE_TURNING=2 };
Mode mode = MODE_NORMAL;

// helper: angle difference normalized to [-180,180]
float angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
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
  digitalWrite(IN2_1, forward ? LOW  : HIGH);
  analogWrite(ENA_1, speed);
  // Motor 2
  digitalWrite(IN1_2, forward ? LOW : HIGH);
  digitalWrite(IN2_2, forward ? HIGH : LOW);
  analogWrite(ENA_2, speed);
}

// Rotate in place LEFT: left wheel backward, right wheel forward
void rotateLeftInPlace(int pwm) {
  digitalWrite(IN1_1, LOW);  // motor1 backward
  digitalWrite(IN2_1, HIGH);
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

// Blocking rotate left by 90° using yawZero as reference (target = yawZero + 90)
// NOTE: uses gz integration for responsiveness; gz is read from imu.Read() and gbz is subtracted
void rotateLeft90BlockingToStartRef() {
  // Put servos to a left-turn pose so steering geometry is consistent
  int leftServoPose = constrain(s1mid + MAX_CORRECTION_DEG, 0, 180);
  int rightServoPose = constrain(s2mid - MAX_CORRECTION_DEG, 0, 180);
  servo1.write(leftServoPose);
  servo2.write(rightServoPose);
  lastServoUpdate = millis();

  // compute absolute target yaw relative to program start, normalized to 0..360
  float targetYaw = yawZero + 90.0f;
  while (targetYaw >= 360.0f) targetYaw -= 360.0f;
  while (targetYaw < 0.0f) targetYaw += 360.0f;

  Serial.print("rotateLeft90ToStartRef: currentYaw=");
  Serial.print(currentYawDeg);
  Serial.print(" targetYaw=");
  Serial.println(targetYaw);

  unsigned long lastT = micros();

  // start rotation
  rotateLeftInPlace(TURN_PWM);

  while (true) {
    unsigned long nowT = micros();
    float dt = (nowT - lastT) * 1e-6f;
    if (dt <= 0.0f) dt = 0.001f;
    lastT = nowT;

    // refresh gyro reading and integrate z to update yaw
    if (imu.Read()) {
      float gz = imu.gyro_z_radps() - gbz; // rad/s
      const float R2D = 57.295779513082320876f;
      currentYawDeg += gz * dt * R2D;
      // normalize current yaw to 0..360
      while (currentYawDeg >= 360.0f) currentYawDeg -= 360.0f;
      while (currentYawDeg < 0.0f) currentYawDeg += 360.0f;
      currentYawRateDeg = gz * R2D;
    }

    // compute signed shortest error: target - current (angleDiff handles wrap)
    float err = angleDiff(targetYaw, currentYawDeg); // returns -180..180
    if (fabs(err) <= YAW_TOLERANCE_DEG) break;
    Serial.println(currentYawDeg);
    // brief pause
    delay(5);
  }

  // stop motors and settle
  stopMotors();
  delay(TURN_SETTLE_MS);

  // force currentYawDeg to exact targetYaw normalized to 0..360
  float normalized = yawZero + 90.0f;
  while (normalized >= 360.0f) normalized -= 360.0f;
  while (normalized < 0.0f) normalized += 360.0f;
  currentYawDeg = normalized;

  // set servos back to neutral
  servo1.write(s1mid);
  servo2.write(s2mid);
  lastServoUpdate = millis();

  Serial.print("rotateLeft90ToStartRef: done yaw=");
  Serial.println(currentYawDeg);
}

// ---------- Drive forward while maintaining a target yaw ----------
const float YAW_HOLD_KP = 0.6f;     // deg of servo per deg of yaw error (tune)
const float YAW_HOLD_MAX_DEG = 12;  // maximum yaw-based steering output (degrees) to avoid big jumps

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

  // Warm IMU and set yawZero (uses quaternion-derived yaw after a few updates)
  for (int i=0;i<20;i++){
    if (imu.Read()) {
      // integrate Mahony update to keep quaternion usable (we still compute yaw from quaternion below)
      float ax = imu.accel_x_mps2();
      float ay = imu.accel_y_mps2();
      float az = imu.accel_z_mps2();
      float gx = imu.gyro_x_radps() - gbx;
      float gy = imu.gyro_y_radps() - gby;
      float gz = imu.gyro_z_radps() - gbz;

      float vx = ax, vy = ay, vz = az;
      normalize3(vx, vy, vz);

      float v_est_x = 2.0f * (q1*q3 - q0*q2);
      float v_est_y = 2.0f * (q0*q1 + q2*q3);
      float v_est_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;

      float ex = (v_est_y * vz - v_est_z * vy);
      float ey = (v_est_z * vx - v_est_x * vz);
      float ez = (v_est_x * vy - v_est_y * vx);

      gx += gyroKp * ex;
      gy += gyroKp * ey;
      gz += gyroKp * ez;

      float dq0 =  0.5f * ( - q1*gx - q2*gy - q3*gz );
      float dq1 =  0.5f * (   q0*gx + q2*gz - q3*gy );
      float dq2 =  0.5f * (   q0*gy - q1*gz + q3*gx );
      float dq3 =  0.5f * (   q0*gz + q1*gy - q2*gx );

      unsigned long nowUs = micros();
      float dt = (nowUs - lastMicros) * 1e-6f;
      if (dt <= 0 || dt > 0.1f) dt = sampleInterval;
      lastMicros = nowUs;

      q0 += dq0 * dt;
      q1 += dq1 * dt;
      q2 += dq2 * dt;
      q3 += dq3 * dt;
      normalizeQuat();
    }
    delay(10);
  }

  // compute yawZero from quaternion and normalize to 0..360
  {
    float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    const float R2D = 57.295779513082320876f;
    float yaw_deg = yaw * R2D;
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;
    while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
    yawZero = yaw_deg;
    currentYawDeg = yaw_deg; // initialize current yaw
    Serial.print("yawZero set to ");
    Serial.println(yawZero);
  }

  while (true) {
    // ----- IMU integration (Mahony-like) -----
    unsigned long nowUs = micros();
    float dt = (nowUs - lastMicros) * 1e-6f;
    if (dt <= 0 || dt > 0.1f) dt = sampleInterval;
    lastMicros = nowUs;

    if (imu.Read()) {
      float ax = imu.accel_x_mps2();
      float ay = imu.accel_y_mps2();
      float az = imu.accel_z_mps2();

      float gx = imu.gyro_x_radps() - gbx;
      float gy = imu.gyro_y_radps() - gby;
      float gz = imu.gyro_z_radps() - gbz;

      float vx = ax, vy = ay, vz = az;
      normalize3(vx, vy, vz);

      float v_est_x = 2.0f * (q1*q3 - q0*q2);
      float v_est_y = 2.0f * (q0*q1 + q2*q3);
      float v_est_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;

      float ex = (v_est_y * vz - v_est_z * vy);
      float ey = (v_est_z * vx - v_est_x * vz);
      float ez = (v_est_x * vy - v_est_y * vx);

      gx += gyroKp * ex;
      gy += gyroKp * ey;
      gz += gyroKp * ez;

      float dq0 =  0.5f * ( - q1*gx - q2*gy - q3*gz );
      float dq1 =  0.5f * (   q0*gx + q2*gz - q3*gy );
      float dq2 =  0.5f * (   q0*gy - q1*gz + q3*gx );
      float dq3 =  0.5f * (   q0*gz + q1*gy - q2*gx );

      q0 += dq0 * dt;
      q1 += dq1 * dt;
      q2 += dq2 * dt;
      q3 += dq3 * dt;
      normalizeQuat();

      // compute Euler yaw from quaternion and normalize to 0..360
      float yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
      const float R2D = 57.295779513082320876f;
      float yaw_deg = yaw * R2D;
      if (yaw_deg < 0.0f) yaw_deg += 360.0f;
      while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
      currentYawDeg = yaw_deg;

      // yaw rate in deg/s from gyro z
      currentYawRateDeg = gz * R2D;
    }

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
      // If left ultrasonic large -> enter special forward mode
      if (filteredLeft > (float)LEFT_TRIGGER_DISTANCE_CM) {
        mode = MODE_SPECIAL_FORWARD;
        Serial.println("MODE -> SPECIAL_FORWARD (left > threshold)");
      } else {
        // normal PID wall following
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
      if (filteredFront < (float)FRONT_TRIGGER_DISTANCE_CM) {
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
