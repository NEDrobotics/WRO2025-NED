/*
  EV3-like yaw algorithm implemented for BMI160 (I2C)
  - Zero-rate calibration at startup (N samples)
  - Integrate (rate - bias) * dt to get yaw (deg)
  - While |rate| < STATIONARY_DPS_THRESHOLD, slowly adapt bias (ADAPT_ALPHA)
    to remove long-term drift (mimics EV3 behavior)
  - Serial 'r' resets the reference yaw
*/

#include <Wire.h>
#include <Arduino.h>

const uint8_t BMI160_ADDR = 0x68;    // SA0 tied high -> 0x69
const float    GYRO_FS_DPS  = 250.0; // sensor full scale (deg/s)
const float    GYRO_LSB_PER_DPS = 131.0; // for ±250 dps (LSB/(deg/s))

// Calibration & bias adaptation
const int   CAL_SAMPLES = 600;         // samples for initial bias estimation
const float ADAPT_ALPHA = 0.002f;      // bias adaptation rate when stationary (0..1). smaller -> slower
const float STATIONARY_DPS_THRESHOLD = 1.0f; // deg/s threshold to consider sensor stationary

// State
float gyroBiasRaw = 0.0f;   // bias in raw LSB (not converted)
float yaw_deg = 0.0f;       // integrated yaw in degrees (can roll over beyond 360)
float yaw_ref = 0.0f;       // reference yaw for relative reporting
float prev_fusedYaw = 0.0f; // previous normalized yaw (for unwrap, optional)

// I2C helpers
void bmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
void bmiRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  delay(50);

  // Wake BMI160 (accel + gyro normal)
  bmiWrite(0x7E, 0x11); delay(50);
  bmiWrite(0x7E, 0x15); delay(50);

  // OPTIONAL WHO_AM_I check:
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, (uint8_t)1);
  if (Wire.available()) {
    uint8_t who = Wire.read();
    Serial.print("WHO_AM_I = 0x"); Serial.println(who, HEX);
  }

  Serial.println("Calibrating gyro bias — keep the board perfectly still...");
  // Initial bias calibration (average raw gz)
  long sum = 0;
  uint8_t buf[12];
  for (int i = 0; i < CAL_SAMPLES; ++i) {
    bmiRead(0x0C, buf, 12); // read accel+gyro; gz at bytes 10..11
    int16_t gz_raw = (int16_t)(buf[10] | (buf[11] << 8));
    sum += gz_raw;
    delay(3); // small delay to get many samples in short time (adjust as needed)
  }
  gyroBiasRaw = sum / (float)CAL_SAMPLES;
  Serial.print("Initial gyroBiasRaw = "); Serial.println(gyroBiasRaw, 3);

  // set initial yaw and reference
  yaw_deg = 0.0f;
  yaw_ref = 0.0f;
  prev_fusedYaw = 0.0f;

  Serial.println("Ready. Send 'r' via Serial to reset reference yaw to current yaw.");
}

float normalize180(float ang) {
  while (ang > 180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
}

void loop() {
  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Read accel+gyro
  uint8_t buf[12];
  bmiRead(0x0C, buf, 12);
  // gz is bytes 10 (LSB) and 11 (MSB)
  int16_t gz_raw = (int16_t)(buf[10] | (buf[11] << 8));

  // Convert raw to deg/s (but first subtract raw bias)
  float gz_unbiased_dps = (gz_raw - gyroBiasRaw) / GYRO_LSB_PER_DPS;

  // Integrate to yaw
  yaw_deg += gz_unbiased_dps * dt; // continuous yaw in degrees

  // EV3-like automatic bias adaptation:
  // while the sensor is essentially stationary (|rate| small), slowly move the raw bias
  // toward the measured raw value so that integrated drift reduces over time.
  if (fabs(gz_unbiased_dps) < STATIONARY_DPS_THRESHOLD) {
    // adapt gyroBiasRaw towards the current raw reading (gz_raw)
    gyroBiasRaw = (1.0f - ADAPT_ALPHA) * gyroBiasRaw + ADAPT_ALPHA * (float)gz_raw;
  }
  // else: do not adapt while rotating (prevents corrupting bias during motion)

  // Handle Serial commands: 'r' resets reference
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      yaw_ref = yaw_deg;
      Serial.println("Reference yaw reset.");
    }
  }

  // Compute relative yaw:
  float rel_norm = normalize180(yaw_deg - yaw_ref);  // normalized [-180..180]
  float rel_cont = yaw_deg - yaw_ref;                // continuous total rotation in degrees

  // Print results
  Serial.print("yaw_abs: "); Serial.print(yaw_deg, 2);
  Serial.print("  rel_norm: "); Serial.print(rel_norm, 2);
  Serial.print("  rel_cont: "); Serial.print(rel_cont, 2);
  Serial.print("  gz_dps: "); Serial.print(gz_unbiased_dps, 3);
  Serial.print("  bias_raw: "); Serial.println(gyroBiasRaw, 3);

  // small sleep to avoid flooding serial (dt controls integration)
  delay(10);
}
