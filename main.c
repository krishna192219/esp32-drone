// ============================================================
//  ESP32-C3 SuperMini — Whoop Flight Controller
//  Motors : GPIO 10 (FL, CW)  GPIO 3  (FR, CCW)
//           GPIO 0  (RL, CCW) GPIO 1  (RR, CW)
//  IMU    : MPU-6050 via I2C  (SDA=8, SCL=9)
//  NRF24  : CE=7, CSN=2, SCK=4, MISO=5, MOSI=6
// ============================================================

#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ── Motor pins ───────────────────────────────────────────────
#define PIN_FL 10
#define PIN_FR 3
#define PIN_RL 0
#define PIN_RR 1

// ── I2C pins ─────────────────────────────────────────────────
#define SDA_PIN 8
#define SCL_PIN 9

// ── NRF24 pins ───────────────────────────────────────────────
#define CE_PIN   7
#define CSN_PIN  2
#define SCK      4
#define MISO     5
#define MOSI     6

// ── PWM config ───────────────────────────────────────────────
#define PWM_FREQ 24000
#define PWM_BITS 8

// ── Loop timing ──────────────────────────────────────────────
#define LOOP_US 2000  // 500 Hz

// ── Complementary filter ─────────────────────────────────────
#define CF_ALPHA 0.98f

// ── Calibration ──────────────────────────────────────────────
#define CAL_SAMPLES 2000

// ── Safety ───────────────────────────────────────────────────
#define THROTTLE_MIN 30
#define THROTTLE_MAX 150
#define ANGLE_LIMIT  45.0f
#define RC_TIMEOUT   500   // ms before signal-loss disarm

// ─────────────────────────────────────────────────────────────
//  RC struct  — must match TX exactly
// ─────────────────────────────────────────────────────────────
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "RC001";

struct Data {
  int16_t j1x, j1y, j2x, j2y;
  bool b1, b2, t1, t2;
};
Data rcData;
uint32_t lastPacket = 0;

// ─────────────────────────────────────────────────────────────
//  PID
// ─────────────────────────────────────────────────────────────
struct PIDController {
  float kp, ki, kd;
  float i_limit;

  float integral   = 0;
  float prev_error = 0;

  float compute(float setpoint, float measured, float dt) {
    float error      = setpoint - measured;
    integral        += error * dt;
    integral         = constrain(integral, -i_limit, i_limit);
    float derivative = (error - prev_error) / dt;
    prev_error       = error;
    return kp * error + ki * integral + kd * derivative;
  }

  void reset() { integral = 0; prev_error = 0; }
};

//                           Kp     Ki      Kd    I-limit
PIDController pidRoll  = { 5.5f,  0.5f,  1.80f, 200.0f };
PIDController pidPitch = { 5.5f,  0.5f,  1.80f, 200.0f };
PIDController pidYaw   = { 3.0f,  0.01f, 1.00f, 100.0f };

// ─────────────────────────────────────────────────────────────
//  Global state
// ─────────────────────────────────────────────────────────────
MPU6050 mpu;

float off_ax, off_ay, off_az;
float off_gx, off_gy, off_gz;

float roll_angle  = 0.0f;
float pitch_angle = 0.0f;
float roll_rate, pitch_rate, yaw_rate;

float sp_roll      = 0.0f;
float sp_pitch     = 0.0f;
float sp_yaw_rate  = 0.0f;
float base_throttle = 0.0f;

bool armed = false;

// ─────────────────────────────────────────────────────────────
//  Motor helpers
// ─────────────────────────────────────────────────────────────
void motorsInit() {
  ledcAttach(PIN_FL, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_FR, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_RL, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_RR, PWM_FREQ, PWM_BITS);
  motorsWrite(0, 0, 0, 0);
}

void motorsWrite(int fl, int fr, int rl, int rr) {
  ledcWrite(PIN_FL, constrain(fl, 0, 255));
  ledcWrite(PIN_FR, constrain(fr, 0, 255));
  ledcWrite(PIN_RL, constrain(rl, 0, 255));
  ledcWrite(PIN_RR, constrain(rr, 0, 255));
}

// ─────────────────────────────────────────────────────────────
//  Motor mixer
//  FL(10) CW  : +pitch -roll -yaw
//  FR(3)  CCW : +pitch +roll +yaw
//  RL(0)  CCW : -pitch -roll +yaw
//  RR(1)  CW  : -pitch +roll -yaw
// ─────────────────────────────────────────────────────────────
void mixer(float throttle, float roll, float pitch, float yaw) {
  int fl = (int)(throttle + pitch - roll - yaw);
  int fr = (int)(throttle + pitch + roll + yaw);
  int rl = (int)(throttle - pitch - roll + yaw);
  int rr = (int)(throttle - pitch + roll - yaw);
  motorsWrite(fl, fr, rl, rr);
}

// ─────────────────────────────────────────────────────────────
//  IMU calibration
// ─────────────────────────────────────────────────────────────
void calibrateIMU() {
  Serial.println("[CAL] Keep drone still...");
  delay(500);

  long sum_ax=0, sum_ay=0, sum_az=0;
  long sum_gx=0, sum_gy=0, sum_gz=0;
  int16_t a[3], g[3];

  for (int i = 0; i < CAL_SAMPLES; i++) {
    mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
    sum_ax += a[0]; sum_ay += a[1]; sum_az += a[2];
    sum_gx += g[0]; sum_gy += g[1]; sum_gz += g[2];
    delayMicroseconds(2000);
  }

  off_ax = (float)sum_ax / CAL_SAMPLES;
  off_ay = (float)sum_ay / CAL_SAMPLES;
  off_az = (float)sum_az / CAL_SAMPLES - 16384.0f;
  off_gx = (float)sum_gx / CAL_SAMPLES;
  off_gy = (float)sum_gy / CAL_SAMPLES;
  off_gz = (float)sum_gz / CAL_SAMPLES;

  roll_angle  = 0.0f;
  pitch_angle = 0.0f;

  Serial.printf("[CAL] Done. gx=%.1f gy=%.1f gz=%.1f\n", off_gx, off_gy, off_gz);
}

// ─────────────────────────────────────────────────────────────
//  IMU read + complementary filter
// ─────────────────────────────────────────────────────────────
void updateIMU(float dt) {
  int16_t raw[6];
  mpu.getMotion6(&raw[0], &raw[1], &raw[2], &raw[3], &raw[4], &raw[5]);

  float ax = (raw[0] - off_ax) / 16384.0f;
  float ay = (raw[1] - off_ay) / 16384.0f;
  float az = (raw[2] - off_az) / 16384.0f;

  roll_rate  = (raw[3] - off_gx) / 131.0f;
  pitch_rate = (raw[4] - off_gy) / 131.0f;
  yaw_rate   = (raw[5] - off_gz) / 131.0f;

  float roll_acc  = atan2f(ay, az) * RAD_TO_DEG;
  float pitch_acc = atan2f(-ax, az) * RAD_TO_DEG;

  roll_angle  = CF_ALPHA * (roll_angle  + roll_rate  * dt) + (1.0f - CF_ALPHA) * roll_acc;
  pitch_angle = CF_ALPHA * (pitch_angle + pitch_rate * dt) + (1.0f - CF_ALPHA) * pitch_acc;
}

// ─────────────────────────────────────────────────────────────
//  RC handler
// ─────────────────────────────────────────────────────────────
int16_t deadzone(int16_t val, int center = 512, int zone = 20) {
  return (abs(val - center) < zone) ? center : val;
}

void handleRC() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(rcData));
    lastPacket = millis();

    static bool lastB2 = false;
        if (rcData.b2 && !lastB2) {
      if (!armed) {
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
        // NO calibrateIMU() here anymore
        lastPacket = millis();
        armed = true;
        Serial.println("[ARM] ARMED");
      } else {
        armed = false;
        motorsWrite(0, 0, 0, 0);
        Serial.println("[DISARM] via B2");
      }
    }
    lastB2 = rcData.b2;

    base_throttle = map(rcData.j2y, 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
    sp_roll       = map(deadzone(rcData.j1x), 0, 1023, -30, 30);
    sp_pitch      = map(deadzone(rcData.j1y), 0, 1023, -30, 30);
    sp_yaw_rate   = map(deadzone(rcData.j2x), 0, 1023, -90, 90);
  }

  // Signal loss — only triggers after 5 seconds of no packets
  // Increase this if still triggering false positives
  if (armed && (millis() - lastPacket > 5000)) {
    armed = false;
    motorsWrite(0, 0, 0, 0);
    Serial.println("[SAFETY] RC lost — DISARMED");
  }
}

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32-C3 Drone FC ===");

  motorsInit();

  // IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERR] MPU6050 not found!");
    while (true) delay(1000);
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  Serial.println("[INIT] MPU6050 OK");

  // NRF24 — exactly matching working standalone code
  SPI.begin(SCK, MISO, MOSI, -1);
  delay(100);
  if (!radio.begin()) {
    Serial.println("[ERR] NRF24 not found!");
    while (true) delay(1000);
  }
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(120);
  radio.setPayloadSize(sizeof(Data));
  radio.openReadingPipe(1, address);
  radio.startListening();
  Serial.println("[INIT] NRF24 OK");
  Serial.println("[INIT] Press B1 to arm");

  Serial.println("[INIT] Calibrating IMU — keep still...");
  calibrateIMU();
  Serial.println("[INIT] Ready. Press B2 to arm.");
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop() {
  static uint32_t last_us = 0;
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;

  handleRC();  // ← move here, runs every loop regardless of timing

  if (dt < (LOOP_US * 1e-6f)) return;
  last_us = now;
  dt = constrain(dt, 0.0005f, 0.01f);

  updateIMU(dt);

  if (!armed) {
    motorsWrite(0, 0, 0, 0);
  } else {
    if (fabsf(roll_angle) > ANGLE_LIMIT || fabsf(pitch_angle) > ANGLE_LIMIT) {
      armed = false;
      motorsWrite(0, 0, 0, 0);
      Serial.printf("[SAFETY] Tilt disarm! R=%.1f P=%.1f\n", roll_angle, pitch_angle);
    } else {
      float out_roll  = constrain(pidRoll.compute(sp_roll,    roll_angle,  dt), -80, 80);
      float out_pitch = constrain(pidPitch.compute(sp_pitch,  pitch_angle, dt), -80, 80);
      float out_yaw   = constrain(pidYaw.compute(sp_yaw_rate, yaw_rate,    dt), -40, 40);

      if (base_throttle < THROTTLE_MIN) {
        motorsWrite(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN);
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
      } else {
        mixer(base_throttle, out_roll, out_pitch, out_yaw);
      }
    }
  }

  // Telemetry every 200ms
  static uint32_t last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    Serial.println("─────────────────────────────");
    Serial.printf("  State  → %s\n", armed ? "ARMED" : "DISARMED");
    Serial.printf("  Angle  → Roll: %+.1f°  Pitch: %+.1f°  Yaw: %+.1f°/s\n",
                  roll_angle, pitch_angle, yaw_rate);
    Serial.printf("  RC     → Thr:%.0f  R:%.1f  P:%.1f  Y:%.1f\n",
                  base_throttle, sp_roll, sp_pitch, sp_yaw_rate);
    Serial.println("─────────────────────────────");
  }
}
