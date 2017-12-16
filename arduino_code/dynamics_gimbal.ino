//#define ENCODER_DO_NOT_USE_INTERRUPTS

#include "encoders.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"

#define ENC_X_A 2
#define ENC_X_B 4
#define ENC_Y_A 3
#define ENC_Y_B 5

//#define IMU_INT 12
#define IMU_MAG_CAL false

#define M_X_EN 6
#define M_X_IN1 7
#define M_X_IN2 8

#define M_Y_EN 9
#define M_Y_IN1 10
#define M_Y_IN2 11

#define M_K_E 0.457
#define M_K_T 0.265
#define M_R 3.3

#define I_X 0.0110680054487
#define I_Y 0.0110680054487
#define I_Z 0.000819922375447

#define ABORT_PIN 12

#ifndef M_PI
#define M_PI 3.1415926535F
#endif

// Encoder : -X, +Y
// IMU : +X, +Y
// Motor : -X, +Y

EncoderManager enc_mgr(ENC_X_A, ENC_X_B, ENC_Y_A, ENC_Y_B);
IMUManager imu_mgr;

Motor motor_x(M_X_IN1, M_X_IN2, M_X_EN);
Motor motor_y(M_Y_IN1, M_Y_IN2, M_Y_EN);

PID pid_x(30.0, 15.16, 1.12);
PID pid_y(20.05, 3.16, 5.07);

unsigned long t_setup;
bool good = true;
float roll_0, pitch_0;

void setup() {
  Serial.begin(9600);
  enc_mgr.setup();
  imu_mgr.setup(IMU_MAG_CAL);
  motor_x.setup();
  motor_y.setup();
  pid_x.setup();
  pid_y.setup();
  t_setup = millis();

  pinMode(ABORT_PIN, INPUT);
  roll_0 = 0;
  pitch_0 = 0;
}

float get_Ix(float y) {
  float cy = cos(y);
  float sy = sin(y);
  return I_X * abs(cy) + I_Z * abs(sy);
}

int32_t get_motor_pwm(float w, float a, float I) {
  //return int32_t( (255/9.0) * (M_K_E * w + I * a * M_R / M_K_T));
  float res = (255 / 9.0) * (I * a * M_R / M_K_T);
  if (res > 255) res = 255;
  if (res < -255) res = -255;
  if (0 < res && res < 17) res = 20; // artificially kill deadzone?
  if (res > -17 && res < 0) res = -20;
  return res;
}

float get_angle(float x) {
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0) x += 2 * M_PI;
  return x - M_PI;
}

void loop() {
  if (digitalRead(ABORT_PIN) == HIGH) {
    good = false;
  }

  if (!good) {
    motor_x.brake();
    motor_y.brake();
    Serial.println("Aborted!");
    return;
  }

  unsigned long now = millis();
  enc_mgr.loop(now);

  float enc_x = -enc_mgr.x_prv * 2 * M_PI / TICKS_PER_CYCLE;
  float enc_y = enc_mgr.y_prv * 2 * M_PI / TICKS_PER_CYCLE;

  //  Serial.print(enc_x);
  //  Serial.print(',');
  //  Serial.print(enc_y);
  //  Serial.print("||");

  imu_mgr.read(now);

  float imu_wx = imu_mgr.imu.gx * DEG_TO_RAD;
  float imu_wy = imu_mgr.imu.gy * DEG_TO_RAD;

  //  Serial.print(imu_wx);
  //  Serial.print(',');
  //  Serial.print(imu_wy);
  //  Serial.print("||");
  //
  //  Serial.print(imu_mgr.imu.roll * DEG_TO_RAD);
  //  Serial.print(',');
  //  Serial.print(imu_mgr.imu.pitch * DEG_TO_RAD);
  //  Serial.print("||");

  float x = get_angle(imu_mgr.imu.roll * DEG_TO_RAD);
  float y = get_angle(imu_mgr.imu.pitch * DEG_TO_RAD);

  //float x = get_angle(roll_0 + enc_x);
  //float y = get_angle(pitch_0 + enc_y);

  //
  //  Serial.print(x);
  //  Serial.print(',');
  //  Serial.println(y);

  float w_x = imu_wx; //TODO : filter
  float w_y = imu_wy;

  //  int32_t pwm_x = -x * RAD_TO_DEG; //180 deg. == 255??
  //  int32_t pwm_y = -y * RAD_TO_DEG;

  float u_x = -pid_x.compute_2(x, w_x, now); // u = -Kx
  float u_y = -pid_y.compute_2(y, w_y, now);

  Serial.print(u_x);
  Serial.print(',');
  Serial.print(u_y);
  Serial.print("||");

  int32_t pwm_x = -get_motor_pwm(w_x, u_x, get_Ix(y));
  int32_t pwm_y = get_motor_pwm(w_y, u_y, I_Y);

  Serial.print(pwm_x);
  Serial.print(',');
  Serial.print(pwm_y);
  Serial.println(';');

  if (now - t_setup < 500) {
    // initial calibration
    roll_0 = imu_mgr.imu.roll * DEG_TO_RAD;
    pitch_0 = imu_mgr.imu.pitch * DEG_TO_RAD;
    return;
  }

  //motor_x stalled at ~35
  motor_x.drive(pwm_x); // pos = clk
  motor_y.drive(pwm_y); // neg = counter-clk
  //  motor_x.drive(0); // pos = clk
  //  motor_y.drive(0); // neg = counter-clk
}
