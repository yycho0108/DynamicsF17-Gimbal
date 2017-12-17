//#define ENCODER_DO_NOT_USE_INTERRUPTS

#include "encoders.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
//#include "kalman.h"

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

#define DRIVE
#define ABORT_PIN 12
#ifndef M_PI
#define M_PI 3.1415926536F
#endif

//#define DRIVE
#define MOTOR_LIM 255
#define STARTUP_DELAY 3000

// Encoder : -X, +Y
// IMU : +X, +Y
// Motor : -X, +Y

EncoderManager enc_mgr(ENC_X_A, ENC_X_B, ENC_Y_A, ENC_Y_B);
IMUManager imu_mgr;

Motor motor_x(M_X_IN1, M_X_IN2, M_X_EN);
Motor motor_y(M_Y_IN1, M_Y_IN2, M_Y_EN);

PID pid_x(1.2, 1.5, 0.12);
PID pid_y(0.8, 0.03, 0.02);
//KalmanFilter KF;

unsigned long t_setup;
bool abort_drive = false;

void setup() {
  Serial.begin(9600);
  enc_mgr.setup();
  imu_mgr.setup(IMU_MAG_CAL);
  motor_x.setup();
  motor_y.setup();
  pid_x.setup();
  pid_y.setup();
  t_setup = millis();
  //KF.setup(t_setup);

  pinMode(ABORT_PIN, INPUT);
  abort_drive = false;
}

float get_Ix(float y) {
  float cy = cos(y);
  float sy = sin(y);
  return I_X * abs(cy) + I_Z * abs(sy);
}

int32_t get_motor_pwm(float w, float u, float I) {
  //return int32_t( (255/9.0) * (M_K_E * w + I * a * M_R / M_K_T));
  float res = (255.0 / 9.0) * (u * M_R / M_K_T);
  //float res = (255.0 / 9.0) * (u * M_R / M_K_T);

  if (res > MOTOR_LIM) res = MOTOR_LIM;
  if (res < -MOTOR_LIM) res = -MOTOR_LIM;

  if (0 < res && res < 20) res = 20;
  if (res > -20 && res < 0) res = -20;

  return res;
}

float get_angle(float x) {
  x = fmod(x + 3 * M_PI, 2 * M_PI);
  if (x < 0) x += 2 * M_PI;
  return x - M_PI;
}

void loop() {
  if (digitalRead(ABORT_PIN) == HIGH) {
    abort_drive = true;
    Serial.println("Aborted!");
  }

  if (abort_drive) {
    motor_x.brake();
    motor_y.brake();
    return;
  }

  unsigned long now = millis();

  // encoder
  enc_mgr.loop(now);
  float motor_wx = -enc_mgr.x_vel;
  float motor_wy = enc_mgr.y_vel;

  //  Serial.print(enc_x);
  //  Serial.print(',');
  //  Serial.print(enc_y);
  //  Serial.print("||");

  // imu
  imu_mgr.read(now);
  float x = get_angle(imu_mgr.imu.roll * DEG_TO_RAD);
  float y = get_angle(imu_mgr.imu.pitch * DEG_TO_RAD);
  float imu_wx = imu_mgr.imu.gx * DEG_TO_RAD;
  float imu_wy = imu_mgr.imu.gy * DEG_TO_RAD;

  float z[4] = {x, y, imu_wx, imu_wy};
  
//  KF.predict(now);
//  KF.update(z);
//
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print("||");
//  Serial.print(',');
//  Serial.print(imu_wx);
//  Serial.print(',');
//  Serial.print(imu_wy);
//  Serial.print(',');
//
//  Serial.print("||");
//  
//  float kx = get_angle(KF.get_x()[0]);
//  float ky = get_angle(KF.get_x()[1]);
//  
//  Serial.print(kx);
//  Serial.print(',');
//  Serial.print(ky);
//  Serial.print(',');
//  Serial.print(KF.get_x()[2]);
//  Serial.print(',');
//  Serial.println(KF.get_x()[3]);

  float w_x = imu_wx; //TODO : filter
  float w_y = imu_wy;

  float u_x = -pid_x.compute_2(x, imu_wx, now); // u = -Kx, Torque
  float u_y = -pid_y.compute_2(y, imu_wy, now);

//  Serial.print(u_x);
//  Serial.print(',');
//  Serial.print(u_y);
//  Serial.print("||");

  int32_t pwm_x = -get_motor_pwm(motor_wx, u_x, get_Ix(y));
  int32_t pwm_y = get_motor_pwm(motor_wy, u_y, I_Y);

  Serial.print(pwm_x);
  Serial.print(',');
  Serial.print(pwm_y);
  Serial.println(';');

  if (now - t_setup < STARTUP_DELAY) {
    pid_x.reset(now);
    pid_y.reset(now);
    // initial calibration
    return;
  }

#ifdef DRIVE
  motor_x.drive(pwm_x); // pos = clk
  motor_y.drive(pwm_y); // neg = counter-clk
#endif

}
