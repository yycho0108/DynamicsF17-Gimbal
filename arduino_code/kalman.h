#define DIM_X 4 //state
#define DIM_Z 4 //measurement

#include "mat.h"

// state =
// [tx, ty, wx, wy]
// measurement =
// [imu_x, imu_y, imu_wx, imu_wy]

float normalize_angle(float x) {
  x = fmod(x + 3 * M_PI, 2 * M_PI);
  if (x < 0) x += 2 * M_PI;
  return x - M_PI;
}

class KalmanFilter {
    float t;
    float x[DIM_X]; // state
    float P[DIM_X][DIM_X]; // covariance
    float Q[DIM_X]; // process noise
    float R[DIM_Z][DIM_Z]; // measurement noise

    float P_n[DIM_X][DIM_X];
    float y[DIM_Z]; // error residual placeholder
    float dx[DIM_X];

    float S[DIM_Z][DIM_Z];
    float S_i[DIM_Z][DIM_Z];
    float K[DIM_Z][DIM_X];
  public:
    KalmanFilter() {
      memset(x, 0, sizeof(x));
      memset(P, 0, sizeof(P));
      memset(Q, 0, sizeof(Q));
      memset(R, 0, sizeof(R));

      // process noise
      Q[0] = 0.041;
      Q[1] = 0.041;
      Q[2] = 0.2;
      Q[3] = 0.2;

      // msmt
      R[0][0] = 2 * DEG_TO_RAD;
      R[1][1] = 2 * DEG_TO_RAD;
      R[2][2] = 0.1;
      R[3][3] = 0.1;
    }
    void setup(float t) {
      this->t = t;
    }
    void predict(unsigned long now) {
      float dt = (now - t) / 1000.0;
      // assume u = [ax, ay], predivided by I
      x[0] = normalize_angle(x[0] + x[2] * dt); //tx += wx*dt
      x[1] = normalize_angle(x[1] + x[3] * dt); //ty += wy*dt

      // ignore ctrl input for now ...
      // x[2] += a[0] * dt;
      // x[3] += a[1] * dt;

      // transfer
      P_n[0][0] = P[0][0] + dt * (dt * P[2][2] + P[0][2]);
      P_n[0][1] = P[0][1] + dt * (dt * P[2][3] + P[0][3]);
      P_n[0][2] = P[0][2] + dt * P[2][2];
      P_n[0][3] = P[0][3] + dt * P[2][3];
      P_n[1][0] = P[1][0] + dt * (dt * P[3][2] + P[1][2]);
      P_n[1][1] = P[1][1] + dt * (dt * P[3][3] + P[1][3]);
      P_n[1][2] = P[1][2] + dt * P[3][2];
      P_n[1][3] = P[1][3] + dt * P[3][3];
      P_n[2][0] = P[2][0] + dt * P[2][2];
      P_n[2][1] = P[2][1] + dt * P[2][3];
      P_n[2][2] = P[2][2];
      P_n[2][3] = P[2][3];
      P_n[3][0] = P[3][0] + dt * P[3][2];
      P_n[3][1] = P[3][1] + dt * P[3][3];
      P_n[3][2] = P[3][2];
      P_n[3][3] = P[3][3];
      memcpy(P, P_n, sizeof(P));

      // process noise - constsant velocity model Q
      P[0][0] += Q[0] * (dt * dt * dt / 3) + Q[2] * (dt * dt / 2);
      P[1][1] += Q[1] * (dt * dt * dt / 3) + Q[3] * (dt * dt / 2);
      P[2][2] += Q[0] * (dt * dt / 2) + Q[2] * (dt);
      P[3][3] += Q[1] * (dt * dt / 2) + Q[3] * (dt);
    }
    void update(const float* z) {
      y[0] = normalize_angle(z[0] - x[0]);
      y[1] = normalize_angle(z[1] - x[1]);
      y[2] = z[2] - x[2];
      y[3] = z[3] - x[3];

      memcpy(S, P, sizeof(S));

      // S = H*P*H.T + R, skipping here because H = eye(4)
      // assume diagonal R
      S[0][0] += R[0][0];
      S[1][1] += R[1][1];
      S[2][2] += R[2][2];
      S[3][3] += R[3][3];

      m_inv((const float*)S, (float*)S_i);
      m_dot((const float*)P, (const float*)S_i, (float*)K, DIM_X, DIM_X, DIM_X);
      m_dot((const float*)K, (const float*)y, (float*)dx, DIM_X, 1, DIM_X); // dx = K*y

      x[0] = normalize_angle(x[0] + dx[0]);
      x[1] = normalize_angle(x[1] + dx[1]);
      x[2] += dx[2];
      x[3] += dx[3];
    }
    float* get_x(){
      return x;
    }
};
