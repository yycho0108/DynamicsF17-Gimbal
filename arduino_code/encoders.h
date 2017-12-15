#include <Encoder.h>

#define ENC_LOOP (10)
#define TICKS_PER_CYCLE 2796

class EncoderManager {
  private:
    unsigned long t_prv;
    Encoder enc_x;
    Encoder enc_y;
  public:
  
    int32_t x_prv;
    float x_vel;
    int32_t y_prv;
    float y_vel;

  public:
    EncoderManager(int x_a, int x_b, int y_a, int y_b):
      enc_x(x_a, x_b), enc_y(y_a, y_b) {
      pinMode(x_a, INPUT);
      pinMode(x_b, INPUT);
      pinMode(y_a, INPUT);
      pinMode(y_b, INPUT);
    }

    void setup() {
      t_prv = millis();
      x_prv = 0;
      y_prv = 0;
    }

    void read(float dt) {
      int32_t x_pos = enc_x.read();
      int32_t dx = (x_pos - x_prv);

      int32_t y_pos = enc_y.read();
      int32_t dy = (y_pos - y_prv);

      if (dt > 0) {
        if (dx < -32768) dx += 65536;
        if (dx > 32768) dx -= 65536;
        x_vel = (dx / dt) / TICKS_PER_CYCLE;
        x_prv = x_pos;
        y_vel = (dy / dt) / TICKS_PER_CYCLE;
        y_prv = y_pos;
      }
    }

    void loop(unsigned long now) {
      float dt = now - t_prv;
      if (dt >= ENC_LOOP) {
        read(dt / 1000.); // sec.
        t_prv = now;
      }
    }
};
