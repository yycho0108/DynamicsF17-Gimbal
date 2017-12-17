class PID {
  private:
    unsigned long t;
    float k_p, k_i, k_d;
    float e_i, e_d;

  public:
    PID(float k_p, float k_i, float k_d):
      k_p(k_p), k_i(k_i), k_d(k_d) {
      e_i = 0.0;
      e_d = 0.0;
    }
    void setup() {
      t = 0; //not initialized
    }
    //  float compute(float err, float now){
    //    if(t == 0){
    //      t = now;
    //      e_d = err;
    //      return 0;
    //    }
    //    float dt = (now - t) / 1000.;
    //    float val = k_i * e_i + k_p * err + k_d * (err-e_d)/dt;
    //
    //    // accumulate + save
    //    e_i += err * dt;
    //    e_d = err;
    //    t = now;
    //
    //    return val;
    //  }
    float compute_2(float ex, float ev, float now) {
      if(t == 0){
        t = now;
        e_d = ex;
        return 0;
      }
      float dt = (now - t) / 1000.;
      float val = k_i * e_i + k_p * ex + k_d * ev;
      //float val = k_p * ex + k_d * ev;
      e_i += ex * dt;
      t = now;
      return val;
    }
    void reset(unsigned long t0){
      t = t0;
      k_i = k_d = 0.0;
    }
};
