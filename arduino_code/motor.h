class Motor {
  private:
    int in1, in2, en;
  public:
    Motor(int in1, int in2, int en):
    in1(in1), in2(in2), en(en){
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(en, OUTPUT); //PWM
    }
    void setup() {
      brake();
    }
    void drive(int s) {
      // s = -255 - 255
      if (s < 0) {
        bwd(-s);
      } else {
        fwd(s);
      }
    }
    void fwd(int s) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(en, s);
    }
    void bwd(int s) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(en, s);
    }
    void brake() {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
};
