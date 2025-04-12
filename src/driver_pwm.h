// drive PWM pin ( DC motors, LEDs, ...)

struct DriverPWM : public Driver {
  ESP32PWM pwm;
  int pin;

  DriverPWM(int _pin){ pin=_pin; };
  virtual ~DriverPWM() { analogWrite(pin,0);}

  void init(Channel& c) {
    analogWriteResolution(10);
    Serial.println("PWM started.");
  };

  void update(Channel& c, uint32_t dt) {
    if(c.enabled) {
      analogWrite(pin, max(0, min(1023,(int)c.target)));
      if(c.target <= 0) // ESP PWM seems to not reach perfect 0V, but digitalWrtie does. 
        digitalWrite(pin, LOW);
      c.position = c.target; // no real feedback possible
    }
  };
};

