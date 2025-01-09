// drive PPM-controlled RC servo

#include <ESP32Servo.h>

struct DriverServo : public Driver {
  Servo servo;
  int pin;

  DriverServo(int _pin){ pin=_pin; };
  virtual ~DriverServo() { servo.detach(); }

  void init(Channel& c) {
    servo.setPeriodHertz(50);// Standard 50hz servo
    Serial.println("Servo started.");
  };

  void update(Channel& c, uint32_t dt) {
    if(!c.enabled && c.last_enabled)
      servo.detach();
    if(c.enabled && !c.last_enabled)
      servo.attach(pin, 500, 2400);

    if(c.enabled) {
      servo.write(1000 + max(0,c.target));
      c.position = c.target; // no real feedback possible
    }
  };
};

