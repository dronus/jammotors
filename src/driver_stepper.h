// drive stepper motors via step, dir pins

#include "FastAccelStepper.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

struct DriverStepper : public Driver {
  FastAccelStepper *stepper = NULL;

  const uint8_t pinStep = 16, pinDir = 17, pinEnable = 21, pinAlarm = 22;

  virtual ~DriverStepper() {
    if(stepper) {
      stepper->forceStop();
      stepper->detachFromPin();
      delete stepper;
      pinMode(pinEnable,INPUT);
    }
  }

  // simple external pin handler - the FastAccelStepper original one does not like other interrupts.
  static bool setExternalPin(uint8_t pin, uint8_t value) {
    pin = pin & ~PIN_EXTERNAL_FLAG;
    pinMode(pin, OUTPUT);
    bool oldValue = digitalRead(pin);
    digitalWrite(pin, value);
    return oldValue;
  }

  void init(Channel& c) {
    // start stepper driver
    engine.init();
    stepper = engine.stepperConnectToPin(pinStep, DRIVER_MCPWM_PCNT);
    stepper->setDirectionPin( pinDir + PIN_EXTERNAL_FLAG ); // use external pin handler
    engine.setExternalCallForPin(DriverStepper::setExternalPin);
    pinMode(pinEnable, OUTPUT);
    Serial.println("Stepper started.");
  };

  void update(Channel& c, uint32_t dt) {
    if(c.reset_zero) {
      stepper->setPositionAfterCommandsCompleted(0);
      c.reset_zero = false;
    }
    digitalWrite(pinEnable, c.enabled);
    stepper->setSpeedInHz(c.speed);
    stepper->setAcceleration(c.accel);
    stepper->moveTo(c.target);
    c.position = stepper->getCurrentPosition();
  };
};
