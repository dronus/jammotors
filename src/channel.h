#pragma once 
#include "motion.h"

struct Channel : public Params {

  P_uint8_t (driver_id,true, 0, 0xFF, 0);
  P_uint8_t (pin_id, true,0, 0xFF, 0);
  P_uint8_t (poweron_en, true,0, 1, 0);
  P_float (speed, true,0, 100000, 10000);
  P_float (accel, true,0, 100000, 10000);
  P_float (pos_kp, true,0, 500000, 10);
  P_float (pos_kd, true,0, 5000, 1);
  P_float (ik_a  ,true,-100000, 100000, 0);
  P_float (pos_min ,true,-100000, 100000, -1000);
  P_float (pos_max ,true,-100000, 100000,  1000);
  P_uint8_t (enabled   ,false,0,1,0);
  P_uint8_t (alarm,false,0,1,0);
  P_uint8_t (reset_zero,false,0,1,0);
  P_float (target ,false,0,0,0);
  P_float (position ,false,0,0,0);
  P_float (torque ,false,0,0,0);
  P_float (torque_in,false,-100000,100000,0);
  P_float (temperature ,false,0,0,0);
  P_uint8_t (set_can_id ,true,0,255,0);
  
  P_end;

  int last_enabled=false;
    
  float ik_target;
  float osc_phase = 0;
  float random_countdown = 0;
  float random_target = 0;
  uint8_t last_driver_id=0, last_pin_id=0;
  Driver* driver=NULL;

  void init() {
    enabled = poweron_en;
  }
  
  void update(float dt) {

    // check if driver is still up-to-date 
    // and reinitialize if needed
    if(driver_id != last_driver_id || pin_id != last_pin_id) {
      if(driver) delete driver;
      driver = createDriver(driver_id,pin_id);
      last_driver_id = driver_id;
      last_pin_id = pin_id;
      if(driver) driver->init(*this);
    }

    // compute and set motion target
    target = min(max(ik_target * ik_a, pos_min),pos_max); 
   
    if(!last_enabled && enabled)
      alarm = 0;

    if(driver)
      driver->update(*this, dt);
    else if(enabled)
      position = target;

    last_enabled = enabled;
  }
};

