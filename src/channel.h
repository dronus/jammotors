#pragma once 
#include "motion.h"

struct Channel : public Params {

  P_uint8_t (driver_id,true, 0, 0xFF, 0);
  P_uint8_t (pin_id, true,0, 0xFF, 0);
  P_uint8_t (poweron_en, true,0, 1, 0);
  P_float (speed, true,0, 100000, 10000);
  P_float (accel, true,0, 100000, 10000);
  P_float (pos_kp, true,0, 500000, 1000);
  P_float (pos_kd, true,0, 5000, 100);
  P_uint8_t (dmx_channel, true,0, 255, 0);
  P_float (scale, true,0, 10000,  0);
  P_float (offset, true,-100000, 100000,  0);
  P_float (osc_f,true, 0, 100,  1);
  P_float (osc_fb,true, 0, 2,  0);
  P_float (osc_a,true, 0, 100000, 0);
  P_float (random_d, true,0, 3600, 1);
  P_float (random_rd,true,0, 3600, 1);
  P_float (random_a ,true,0, 200000, 0);
  P_float (ik_a ,true,-100000, 100000, 0);
  P_uint8_t (enabled   ,false,0,1,0);
  P_uint8_t (alarm,false,0,1,0);
  P_uint8_t (reset_zero,false,0,1,0);
  P_float (target ,false,0,0,0);
  P_float (position ,false,0,0,0);
  P_float (torque ,false,0,0,0);
  P_float (temperature ,false,0,0,0);
  P_uint8_t (set_can_id ,true,0,255,0);
  
  P_end;

  int last_enabled=false;
    
  float artnet_target=0;
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
    // set by stored offset.
    target = offset;

    // add artnet commanded target
    target += artnet_target;
   
    // add oscillatory movement 

    target += motion_fm_osc(0, osc_a, osc_f, osc_fb, dt, osc_phase);
    
    // add random movement
    random_countdown -= dt;
    if(random_countdown < 0) {
      random_target    = random(random_a);
      random_countdown = random_d + random(random_rd);
    }
    target += random_target;
    
    // add IK movement
    if(ik_a != 0)
      target += ik_a * ik_target;

    if(!last_enabled && enabled)
      alarm = 0;

    if(driver)
      driver->update(*this, dt);
    else if(enabled)
      position = target;

    last_enabled = enabled;
  }
};

