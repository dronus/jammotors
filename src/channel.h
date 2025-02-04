struct Channel : public Params {

  P_uint8_t (driver_id, 0, 0xFF, 0);
  P_uint8_t (pin_id, 0, 0xFF, 0);
  P_int32_t (poweron_en, 0, 1, 0);
  P_int32_t (speed, 0, 100000, 10000);
  P_int32_t (accel, 0, 100000, 10000);
  P_int32_t (pos_kp, 0, 500000, 1000);
  P_int32_t (pos_kd, 0, 5000, 100);
  P_int32_t (dmx_channel, 0, 255, 0);
  P_int32_t (scale, 0, 10000,  0);
  P_int32_t (offset, -100000, 100000,  0);
  P_int32_t (osc_f, 0, 10000,  1000);
  P_int32_t (osc_a, 0, 100000, 0);
  P_int32_t (osc_p, 0, 100000, 0);
  P_int32_t (random_d, 0, 100000, 1000);
  P_int32_t (random_rd,0, 100000, 1000);
  P_int32_t (random_a ,0, 200000, 0);
  P_int32_t (ik_a ,-100000, 100000, 0);
  P_end;

  int enabled, last_enabled=false;
  int have_alarm;
  bool reset_zero = false;
  int32_t target = 0;
  int32_t position = 0, torque = 0, temperature = 0;
  int32_t manual_target=0;
  int32_t artnet_target=0;
  float ik_target;
  float osc_phase = 0;
  int32_t random_countdown = 0;
  int32_t random_target = 0;
  uint8_t last_driver_id=0, last_pin_id=0;
  Driver* driver=NULL;

  void init() {
    enabled = poweron_en;
  }
  
  void update(uint32_t dt) {

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
    // set manual target (temporary offset) and stored offset.
    target = manual_target + offset;

    // add artnet commanded target
    target += artnet_target;
   
    // add oscillatory movement 
    osc_phase += osc_f * dt * 2.f * (float)PI / 1000.f / 1000.f;
    osc_phase = fmod(osc_phase, ((float)PI * 2.f));
    target += floor( osc_a * sin(osc_phase + osc_p * 2.f * (float)PI / 1000.f) );
    
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

    if(driver)
      driver->update(*this, dt);
    
    last_enabled = enabled;
  }
};

