#pragma once

struct Axis : public  Params {
  P_float (ik_target ,false,-1000, 1000, 0);
  P_float (ik_manual ,false,-1000, 1000, 0);
  P_float (ik_offset,true, -1000,  1000, 0);
  P_float (ik_feedback ,false,0,0, 0);
  P_float (ik_osc_a ,true, -1000,  1000, 0);
  P_float (ik_osc_f ,true,     0, 100, 1);
  P_float (ik_osc_fb,true,     0,  2, 0);
  P_float (ik_random_a  ,true, -10000, 10000, 0);
  P_float (ik_random_d  ,true, 0, 60, 0.5f);
  P_float (ik_random_rd ,true, 0, 60, 0.5f);
  P_uint8_t (ik_dmx_ch,true,     0,   255, 10);
  P_float (ik_dmx_a ,true,-10000, 10000, 0);
  P_uint8_t (ik_midi_cc,true,     0,   127, 0);
  P_float (ik_midi_a ,true,-10000, 10000, 0);
  P_float (ik_ext_in,false,0, 0, 0);
  P_float (ik_hid_a  ,true,-10000, 10000, 0);
  P_int32_t (ik_hid_ch ,true,-1, 15, -1);
  P_float (midi_move_a ,true,-1000, 1000, 0);
  P_float (midi_pick_a ,true,-1000, 1000, 0);
  P_float (ik_pos ,false,0, 0, 0);
  P_float (ik_pred_err, false, 0,0,0);
  P_float (ik_pred_thres, true, 0,1000,10);
  P_uint32_t (ik_cp_index, false, 0,ik_cp_length,2);
  P_end;

  uint8_t id;
  static uint8_t next_id;
  
  float vel=0, ik_input=0;
  float ik_phase=0;
  float random_target=0, random_dt=0;
  
  static const uint32_t ik_cp_length = 1024;
  float ik_cp_x[ik_cp_length];  
  float ik_cp_t[ik_cp_length];
  
  Axis() { id = next_id++;}  // assign unique id
  
  float compute_max_vel(float dx_target, float v0, float a_max, float vel_k) {

    // compute time to stop under maximal decelleration
    float dt = abs(v0) / a_max;

    // compute distance travelled under maximal decelleration
    float dx = 1.f / 2.f * a_max * dt * dt;
    
    if(dx >= abs(dx_target)) // we need to brake.
      return 0.f;
    else // we need to accelerate, cruise or dampen close to target.
      return vel_k * dx_target;
  }

  float update(float dt, float vel_max, float acc_max, float vel_k) {
  
    // add external inputs
    ik_input += ik_manual + ik_offset + ik_ext_in;
    // add oscillator
    ik_input += motion_fm_osc(0.f, ik_osc_a, ik_osc_f, ik_osc_fb, dt, ik_phase);
    // add random movement
    ik_input += motion_random(ik_random_a, ik_random_d, ik_random_rd, dt, random_target, random_dt);   
    
    ik_target = ik_input;

    if(dt == 0 || isnan(ik_input)) return ik_pos;

    float dx = ik_input - ik_pos;
    float v_target = compute_max_vel(dx, vel, acc_max, vel_k);
    v_target = min( v_target,  vel_max * 1.f);
    v_target = max( v_target, -vel_max * 1.f);

    float acc = ( v_target - vel ) / dt;
    acc = min( acc,  acc_max);
    acc = max( acc, -acc_max);

    vel += acc * dt;
    ik_pos += vel * dt;

    // update prediction (for record and playback features)
    predict(dt);

    return ik_pos;
  }
  
  static float lerp(float a, float b, float t) {
    return a * (1.f - t) + b * t;
  }
  
  float predict_right() {
    // approximate speed and acceleration for last known time ik_cp_t[0]
    // currently we attribute all linear approximations to the right side eg. the newest point in time.  
    float dt1 = ik_cp_t[0] - ik_cp_t[1];
    float dt2 = ik_cp_t[1] - ik_cp_t[2];
    if(dt1 == 0.f || dt2 == 0.f) dt1 = dt2 = 0.1; // bootstrapping on startup 
    float dx1 = ik_cp_x[0] - ik_cp_x[1];
    float dx2 = ik_cp_x[1] - ik_cp_x[2];
    float v1  = dx1 / dt1; // attribute velocity to newest point
    float v2  = dx2 / dt2;
    float dv1 = v1 - v2; // attribute acceleration to newest point 
    float a1  = dv1 / dt1;
    
    // get prediction
    float dt0 = 0.f - ik_cp_t[0];
    float x0 = ik_cp_x[0] + v1 * dt0 + a1 * dt0 * dt0;
    
    return x0;
  }
  
  float predict_center() {
    // approximate speed and acceleration for last known time ik_cp_t[0]
    // currently we attribute all linear approximations to the right side eg. the newest point in time.  
    uint32_t i = ik_cp_index;
    
    float t1 = ik_cp_t[i], t2 = ik_cp_t[i-1], t3 = ik_cp_t[i-2];  
    
    float dt1 = t1 - t2;
    float dt2 = t2 - t3;
    if(dt1 == 0.f || dt2 == 0.f) dt1 = dt2 = 0.1; // bootstrapping on startup
    float x1 = ik_cp_x[i], x2 = ik_cp_x[i-1], x3 = ik_cp_x[i-2];
    float dx1 = x1 - x2;
    float dx2 = x2 - x3;
    float v1  = dx1 / dt1; // attribute velocities to center points
    float v2  = dx2 / dt2;
    float dt12 = dt1/2 + dt2/2;
    float dv12 = v1 - v2; // attribute acceleration to center point 
    float a12  = dv12 / dt12;
    
    // get prediction
    float dt0 = ik_pred_t - t1;
    float v0 = v1 + a12 * dt1 / 2;
    float x0 = x1 + v0 * dt0 + a12 * dt0 * dt0;
    
    return x0;
  }
  
  float ik_pred_t=0;
  void predict(float dt) {
    
    ik_pred_t += dt;
    float x0 = predict_center();
    
    // compare
    ik_pred_err = x0 - ik_feedback; 
    
    // insert new control point, if error is above threshold
    if(abs(ik_pred_err) > ik_pred_thres) {
      ik_cp_index++;
      if(ik_cp_index >= ik_cp_length) return;
      ik_cp_x[ik_cp_index] = ik_feedback;
      ik_cp_t[ik_cp_index] = ik_pred_t;
      
      Serial.printf("New IK CP #%d on axis %d : %.5g after %.5g s (error was %.5g) \n", ik_cp_index, id, ik_cp_x[ik_cp_index],  ik_cp_t[ik_cp_index] - ik_cp_t[ik_cp_index-1], ik_pred_err);
    }    
  }
};

uint8_t Axis::next_id = 0;
