#pragma once

#include "position_recorder.h"

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
  
  float predict_center(float dt0) {
    // approximate speed and acceleration for last known time ik_cp_t[0]
    // currently we attribute all linear approximations to the right side eg. the newest point in time.  
    Position p1 = recorder.get(0,id), p2=recorder.get(-1,id), p3=recorder.get(-2,id);
    float dt1 = p1.dt, dt2 = p2.dt;
        
    if(dt1 == 0.f || dt2 == 0.f) dt1 = dt2 = 0.1; // bootstrapping on startup
    float x1 = p1.x, x2 = p2.x, x3 = p3.x;
    float dx1 = x1 - x2;
    float dx2 = x2 - x3;
    float v1  = dx1 / dt1; // attribute velocities to center points
    float v2  = dx2 / dt2;
    float dt12 = dt1/2 + dt2/2;
    float dv12 = v1 - v2; // attribute acceleration to center point 
    float a12  = dv12 / dt12;
    
    // get prediction    
    float v0 = v1 + a12 * dt1 / 2;
    float x0 = x1 + v0 * dt0 + a12 * dt0 * dt0;
    
    return x0;
  }
  
  
  void predict(float dt) {

    // predict
    float x0 = predict_center(recorder.dt);
    // compare
    ik_pred_err = x0 - (ik_feedback - ik_offset); 
    
    // on recording, insert new control point, if error is above threshold
    
    if(recorder.recording && id >= 3) // only record wrist and IK axes for now.
      recorder.put(id, ik_feedback - ik_offset, abs(ik_pred_err) > ik_pred_thres);
      
    if(recorder.recording && abs(ik_pred_err) > ik_pred_thres) {
            
      Serial.printf("New IK CP #%d on axis %d : %.5g after %.5g s (error was %.5g) \n", recorder.index, id, recorder.get(0,id).x, recorder.get(0,id).dt, ik_pred_err);      
    }
    
    // on playback, set manual axis input and advance on need
    if(recorder.playback)
      ik_manual = x0;          
  }
};

uint8_t Axis::next_id = 0;
