#pragma once


struct Axis : public  Params {
  P_float (ik_target ,false,-10000, 10000, 0);
  P_float (ik_manual ,false,-10000, 10000, 0);
  P_float (ik_offset,true, -10000,  10000, 0);
  P_float (ik_feedback ,false,0,0, 0);
  P_float (ik_osc_a ,true, -10000,  10000, 0);
  P_float (ik_osc_f ,true,     0, 100, 1);
  P_float (ik_osc_fb,true,     -2,  2, 0);
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
  P_float (ik_hid_in ,false,-1, 1, 0);
  P_float (midi_move_a ,true,-10000, 10000, 0);
  P_float (midi_pick_a ,true,-10000, 10000, 0);
  P_float (ik_ik_in ,false,0, 0, 0);
  P_float (ik_pos ,false,0, 0, 0);
  P_float (ik_pred_err, false, 0,0,0);
  P_float (ik_pred_thres, true, 0,10000,10);
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
  
  float update(float dt, float vel_max, float acc_max, float vel_k, float crawl_thres, float crawl_vel) {
  
    // add external inputs
    ik_input += ik_manual + ik_offset + ik_ik_in + ik_hid_in * ik_hid_a + ik_ext_in;
    // add oscillator
    ik_input += motion_fm_osc(0.f, ik_osc_a, ik_osc_f, ik_osc_fb, dt, ik_phase);
    // add random movement
    ik_input += motion_random(ik_random_a, ik_random_d, ik_random_rd, dt, random_target, random_dt);   
    
    ik_target = ik_input;

    if(dt == 0 || isnan(ik_input)) return ik_pos;

    // compute distance to target
    float dx = ik_input - ik_pos;
    
     // compute desired spped.     
    float dt_max = abs(vel) / acc_max; // time to stop under maximal decelleration    
    float dx_max = 1.f / 2.f * acc_max * dt_max * dt_max; // distance travelled under maximal decelleration    
    float v_target;
    if(dx_max >= abs(dx)) // we need to brake.
      v_target =  0.f;
    else // we need to accelerate, cruise or dampen close to target.
      v_target = vel_k * dx;
    // limit to range
    float active_vel_max = abs(dx) > crawl_thres ? crawl_vel : vel_max;
    v_target = min( v_target,  active_vel_max);
    v_target = max( v_target, -active_vel_max);

    float acc = ( v_target - vel ) / dt;
    acc = min( acc,  acc_max);
    acc = max( acc, -acc_max);

    vel += acc * dt;
    ik_pos += vel * dt;

    return ik_pos;
  } 
};

uint8_t Axis::next_id = 0;
