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
  P_float   (ik_midi_target ,false,0, 0, 0);
  P_float (ik_hid_a  ,true,-10000, 10000, 0);
  P_int32_t (ik_hid_ch ,true,-1, 15, -1);
  P_float (midi_move_a ,true,-1000, 1000, 0);
  P_float (midi_pick_a ,true,-1000, 1000, 0);
  P_float (ik_pos ,false,0, 0, 0);
  P_end;

  float vel=0, ik_input=0;
  float ik_phase=0;
  float ik_dmx_target=0;
  float random_target=0, random_dt=0;
  
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
    ik_input += ik_manual + ik_offset + ik_dmx_target + ik_midi_target;
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

    return ik_pos;
  }
};
