#pragma once

struct Axis : public  Params {
  P_int32_t (ik_offset,true, -1000,  1000, 0);
  P_int32_t (ik_feedback ,false,0,0, 0);
  P_int32_t (ik_osc_a ,true, -1000,  1000, 0);
  P_int32_t (ik_osc_f ,true,     0, 10000, 1000);
  P_int32_t (ik_osc_fb,true,     0,  2000, 0);
  P_uint8_t (ik_dmx_ch,true,     0,   255, 10);
  P_int32_t (ik_dmx_a ,true,-10000, 10000, 0);
  P_int32_t (midi_move_a ,true,-1000, 1000, 0);
  P_int32_t (midi_pick_a ,true,-1000, 1000, 0);
  P_end;

  float pos=0, vel=0, target=0;
  float ik_phase=0;
  float ik_dmx_target;
};
