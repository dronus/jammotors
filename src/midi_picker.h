#include "param.h"
#include "axis.h"

struct MidiPicker : public Params{

  P_int32_t (midi_base_note,true,0,96,48);
  P_int32_t (midi_move_dur,true,0,10000,1000);
  P_int32_t (midi_pick_dur,true,0,1000 , 200);
  P_end;
  
  uint8_t  note;
  uint32_t t;

  void update(Axis (&axes)[4], uint32_t dt) {
  
    if(note == 0) return; // nothing to play

    if(t >= midi_move_dur + midi_pick_dur) { // finished 
      note = 0; t=0;
      return;
    }

    for(Axis& axis : axes) {
      axis.ik_input += axis.midi_move_a * ( note - midi_base_note);  // move towards pick location
      if (t > midi_move_dur && t < midi_move_dur + midi_pick_dur / 2) // if moved long enough, pick down and release again
        axis.ik_input += axis.midi_pick_a;
    }

    t += dt;
  }

  void pick(uint8_t _note) {
    if(t == 0) // ignore if still picking TODO or just move on?
      note = _note;     // initiate pick
  }
};
