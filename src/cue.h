#pragma once 

#include "position_recorder.h"

struct Cue  : public Params{
  P_string  (cue_name, true, "Unnamed");
  P_uint8_t (cue_running, false, 0, 1, 0);
  P_int32_t (cue_cursor, false, 0, 4000, 0);
  P_string  (cue_script, true, "\n");
  P_bool (cue_record, false, false);
  P_bool (cue_play, false, false);
  P_end;
    
  static uint8_t next_id;
  uint8_t id;
  Cue() {id = next_id++;}  // assign unique id
    
  void update( void (*command_func)(char* command) ) {
  
    if(cue_record) {
      cue_record = false;
      recorder.record(id);
    }
    if(cue_play) {
      cue_play = false;
      recorder.play(id);
    }
  
    if(!cue_running)
      return;

    int32_t end = cue_script.find("\\n", cue_cursor);
    if(end == std::string::npos) {
      // no further commands
      cue_cursor = 0;
      cue_running = 0;
      return;
    }    
    std::string command = cue_script.substr(cue_cursor, end - cue_cursor);
    
    if(command.length() > 2) {
      Serial.printf("Cue command : \"%s\"\n", command.c_str() );
      command_func((char*)command.c_str());
    }
    
    cue_cursor = end + 2; // skip \n
  }  
};

uint8_t Cue::next_id = 0;

