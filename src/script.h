#pragma once 

struct ScriptRuntime : public Params{
  P_float (wait, false, 0,60000, 1);
  P_float (wait_thres, false, 0,10000, 0);
  P_end;
} scriptRuntime;

struct Script  : public Params{
  P_string  (s_name, true, "Unnamed");
  P_uint8_t (s_running, false, 0, 1, 0);
  P_int32_t (s_cursor, false, 0, 4000, 0);
  P_string  (s_script, true, "\n");
  P_end;

  static uint8_t next_id;
  uint8_t id;
  Script() {id = next_id++;}  // assign unique id

  void update( void (*command_func)(char* command), float dt, float pos_error ) {

    if(scriptRuntime.wait > 0)
      scriptRuntime.wait -= dt;

    while(s_running) {
      if(scriptRuntime.wait > 0)
        return; // where awaiting a timed delay
      if(scriptRuntime.wait_thres > 0 && pos_error > scriptRuntime.wait_thres)
        return; // where awaiting a target position by error threshold
      scriptRuntime.wait_thres = 0;  // clear target threshold

      int32_t end = s_script.find("\\n", s_cursor);
      std::string command = s_script.substr(s_cursor, end - s_cursor);

      if(command.length() > 2) {
        // Serial.printf("Script command : \"%s\"\n", command.c_str() );
        command_func((char*)command.c_str());
      }

      if(end == std::string::npos) {
        // no further commands
        s_cursor = 0;
        s_running = 0;
        return;
      }

      s_cursor = end + 2; // skip \n

      // if script_thres was just set, allow position error to update before next cycle.
      if(scriptRuntime.wait_thres>0)
        return;
    }
  }
};

uint8_t Script::next_id = 0;

