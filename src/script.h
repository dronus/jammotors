#pragma once 

struct Script  : public Params{
  P_string  (script_name, true, "Unnamed");
  P_uint8_t (script_running, false, 0, 1, 0);
  P_int32_t (script_cursor, false, 0, 4000, 0);
  P_string  (script_script, true, "\n");
  P_end;

  static uint8_t next_id;
  uint8_t id;
  Script() {id = next_id++;}  // assign unique id
    
  void update( void (*command_func)(char* command) ) {

    if(!script_running)
      return;

    int32_t end = script_script.find("\\n", script_cursor);
    std::string command = script_script.substr(script_cursor, end - script_cursor);

    if(command.length() > 2) {
      // Serial.printf("Script command : \"%s\"\n", command.c_str() );
      command_func((char*)command.c_str());
    }

    if(end == std::string::npos) {
      // no further commands
      script_cursor = 0;
      script_running = 0;
      return;
    }

    script_cursor = end + 2; // skip \n
  }
};

uint8_t Script::next_id = 0;

