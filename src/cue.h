
struct Cue  : public Params{
  P_string  (cue_name, true, "Unnamed");
  P_uint8_t (cue_running, false, 0, 1, 0);
  P_int32_t (cue_cursor, false, 0, 4000, 0);
  P_string  (cue_script, true, "\n");
  P_end;
    
  void update( void (*command_func)(char* command) ) {
    if(!cue_running)
      return;
    
    /*
    if(cue_cursor > cue_script.length()-1) {
      cue_cursor = 0;
      cue_running = 0;
      return;
    }
    */
    
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
