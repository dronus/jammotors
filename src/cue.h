
struct Cue  : public Params{
  P_int32_t (cue_cursor, false, -1, 4000, -1);
  P_string  (cue_name, true, "Unnamed");
  P_string  (cue_script, true, "\n");
  P_end;
    
  void update( void (*command_func)(char* command) ) {
    if(cue_cursor < 0) return;    
    if(cue_cursor > cue_script.length()-1) {
      cue_cursor = -1;
      return;
    }
    
    int32_t end = cue_script.find("\\n", cue_cursor);
    if(end == std::string::npos) {
      cue_cursor = -1; 
      return;
    }    
    std::string command = cue_script.substr(cue_cursor, end - cue_cursor);
    
    if(command.length() > 2) {
      Serial.printf("Execute cue command: \"%s\"\n", command.c_str() );
      command_func((char*)command.c_str());
    }
    
    cue_cursor = end + 2; // skip \n
  }  
};
