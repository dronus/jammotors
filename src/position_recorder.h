#pragma once 

struct Position {
  float dt, x;
};

struct Frame {
  uint32_t dt;
  int16_t x[4];
};

struct Recorder {
  File file;
  uint8_t current_cue_id;
  bool recording = 0;
  bool playback  = 0;
  uint32_t index = 0;
  float dt = 0;  
  Frame frames[4];
  
  void open(bool for_record) {    
    char filename[32];
    sprintf(filename, "/cue_motion_%d", current_cue_id);
    file = LittleFS.open(filename, for_record ? "w" : "r", true); 
  }
  
  void record(uint8_t cue_id) {
    current_cue_id = cue_id;
    stop();
    open(true);
    recording = true;
  }

  void play(uint8_t cue_id) {
    current_cue_id = cue_id;
    stop();
    open(false);
    playback = true; 
  }
  
  void stop() {
    recording = false;
    playback = false;
    dt = 0;
    index = 0;
    if(file) file.close();    
  }
  
  Position get(size_t index, uint8_t axis) {
    Frame& f = frames[index + 2]; 
    // only IK and wrist angle axes 3-6 are stored.
    return { f.dt / 1000.f, axis >=3 ? (float)f.x[axis-3] : 0.f};
  }
  
  bool need_cp = false;
  void put(uint8_t axis, float x, bool is_control) {
    if(axis<3) return; // only IK and wrist angle axes 3-6 are stored.
    frames[3].x[axis-3] = x;
    need_cp = need_cp || is_control;
  }
  
  void update(float _dt) {
    if(recording || playback)
      dt += _dt;

    if(recording && need_cp) {
      need_cp = false;
      frames[3].dt = dt * 1000.f;
      // file.seek(index * sizeof(Frame));
      file.write((uint8_t*)&frames[3], sizeof(Frame));
      //Serial.printf("IK CP written: idx: %d dt: %.5g file pos: %d \n", index, frames[3].dt / 1000.f, file.position());

      frames[0] = frames[1];
      frames[1] = frames[2];
      frames[2] = frames[3];

      dt = 0;
      index++;      
      if(index >= 1024) stop();      
    }
        
    if(playback && dt > frames[3].dt / 1000.f) {
      dt -= frames[3].dt / 1000.f;
      frames[0] = frames[1];
      frames[1] = frames[2];
      frames[2] = frames[3];
      file.read((uint8_t*)&frames[3], sizeof(Frame));
      //Serial.printf("IK CP read: idx: %d dt: %.5g file pos: %d \n", index, dt, file.position());

      index++;
      if(index >= size()) stop();
    }
  }
  
  size_t size() {
    if(!file) return 0;
    return file.size() / sizeof(Frame);
  }
} recorder;
