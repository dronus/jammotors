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
    Serial.printf("Opening  %s for %s\n", filename, for_record ? "recording" : "playback");
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
  
  Position get(size_t index, uint8_t axis_id) {
    Frame& f = frames[index + 2]; 
    // only IK 0-2 and wrist angle axes 6 are stored.
    if (axis_id == 6 ) axis_id=3; // fourth axis is wrist axis
    return { f.dt / 1000.f, axis_id <= 3 ? (float)f.x[axis_id] : 0.f};
  }
  
  bool need_cp = false;
  void put(uint8_t axis_id, float x, bool is_control) {
    if (axis_id == 6 ) axis_id=3; // fourth axis is wrist axis
    if (axis_id > 3 ) return;  
    frames[3].x[axis_id] = x;
    need_cp = need_cp || is_control;
  }
     
  // inter- or extrapolate position at t0 using matching three known points with a quadratic polynomial. 
  // works both as an extrapolating predictor (t0 > 0) and an interpolating approximator (t < 0).
  // control points are defined by their delta positions, and are aligned at t = 0 going into the t-negative direction.
  float interpolate(Position p1, Position p2, Position p3, float t0) {

    float t1 = -p1.dt, t2 = t1 -p2.dt;
    if(t1 == 0.f || t2 == 0.f || t1 == t2) t1 = -0.1f, t2 = -0.2f; // bootstrapping on startup

    float x1 = p1.x, x2 = p2.x, x3 = p3.x;
    float s1 = x2 - x1;
    float s2 = x3 - x1;
    float v1  = s1 / t1; // integral velocity on s1
    float v2  = s2 / t2; // integral velocity on s2
    
    float a = 2 * (v1-v2) / (t1-t2);      // formula from solving  s = a*t*t + v0*t equations for s1 and s2.
    float v0= (s1-1.f/2.f*a*t1*t1) / t1;  // also from same equations

    // get interpolation
    float x0 = x1 + v0*t0 + 1.f/2.f * a*t0*t0;

    return x0;
  }
  
  // interpolate with cubic function using 4 control points
  // the error is guaranteed to be less then the first 3 control point quadric form already guarantees
  // which is already gouverned by control point placement threshold.
  float interpolate4(Position p0, Position p1, Position p2, Position p3, float t0) {
    float x0 = interpolate(p1,p2,p3,t0);    
    float dxdt = ( p0.x - interpolate(p1,p2,p3,p0.dt) ) / p0.dt;
    
    return x0 + dxdt * t0;
  }

  void update_axis(float dt, Axis& axis) {
    uint8_t id = axis.id;
    if(recording) {
      // predict      
      Position p1 = get(0,id), p2=get(-1,id), p3=get(-2,id);
      float x0 = interpolate(p1,p2,p3,dt);
      // compare
      axis.ik_pred_err = x0 - (axis.ik_feedback - axis.ik_offset);
      // on recording, insert new control point, if error is above threshold
      put(id, axis.ik_feedback - axis.ik_offset, abs(axis.ik_pred_err) > axis.ik_pred_thres);
    }
    // on playback, set manual axis input by interpolating three past and one future point
    if(playback) {
      Position p0 = get(1,id), p1 = get(0,id), p2=get(-1,id), p3=get(-2,id);    
      axis.ik_manual = interpolate4(p0,p1,p2,p3,dt);
    }
  }

  void update(float _dt, std::vector<Axis>& axes) {
    if(recording || playback) {
      update_axis(dt, axes[0]); // x
      update_axis(dt, axes[1]); // y
      update_axis(dt, axes[2]); // z
      update_axis(dt, axes[6]); // wrist
      dt += _dt;
    }

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
