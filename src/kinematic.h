#pragma once

#include "param.h"
#include "channel.h"
#include "axis.h"
#include "position_recorder.h"


struct Kinematic  : public Params {
  virtual void update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) = 0;
  virtual float update_feedback(std::vector<Channel>& channels, std::vector<Axis>& axes) = 0;
};


struct CartesianKinematic : public Kinematic  {
  P_float (ik_length_c,true,    0,  1000,  90);
  P_float (ik_length_a,true,    0,  1000, 330);
  P_float (ik_length_b,true,    0,  1000, 390);
  P_end;

  void update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) {
    float x     = axes[0].ik_pos;
    float y     = axes[1].ik_pos;
    float z     = axes[2].ik_pos;

    // define shoulder - target angle
    // define xy-plane origin - shoulder - target triangle
    float rot = sqrtf(x*x+y*y); // span origin to target in xy-plane
    if(rot < 5.f) return; // if x and y are too close to zero, just keep last angles.
    float ros = ik_length_c; // shoulder offset
    if(rot < ros) return; // target to close
    float rst = sqrtf(rot*rot - ros*ros); // offset shoulder to target distance
    float alpha =  atan2f(x,y); // - acosf ((rot*rot + rst*rst - ros*ros) / (2 * rot * rst));
    //alpha = fmodf(alpha + 1.5f * (float)pi, (float)pi) - 0.5f * (float)pi;
    axes[3].ik_ik_in = channels[0].ik_a * alpha  / (float)pi;

    // get elbow angle by triangle cosine equation 
    float a = ik_length_a; // upper arm
    float b = ik_length_b; // lower arm
    float c = sqrtf(rst*rst + z*z); // span to target
    if(c>a+b) return; // point out of reach - arm to short.
    if(b>a+c) return; // point out of reach - lower arm to long.
    if(a>b+c) return; // point out of reach - upper arm to long.
    float gamma = acosf ((a*a + b*b - c*c) / (2 * a * b)) - pi;
    
    // get shoulder angle by triangle cosine equation and atan offset
    if(z==0 && rst==0) return; // point undefined.
    float beta  = acosf ((a*a + c*c - b*b) / (2 * a * c)) - atan2f(rst,z);
    axes[4].ik_ik_in = channels[1].ik_a * beta  / (float)pi;
    axes[5].ik_ik_in = channels[2].ik_a * gamma / (float)pi;
  }

  void rot(float _x_in, float _y_in, float alpha, float& x_out, float& y_out) {
    float x_in = _x_in;
    float y_in = _y_in;
    x_out =  x_in * cosf(alpha) - y_in * sinf(alpha);
    y_out =  x_in * sinf(alpha) + y_in * cosf(alpha);
  }

  float update_feedback(std::vector<Channel>& channels, std::vector<Axis>& axes) {

    for(uint8_t i=0; i<channels.size(); i++) 
      axes[i+3].ik_feedback = channels[i].position / (float)channels[i].ik_a;

    float alpha = ( channels[0].position - axes[3].ik_offset ) / (float)channels[0].ik_a * (float)pi;
    float beta  = ( channels[1].position - axes[4].ik_offset ) / (float)channels[1].ik_a * (float)pi;
    float gamma = ( channels[2].position - axes[5].ik_offset ) / (float)channels[2].ik_a * (float)pi;
    
    float x=0, y=0, z=0;
    z += ik_length_b;
    rot(y,z,gamma,y,z);
    z += ik_length_a;
    rot(y,z,beta,y,z);
    x += ik_length_c;
    rot(y,x,alpha,y,x);

    axes[0].ik_feedback = x;
    axes[1].ik_feedback = y;
    axes[2].ik_feedback = z;
    
    // compute ik_error from x,y,z only.
    float dx = axes[0].ik_pos - x;
    float dy = axes[1].ik_pos - y;
    float dz = axes[2].ik_pos - z;
    return sqrtf( dx*dx + dy*dy + dz*dz );
  }
};


// Controller: update position of all axes:
// - the virtual kinematic input axes, which provide input to the kinematic model
// - the output axes, which provide position to the output channels
struct MotionController : public Params {
  P_float (ik_vel_max,true,    0,  10000,  500);
  P_float (ik_vel_k  ,true,    0,   1000,  100);
  P_float (ik_acc_max,true,    0, 100000, 5000);
  P_float (chan_vel_max,true,    0, 1E9f,  500);
  P_float (chan_vel_k  ,true,    0,   1000,  100);
  P_float (chan_acc_max,true,    0, 1E9f, 5000);
  P_float (ik_crawl_thres,true,    0,  10000, 10000);
  P_float (ik_crawl_vel,  true,    0,  10000, 10000);

  P_bool    (cue_stop, false,false);
  P_uint32_t(cue_index, false,0,1024,0);
  P_uint32_t(cue_size, false,0,1024,0);
  P_uint8_t(running_cue,false,0,255,0);
  P_end;
  
  Kinematic* kinematic = new CartesianKinematic();
  
  // update kinematic input and output axes and write output to channels.  
  // return "error" in the kinematics underlying unit (eg. mm for cartesian inverse kinematics)
  float update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) {
    if(cue_stop) {
      cue_stop = false;
      recorder.stop();      
    }
    recorder.update(dt,axes);
    cue_index  = recorder.index;
    cue_size   = recorder.size();
    running_cue = (recorder.recording || recorder.playback) ? recorder.current_cue_id + 1 : 0;
    
    // update kinematic input axes (eg. cartesian in case of inverse kinematics)
    for(uint8_t i=0; i<=2; i++)
      axes[i].update(dt,ik_vel_max, ik_acc_max,ik_vel_k, ik_crawl_thres, ik_crawl_vel);

    // inverse kinematics : map cartesian to angular axes
    kinematic->update(dt,axes,channels);
    
    // update actual output (angular) axes
    for(uint8_t i=0; i<channels.size(); i++)
      channels[i].ik_target = axes[i+3].update(dt,chan_vel_max, chan_acc_max,chan_vel_k, ik_crawl_thres, ik_crawl_vel);
      
    // update feedback
    float error = kinematic->update_feedback(channels, axes);
    return error;
  }
};

