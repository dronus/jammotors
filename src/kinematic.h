#pragma once

#include "param.h"
#include "channel.h"
#include "axis.h"
#include "position_recorder.h"


struct Kinematic  : public Params {
  virtual void update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) = 0;
  virtual float update_feedback(std::vector<Channel>& channels, std::vector<Axis>& axes) = 0;
};

Kinematic* createKinematic(uint8_t kinematic_id);

struct KinematicArmCartesian : public Kinematic  {
  P_float (ik_length_c,true,    0,  1000,  90);
  P_float (ik_length_a,true,    0,  1000, 330);
  P_float (ik_length_b,true,    0,  1000, 390);
  P_float (ik_mass_a,true, -100000,  100000,  0);
  P_float (ik_mass_b,true, -100000,  100000,  0);
  P_end;
 
  void update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) {
  
    float x=axes[0].ik_pos, y=axes[1].ik_pos, z=axes[2].ik_pos;
    
    // define shoulder - target angle
    // define xy-plane origin - shoulder - target triangle
    float rot = sqrtf(z*z+y*y); // span origin to target in zy-plane
    if(rot < 5.f) return; // if z and y are too close to zero, just keep last angles.
    float ros = ik_length_c; // shoulder offset
    if(rot < ros) return; // target to close
    float rst = sqrtf(rot*rot - ros*ros); // offset shoulder to target distance
    float alpha =  atan2f(z,y); // - acosf ((rot*rot + rst*rst - ros*ros) / (2 * rot * rst));
    axes[3].ik_ik_in = 180 * alpha  / (float)pi;
    
    // get elbow angle by triangle cosine equation 
    float a = ik_length_a; // upper arm
    float b = ik_length_b; // lower arm
    float c = sqrtf(rst*rst + x*x); // span to target
    if(c>a+b) return; // point out of reach - arm to short.
    if(b>a+c) return; // point out of reach - lower arm to long.
    if(a>b+c) return; // point out of reach - upper arm to long.
    float gamma = acosf ((a*a + b*b - c*c) / (2 * a * b)) - pi;
     
    // get shoulder angle by triangle cosine equation and atan offset
    if(z==0 && rst==0) return; // point undefined.
    float beta  = acosf ((a*a + c*c - b*b) / (2 * a * c)) - atan2f(rst,x);
    
    axes[4].ik_ik_in = 180 * beta  / (float)pi;
    axes[5].ik_ik_in = 180 * gamma / (float)pi;
      
    // update static torque (gravity correction)
    channels[0].torque_in = -ik_mass_a * cosf(alpha) * sinf(beta) / ik_length_a * copysign(1000.f, channels[0].ik_a); // in Nm
    channels[1].torque_in = -ik_mass_a * sinf(alpha) * cosf(beta) / ik_length_a * copysign(1000.f, channels[1].ik_a); // in Nm
    channels[2].torque_in = 0; // TODO
  }

  void rot(float _x_in, float _y_in, float alpha, float& x_out, float& y_out) {
    float x_in = _x_in;
    float y_in = _y_in;
    x_out =  x_in * cosf(alpha) - y_in * sinf(alpha);
    y_out =  x_in * sinf(alpha) + y_in * cosf(alpha);
  }

  float update_feedback(std::vector<Channel>& channels, std::vector<Axis>& axes) {

    for(uint8_t i=0; i<channels.size(); i++) 
      axes[i+3].ik_feedback = channels[i].position / channels[i].ik_a;

    float alpha = ( channels[0].position / (float)channels[0].ik_a - axes[3].ik_offset ) / 180 * (float)pi;
    float beta  = ( channels[1].position / (float)channels[1].ik_a - axes[4].ik_offset ) / 180 * (float)pi;
    float gamma = ( channels[2].position / (float)channels[2].ik_a - axes[5].ik_offset ) / 180 * (float)pi;
    
    float x=0, y=0, z=0;
    x += ik_length_b;
    rot(y,x,gamma,y,x);
    x += ik_length_a;
    rot(y,x,beta,y,x);
    z += ik_length_c;
    rot(y,z,alpha,y,z);

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

struct KinematicHand : public Kinematic  {
  P_float (ik_finger_l,true,  0, 1000,   90);
  P_float (ik_hand_w,  true,  0, 1000,  100);  
  P_end;

  void update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels) {
    float grab   = axes[0].ik_pos;
    float tilt   = axes[1].ik_pos;
    float finger_dx = ik_hand_w / 4;
    
    for(uint8_t finger_id=0; finger_id<5; finger_id++) {
      float alpha = grab / ik_finger_l + (finger_id-2.f)/5.f * finger_dx / ik_finger_l * tilt / ik_hand_w;
      axes[finger_id+3].ik_ik_in = channels[finger_id].ik_a * alpha;
    }
  }

  // feedback is useless, as the hand uses non-feedback motors. just pass targets.
  float update_feedback(std::vector<Channel>& channels, std::vector<Axis>& axes) {
    for(uint8_t i=0; i<3; i++)
      axes[i].ik_feedback = axes[i].ik_pos;    
    return 0.f;
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

  P_uint8_t(kinematic_id,true,0,2,1);
  P_end;
  
  uint8_t last_kinematic_id=0;  
  Kinematic* kinematic = NULL;
    
  // update kinematic input and output axes and write output to channels.  
  // return "error" in the kinematics underlying unit (eg. mm for cartesian inverse kinematics)
  float update(float dt, std::vector<Axis>& axes, std::vector<Channel>& channels,  void(*updatePrefs)()) {
      
    recorder.update(dt,axes);
    
    // update kinematic input axes (eg. cartesian in case of inverse kinematics)
    for(uint8_t i=0; i<=2; i++)
      axes[i].update(dt,ik_vel_max, ik_acc_max,ik_vel_k, ik_crawl_thres, ik_crawl_vel);

    // kinematics : map virtual kinematic axes to channel-tied output axes
    if(kinematic_id != last_kinematic_id) {
      if(kinematic) delete kinematic;
      kinematic = createKinematic(kinematic_id);
      last_kinematic_id = kinematic_id;
      (*updatePrefs)();
    }
    if(kinematic) kinematic->update(dt,axes,channels);
    
    // update actual output (angular) axes
    for(uint8_t i=0; i<channels.size(); i++)
      channels[i].ik_target = axes[i+3].update(dt,chan_vel_max, chan_acc_max,chan_vel_k, ik_crawl_thres, ik_crawl_vel);
      
    // update feedback
    float error = kinematic ? kinematic->update_feedback(channels, axes) : 0.;
    return error;
  }
};

