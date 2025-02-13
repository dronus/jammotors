#pragma once

#include "param.h"
#include "channel.h"
#include "axis.h"

struct Kinematic : public Params {
  P_int32_t (ik_length_c,true,    0,  1000,  90);
  P_int32_t (ik_length_a,true,    0,  1000, 330);
  P_int32_t (ik_length_b,true,    0,  1000, 390);
  P_int32_t (ik_vel_max,true,    0,  10000,  500);
  P_int32_t (ik_vel_k  ,true,    0, 100000, 5000);
  P_int32_t (ik_acc_max,true,    0, 100000, 5000);
  P_end;

  void update(uint32_t dt, Axis* axes, Channel* channels) {

    float x     = axes[0].update(dt,ik_vel_max, ik_acc_max,ik_vel_k);
    float y     = axes[1].update(dt,ik_vel_max, ik_acc_max,ik_vel_k);
    float z     = axes[2].update(dt,ik_vel_max, ik_acc_max,ik_vel_k);
    float delta = axes[3].update(dt,ik_vel_max, ik_acc_max,ik_vel_k);
    channels[3].ik_target = delta / (float)pi;

    // define shoulder - target angle
    if(y == 0 && x == 0) return; // if y and z are zero, just keep last angles.
   
    // define xy-plane origin - shoulder - target triangle
    float rot = sqrtf(x*x+y*y); // span origin to target in xy-plane
    float ros = ik_length_c; // shoulder offset
    if(rot < ros) return; // target to close
    float rst = sqrtf(rot*rot - ros*ros); // offset shoulder to target distance
    float alpha = atan2f(x,y) - acosf ((rot*rot + rst*rst - ros*ros) / (2 * rot * rst));
    channels[0].ik_target = alpha  / (float)pi;

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
    channels[1].ik_target = beta  / (float)pi;
    channels[2].ik_target = gamma / (float)pi;
  }

  void rot(float _x_in, float _y_in, float alpha, float& x_out, float& y_out) {
    float x_in = _x_in;
    float y_in = _y_in;
    x_out =  x_in * cosf(alpha) - y_in * sinf(alpha);
    y_out =  x_in * sinf(alpha) + y_in * cosf(alpha);
  }

  float update_feedback(Channel* channels, Axis* axes) {
    float alpha = channels[0].position / (float)channels[0].ik_a * (float)pi;
    float beta  = channels[1].position / (float)channels[1].ik_a * (float)pi;
    float gamma = channels[2].position / (float)channels[2].ik_a * (float)pi;
    float delta = channels[3].position / (float)channels[3].ik_a * (float)pi;

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
    axes[3].ik_feedback = delta;

    float dx = axes[0].ik_pos - x;
    float dy = axes[1].ik_pos - y;
    float dz = axes[2].ik_pos - z;

    return sqrtf( dx*dx + dy*dy + dz*dz );
  }
};

