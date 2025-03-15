#pragma once

float motion_fm_osc(float o, float a, float f, float fb, float dt, float &phase) {

  phase += f * dt * 2.f * (float)PI;
  phase = fmod(phase, ((float)PI * 2.f));

  return o + a * sinf( phase + fb * sinf(phase) );
}

float motion_random(float a, float d, float rd, float dt, float &random_target, float &random_dt) {
  random_dt += dt;
  if(random_dt > d + rd) {
    random_target = random(0xFFF) * a  / (float)0xFFF;
    random_dt     = random(0xFFF) * rd / (float)0xFFF;
  }
  return random_target;
}

