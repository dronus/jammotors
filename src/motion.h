#pragma once

float motion_fm_osc(float o, float a, float f, float fb, float dt, float &phase) {

  phase += f * dt * 2.f * (float)PI;
  phase = fmod(phase, ((float)PI * 2.f));

  return o + a * sinf( phase + fb * sinf(phase) );
}

