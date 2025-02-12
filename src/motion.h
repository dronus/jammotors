#pragma once

float motion_fm_osc(float o, float a, float f, float fb, uint32_t dt, float &phase) {

  phase += f * dt * 2.f * (float)PI / 1000.f / 1000.f;
  phase = fmod(phase, ((float)PI * 2.f));

  return o + a * sinf( phase + fb / 1000.f * sinf(phase) );
}

