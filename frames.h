#ifndef FRAMES_H_
#define FRAMES_H_
#include <stdint.h>

#define DEFAULT_FPS 144

void BEGIN_FRAME(void);
void END_FRAME(void);
void FRAME_TARGET_FPS(uint16_t);

#define BEGIN_PHYSICS(DT) ({                              \
  GLfloat DT = 1.0f / DEFAULT_FPS;                        \
  static double _t0 = 0.0f;                               \
  static double _acc = 0.0f;                              \
  double _t1 = glfwGetTime();                             \
  double _dt = _t1 - _t0;                                 \
  _t0 = _t1;                                              \
  _acc += _dt;                                            \
  while (_acc >= DT) {                                    \
    _acc -= DT;
#define END_PHYSICS() }})

#endif // FRAMES_H_
