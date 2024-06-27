#ifndef FRAMES_H_
#define FRAMES_H_
#include <stdint.h>

#define DEFAULT_FPS 144

void BEGIN_FRAME(void);
void END_FRAME(void);

void FRAME_TARGET_FPS(uint16_t);
double GET_TARGET_FPS(void);

#endif // FRAMES_H_
