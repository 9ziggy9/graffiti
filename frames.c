#include <time.h>
#include <GLFW/glfw3.h>
#include "frames.h"

static double FRAME_TIME0         = 0.0f;
static double TARGET_FPS          = DEFAULT_FPS;
static double TARGET_FRAME_PERIOD = (1.0f / DEFAULT_FPS);

void BEGIN_FRAME(void) { FRAME_TIME0 = glfwGetTime(); }

void END_FRAME(void) {
  double dt = glfwGetTime() - FRAME_TIME0;
  if (dt < TARGET_FRAME_PERIOD) {
    double sleep_until = TARGET_FRAME_PERIOD - dt;
    nanosleep(&(struct timespec) {
        .tv_nsec = (int64_t)((TARGET_FRAME_PERIOD - dt) * 1e9),
        .tv_sec  = (time_t) sleep_until,
      }, NULL);
  }
}

void FRAME_TARGET_FPS(uint16_t fps) {
  TARGET_FPS = (double) fps;
  TARGET_FRAME_PERIOD = 1.0f / TARGET_FPS;
}

double GET_TARGET_FPS(void) { return TARGET_FPS; }
