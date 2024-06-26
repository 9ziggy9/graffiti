#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <time.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"
#include "alloc.h"

void glfwerr_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);

void BEGIN_FRAME(void);
void END_FRAME(void);
void SET_TARGET_FPS(uint16_t);

static double FRAME_TIME0 = 0.0f;
static double TARGET_FPS = DEFAULT_FPS;
static double TARGET_FRAME_PERIOD = 1.0f / DEFAULT_FPS;

int main(void) {
  HW_INIT();
  if (!glfwInit()) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetErrorCallback(glfwerr_cb);

  GLFWwindow *win = glfwCreateWindow(WIN_W, WIN_H, WIN_T, NULL, NULL);
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }
  HW_REGISTER(ID_GL_WIN_PTR, win);

  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

  glfwMakeContextCurrent(win);
  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetKeyCallback(win, handle_key);
  glViewport(0, 0, WIN_W, WIN_H);

  glEnable(GL_MULTISAMPLE);
  glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  SET_TARGET_FPS(144);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();

  float angle = 0.0f;
  vec2 pos_cir = WIN_CENTER;
  vec2 vel_cir = {4.0f, 4.0f};

#define RAD 80.0f
  while (!glfwWindowShouldClose(win)) {
    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      OPEN_SHADER(shd);
        draw_eqtriangle(WIN_CENTER, 0.4f, angle, 0xFF0000FF,
                                                 0x00FF00FF,
                                                 0x0000FFFF);
        draw_circle(pos_cir, RAD, 0xFF66FF66);
      CLOSE_SHADER();

      glfwSwapBuffers(win);
      glfwPollEvents();

      angle += 0.05f;

      pos_cir.x += vel_cir.x;
      pos_cir.y += vel_cir.y;

      if (pos_cir.x - RAD < 0.0f || pos_cir.x + RAD > WIN_W) vel_cir.x *= -1;
      if (pos_cir.y - RAD < 0.0f || pos_cir.y + RAD > WIN_H) vel_cir.y *= -1;

    END_FRAME();
  }

  HW_TEARDOWN();
  glfwTerminate();
  SUCCESS_LOG("program can exit successfully, good bye");
  exit(EXIT_SUCCESS);
}

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
void SET_TARGET_FPS(uint16_t fps) {
  TARGET_FPS = (double) fps;
  TARGET_FRAME_PERIOD = 1.0f / TARGET_FPS;
}

void glfwerr_cb(int error, const char *desc) {
  (void) error;
  fprintf(stderr, COLOR_RED"[GLFW ERROR]"COLOR_RESET": %s\n", desc);
}

void handle_key(GLFWwindow *win, int key, int scode, int act, int mods) {
  (void) scode; (void) mods;
  if (key == GLFW_KEY_ESCAPE && act == GLFW_PRESS) {
    glfwSetWindowShouldClose(win, GLFW_TRUE);
  }
}
