#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <time.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"

void idle_until_fps_met(double);
void glfwerr_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);

void BEGIN_FRAME(void);
void END_FRAME(void);
void SET_TARGET_FPS(uint16_t);
static double FRAME_TIME0 = 0.0f;
static double TARGET_FPS = DEFAULT_FPS;
static double TARGET_FRAME_PERIOD = 1.0f / DEFAULT_FPS;

int main(void) {
  if (!glfwInit()) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetErrorCallback(glfwerr_cb);

  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glEnable(GL_MULTISAMPLE);



  GLFWwindow *win = glfwCreateWindow(WIN_W, WIN_H, WIN_T, NULL, NULL);
  glScissor(50, 50, 50, 50);
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }

  glfwMakeContextCurrent(win);
  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetKeyCallback(win, handle_key);
  glViewport(0, 0, WIN_W, WIN_H);

  SET_TARGET_FPS(144);

  const GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");

  float angle = 0.0f;
  while (!glfwWindowShouldClose(win)) {
    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      OPEN_SHADER(shd);
      draw_eqtriangle(WIN_CENTER, 0.4f, angle,
                      0xFF0000FF, 0x00FF00FF, 0x0000FFFF);
      CLOSE_SHADER();

      angle += 0.05f;

      glfwSwapBuffers(win);
      glfwPollEvents();
    END_FRAME();
  }

  glDeleteProgram(shd);
  glfwDestroyWindow(win);
  glfwTerminate();
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
  TARGET_FPS = (double)fps;
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
