#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <time.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"
#include "alloc.h"
#include "frames.h"

void glfwerr_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);

int main(void) {
  HW_INIT();
  if (!glfwInit()) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetErrorCallback(glfwerr_cb);


  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

  GLFWwindow *win = glfwCreateWindow(WIN_W, WIN_H, WIN_T, NULL, NULL);
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }
  glfwMakeContextCurrent(win);
  HW_REGISTER(ID_GL_WIN_PTR, win);

  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetKeyCallback(win, handle_key);
  glViewport(0, 0, WIN_W, WIN_H);

  glEnable(GL_MULTISAMPLE);
  glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();


  GLfloat angle = 0.0f;
  GLfloat ang_vel = 5.0f;

  vec2 pos_cir = WIN_CENTER;
  vec2 vel_cir = {100.0f, 100.0f};

  FRAME_TARGET_FPS(144);

#define RAD 80.0f
  while (!glfwWindowShouldClose(win)) {
    BEGIN_FRAME();

      BEGIN_PHYSICS(dt);
        angle += ang_vel * dt;
        pos_cir.x += vel_cir.x * dt;
        pos_cir.y += vel_cir.y * dt;
        if (pos_cir.x - RAD <= 0.0f || pos_cir.x + RAD >= WIN_W) vel_cir.x *=-1;
        if (pos_cir.y - RAD <= 0.0f || pos_cir.y + RAD >= WIN_H) vel_cir.y *=-1;
      END_PHYSICS();

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      OPEN_SHADER(shd);
        draw_eqtriangle(WIN_CENTER, 0.4f, angle, 0xFF0000FF,
                                                 0x00FF00FF,
                                                 0x0000FFFF);
        draw_circle(pos_cir, RAD, 0xFF66FF66);
      CLOSE_SHADER();

      glfwSwapBuffers(win);
      glfwPollEvents();


    END_FRAME();
  }

  HW_TEARDOWN();
  glfwTerminate();
  SUCCESS_LOG("program can exit successfully, good bye");
  exit(EXIT_SUCCESS);
}

void glfwerr_cb(int error, const char *desc) {
  (void) error;
  fprintf(stderr, COLOR_RED"[GLFW ERROR]"COLOR_RESET" %s\n", desc);
}

void handle_key(GLFWwindow *win, int key, int scode, int act, int mods) {
  (void) scode; (void) mods;
  if (key == GLFW_KEY_ESCAPE && act == GLFW_PRESS) {
    glfwSetWindowShouldClose(win, GLFW_TRUE);
  }
}
