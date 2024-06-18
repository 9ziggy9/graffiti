#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <time.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"
#include "alloc.h"
#include "frames.h"
#include "physics.h"
#include "io.h"

void window_err_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);

int main(void) {
  HW_INIT();
  WINS_INIT(window_err_cb);

  GLFWwindow *win = window_create(WIN_W, WIN_H, WIN_T1);
  window_attach_handler(win, handle_key);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();
  FRAME_TARGET_FPS(144);

  #define NUM_CIRCS 50
  KinematicCircle circs[NUM_CIRCS];

  SEED_RANDOM(NULL);

  for (int n = 0; n < NUM_CIRCS; n++) {
    circs[n].p.x       = (GLfloat) get_random(0, WIN_W);
    circs[n].p.y       = (GLfloat) get_random(0, WIN_H);
    circs[n].dp_dt.x   = (GLfloat) get_random(-200, 200);
    circs[n].dp_dt.y   = (GLfloat) get_random(-200, 200);
    circs[n].d2p_dt2.x = 0.0f;
    circs[n].d2p_dt2.y = 0.0f;
    circs[n].m         = 5.0f;
    circs[n].rad       = circs[n].m;
    circs[n].color     = get_random_color_from_palette();
  }

  while (!glfwWindowShouldClose(win)) {
    BEGIN_PHYSICS(dt);
      physics_apply_collision(circs, NUM_CIRCS);
      for (int n = 0; n < NUM_CIRCS; n++) {
        physics_apply_boundaries(&circs[n]);
        circs[n].dp_dt.x += (GLfloat)0.5 * circs[n].d2p_dt2.x * dt;
        circs[n].dp_dt.y += (GLfloat)0.5 * circs[n].d2p_dt2.y * dt;
        circs[n].p.x += circs[n].dp_dt.x * dt;
        circs[n].p.y += circs[n].dp_dt.y * dt;
        circs[n].dp_dt.x += (GLfloat)0.5 * circs[n].d2p_dt2.x * dt;
        circs[n].dp_dt.y += (GLfloat)0.5 * circs[n].d2p_dt2.y * dt;
      }
    END_PHYSICS();

    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glfwMakeContextCurrent(win);

      OPEN_SHADER(shd);
        for (int n = 0; n < NUM_CIRCS; n++) {
          draw_circle(circs[n].p, circs[n].rad, circs[n].color);
        }
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

void window_err_cb(int error, const char *desc) {
  (void) error;
  fprintf(stderr, COLOR_RED"[GLFW ERROR]"COLOR_RESET" %s\n", desc);
}

void handle_key(GLFWwindow *win, int key, int scode, int act, int mods) {
  (void) scode; (void) mods;
  if (key == GLFW_KEY_ESCAPE && act == GLFW_PRESS) {
    glfwSetWindowShouldClose(win, GLFW_TRUE);
  }
}
