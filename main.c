#include <GL/glew.h>
#include <GLFW/glfw3.h>

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

void update_physics(KinematicCircle* circs, int num_circs, int n) {
  static double t0 = 0.0f;
  static double acc = 0.0f;
  static const double dt = 1.0f / 300.0f;
  double t1 = glfwGetTime();
  double delta_t = t1 - t0;
  t0 = t1;
  acc += delta_t;
  double sub_dt = dt / n;
  while (acc >= dt) {
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < num_circs; j++) circs[j].d2p_dt2 = (vec2){0.0f, 0.0f};
      for (int j = 0; j < num_circs; j++) {
        circs[j].p.x += circs[j].dp_dt.x * sub_dt
          + 0.5*circs[j].d2p_dt2.x*sub_dt*sub_dt;
        circs[j].p.y += circs[j].dp_dt.y * sub_dt
          + 0.5*circs[j].d2p_dt2.y*sub_dt*sub_dt;
        circs[j].dp_dt.x += 0.5 * circs[j].d2p_dt2.x * sub_dt;
        circs[j].dp_dt.y += 0.5 * circs[j].d2p_dt2.y * sub_dt;
      }
      physics_apply_pairwise_gravity(circs, num_circs);
      for (int j = 0; j < num_circs; j++) {
        circs[j].dp_dt.x += 0.5 * circs[j].d2p_dt2.x * sub_dt;
        circs[j].dp_dt.y += 0.5 * circs[j].d2p_dt2.y * sub_dt;
      }
      physics_apply_collision(circs, num_circs);
      physics_apply_boundaries(circs, num_circs);
    }
    acc -= dt;
  }
}

int main(void) {
  HW_INIT();
  WINS_INIT(window_err_cb);

  GLFWwindow *win = window_create(WIN_W, WIN_H, WIN_T1);
  window_attach_handler(win, handle_key);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();
  FRAME_TARGET_FPS(300);

  #define NUM_CIRCS 160
  KinematicCircle circs[NUM_CIRCS];

  SEED_RANDOM(9001);

  for (int n = 0; n < NUM_CIRCS; n++) {
    circs[n].p.x       = (double)get_random(0, WIN_W);
    circs[n].p.y       = (double)get_random(0, WIN_H);
    circs[n].dp_dt.x   = 0.0f;
    circs[n].dp_dt.y   = 0.0f;
    circs[n].dp_dt.x   = (double)get_random(-2000, 2000);
    circs[n].dp_dt.y   = (double)get_random(-2000, 2000);
    circs[n].d2p_dt2.x = 0.0f;
    circs[n].d2p_dt2.y = 0.0f;
    circs[n].m         = (GLfloat)get_random(60, 120);
    circs[n].rad       = 0.08f * circs[n].m;
    circs[n].color     = get_random_color_from_palette();
  }

  while (!glfwWindowShouldClose(win)) {
    update_physics(circs, NUM_CIRCS, 2);

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
