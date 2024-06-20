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

void update_physics(PhysicsEntity *ps, int num_ps, int n) {
  static double t0 = 0.0f;
  static double acc = 0.0f;
  static const double dt = 1.0f / 300.0f;
  double t1 = glfwGetTime();
  double delta_t = t1 - t0;
  t0 = t1;
  acc += delta_t;
  double _dt = dt / n;
  while (acc >= dt) {
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < num_ps; j++) ps[j].d2q_dt2 = (vec2){0.0f, 0.0f};
      for (int j = 0; j < num_ps; j++) {
        ps[j].q.x += ps[j].dq_dt.x * _dt + 0.5 * ps[j].d2q_dt2.x * _dt * _dt;
        ps[j].q.y += ps[j].dq_dt.y * _dt + 0.5 * ps[j].d2q_dt2.y * _dt * _dt;
        ps[j].dq_dt.x += 0.5 * ps[j].d2q_dt2.x * _dt;
        ps[j].dq_dt.y += 0.5 * ps[j].d2q_dt2.y * _dt;
      }
      physics_apply_pairwise_gravity(ps, num_ps);
      for (int j = 0; j < num_ps; j++) {
        ps[j].dq_dt.x += 0.5 * ps[j].d2q_dt2.x * _dt;
        ps[j].dq_dt.y += 0.5 * ps[j].d2q_dt2.y * _dt;
      }
      physics_apply_collision(ps, num_ps);
      physics_apply_boundaries(ps, num_ps);
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

  #define NUM_PARTS 160
  PhysicsEntity particles[NUM_PARTS];

  SEED_RANDOM(9001);

  for (int n = 0; n < NUM_PARTS; n++) {
    particles[n] = new_physics_entity(
        (vec2){(double)get_random(0, WIN_W)   , (double)get_random(0, WIN_H)},
        (vec2){(double)get_random(-2000, 2000), (double)get_random(-2000, 2000)},
        (vec2){0.0f, 0.0f},
        (GLfloat)get_random(60, 120),
        get_random_color_from_palette());
    physics_entity_bind_geometry(&particles[n], GEOM_CIRCLE, (Geometry){
        .circ.R = 0.08f * particles[n].m
    });
  }

  while (!glfwWindowShouldClose(win)) {
    update_physics(particles, NUM_PARTS, 2);

    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glfwMakeContextCurrent(win);

      OPEN_SHADER(shd);
        for (int n = 0; n < NUM_PARTS; n++) {
          draw_circle(particles[n].q,
                      particles[n].geom.circ.R,
                      particles[n].color);
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
