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
#include "tree.h"

void window_err_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);

void update_physics(PhysicsEntity *ps, int num_ps, BHNode *bh) {
  static double t0 = 0.0f;
  static double acc = 0.0f;
  static const double dt = 1.0f / 300.0f;
  double t1 = glfwGetTime();
  double delta_t = t1 - t0;
  t0 = t1;
  acc += delta_t;
  while (acc >= dt) {

    bhtree_integrate(VERLET_POS | VERLET_VEL, bh, dt);

    bhtree_clear_forces(bh);

    physics_apply_pairwise_gravity(ps, num_ps);
    physics_apply_collision(ps, num_ps);
    /* physics_apply_boundaries(ps, num_ps, BOUNDARY_INF_BOX); */

    bhtree_apply_boundaries(bh);

    bhtree_integrate(VERLET_VEL, bh, dt);

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
      (double)get_random(60, 120),
      get_random_color_from_palette()
    );
    physics_entity_bind_geometry(&particles[n], GEOM_CIRCLE, (Geometry){
        .circ.R = 0.08 * particles[n].m
    });
  }

  BHNode *bhtree_root = bhtree_create((vec2){0.0, 0.0}, (vec2){WIN_W, WIN_H});
  for (int n = 0; n < NUM_PARTS; n++) bhtree_insert(bhtree_root, &particles[n]);

  while (!glfwWindowShouldClose(win)) {

    update_physics(particles, NUM_PARTS, bhtree_root);

    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glfwMakeContextCurrent(win);

      OPEN_SHADER(shd);
        bhtree_draw(bhtree_root);
        /* for (int n = 0; n < NUM_PARTS; n++) { */
        /*   draw_circle(particles[n].q, */
        /*               (GLfloat) particles[n].geom.circ.R, */
        /*               particles[n].color); */
        /* } */
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
