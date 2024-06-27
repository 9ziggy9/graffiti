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

int main(void) {
  HW_INIT();
  WINS_INIT(window_err_cb);

  GLFWwindow *win = window_create(WIN_W, WIN_H, WIN_T1);
  window_attach_handler(win, handle_key);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();
  FRAME_TARGET_FPS(300);

  SEED_RANDOM(9001);

  #define NUM_PARTS 160
  PhysicsEntity particles[NUM_PARTS];

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

  PhysicsSystem sys = new_physics_system(NUM_PARTS, particles,
                                         force_pairwise_gravity,
                                         force_pairwise_impulsive_collision);

  BHNode *bhtree_root = bhtree_create((vec2){0.0, 0.0}, (vec2){WIN_W, WIN_H});
  for (int n = 0; n < NUM_PARTS; n++) bhtree_insert(bhtree_root, &particles[n]);

  while (!glfwWindowShouldClose(win)) {
    BEGIN_PHYSICS(dt);
      bhtree_integrate(VERLET_POS | VERLET_VEL, bhtree_root, dt);
      bhtree_clear_forces(bhtree_root);
      forces_apply_internal(&sys);
      bhtree_apply_boundaries(bhtree_root);
      bhtree_integrate(VERLET_VEL, bhtree_root, dt);
    END_PHYSICS();

    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glfwMakeContextCurrent(win);

      OPEN_SHADER(shd);
        bhtree_draw(bhtree_root);
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
