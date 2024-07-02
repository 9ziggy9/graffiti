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
#include "colors.h"

void window_err_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);


PhysicsEntity *gen_n_particle_system(size_t N) {
  PhysicsEntity *particles = (PhysicsEntity *)malloc(N * sizeof(PhysicsEntity));
  for (size_t n = 0; n < N; n++) {
    particles[n] = new_physics_entity(
      (vec2){(double)get_random(0, WIN_W), (double)get_random(0, WIN_H)},
      (vec2){(double)get_random(0, 0), (double)get_random(0, 0)},
      (vec2){0.0f, 0.0f},
      (double)get_random(100, 150),
      get_random_color_from_palette()
    );
    physics_entity_bind_geometry(&particles[n], GEOM_CIRCLE, (Geometry){
        .circ.R = 0.08 * particles[n].m
    });
  }
  return particles;
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

  SEED_RANDOM(9001);

  // 1 MB per frame
  #define FRAME_MEMORY_SIZE 1024 * 1024
  MemoryArena *FRAME_ARENA = arena_init(FRAME_MEMORY_SIZE);

  #define NUM_PS 4
  PhysicsEntity *particles = gen_n_particle_system(NUM_PS);
  BHNode *ptree = bhtree_build_in_arena(FRAME_ARENA, particles, NUM_PS);

  while (!glfwWindowShouldClose(win)) {
    BEGIN_PHYSICS(dt);
      bhtree_integrate(VERLET_POS | VERLET_VEL, ptree, dt);
      bhtree_apply_boundaries(ptree);
      bhtree_integrate(VERLET_VEL, ptree, dt);
    END_PHYSICS();
    BEGIN_FRAME();
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glfwMakeContextCurrent(win);
      OPEN_SHADER(shd);
        bhtree_draw(ptree);
        bhtree_draw_quads(ptree, CLR_GREEN);
      CLOSE_SHADER();
      glfwSwapBuffers(win);
      glfwPollEvents();
    END_FRAME();
  }

  arena_free(FRAME_ARENA);
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
