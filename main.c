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


#define SPD 0
PhysicsEntity *gen_n_particle_system(size_t N) {
  INFO_LOG("ALLOCATING HEAP SIZE FOR PARTICLES:");
  printf("%zu Kb \n", (N * sizeof(PhysicsEntity)) / 1024);
  PhysicsEntity *particles = (PhysicsEntity *)malloc(N * sizeof(PhysicsEntity));
  for (size_t n = 0; n < N; n++) {
    particles[n] = new_physics_entity(
      (vec2){(double)get_random(0, WIN_W), (double)get_random(0, WIN_H)},
      (vec2){(double)get_random(-SPD, SPD), (double)get_random(-SPD, SPD)},
      (vec2){0.0f, 0.0f},
      (double)get_random(800, 800),
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

  glfwMakeContextCurrent(win);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();
  FRAME_TARGET_FPS(300);

  SEED_RANDOM(11);

  // 1 MB per frame
  #define FRAME_MEMORY_SIZE 1024 * 512
  MemoryArena *FRAME_ARENA = arena_init(FRAME_MEMORY_SIZE, PAGE_PHYSICALLY);

  /*NOTE: greater allocator will REQUIRE that gen_n_particle_system also
   be tied to the arena or generalized to mmap methods. */
  #define NUM_PS 4
  PhysicsEntity *particles = gen_n_particle_system(NUM_PS);
  BHNode *ptree = bhtree_build_in_arena(FRAME_ARENA, particles, NUM_PS);
  bhtree_apply_collisions(ptree, NULL, 0xFF0000FF);

  while (!glfwWindowShouldClose(win)) {
    BEGIN_FRAME();
      /* BEGIN_PHYSICS(dt, 1); */
      /*   /\* bhtree_integrate(VERLET_POS, ptree, dt); *\/ */
      /*   bhtree_apply_boundaries(ptree); */
      /*   /\* for (size_t p = 0; p < NUM_PS; p++) { *\/ */
      /*   /\*   PhysicsEntity *body = &particles[p]; *\/ */
      /*   /\*   bhtree_apply_pairwise_collisions(body, ptree); *\/ */
      /*   /\* } *\/ */
      /*   /\* bhtree_integrate(VERLET_VEL, ptree, dt); *\/ */
      /*   /\* bhtree_clear_forces(ptree); *\/ */
      /*   arena_reset(FRAME_ARENA); */
      /* END_PHYSICS(); */

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      OPEN_SHADER(shd);
        for (size_t n = 0; n < NUM_PS; n++) {
          PhysicsEntity *p = &particles[n];
          draw_circle(p->q, (GLfloat)p->geom.circ.R, p->color);
        }
      CLOSE_SHADER();

      glfwSwapBuffers(win);
      glfwPollEvents();
    END_FRAME();
  }

  arena_reset(FRAME_ARENA);
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
