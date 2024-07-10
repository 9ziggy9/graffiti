#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string.h>

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
void handle_mouse(GLFWwindow *, int, int, int);

#define MAX_PARTICLES 2000
size_t NUM_PS = 0;

PhysicsEntity particles[MAX_PARTICLES];

// 1 MB per frame
#define FRAME_MEMORY_SIZE 1024 * 512
MemoryArena *FRAME_ARENA;
BHNode *ptree;

#define SPD 120
void gen_n_particle_system(size_t N) {
  INFO_LOG("ALLOCATING HEAP SIZE FOR PARTICLES:");
  printf("%zu Kb \n", (N * sizeof(PhysicsEntity)) / 1024);
  for (size_t n = 0; n < N; n++) {
    particles[n] = new_physics_entity(
      (vec2){(double)get_random(0, WIN_W), (double)get_random(0, WIN_H)},
      (vec2){(double)get_random(-SPD, SPD), (double)get_random(-SPD, SPD)},
      (vec2){0.0f, 0.0f},
      (double)get_random(240, 240),
      get_random_color_from_palette()
    );
    physics_entity_bind_geometry(&particles[n], GEOM_CIRCLE, (Geometry){
        .circ.R = 0.08 * particles[n].m
    });
  }
  NUM_PS = N;
}

int main(void) {
  HW_INIT();
  WINS_INIT(window_err_cb);

  GLFWwindow *win = window_create(WIN_W, WIN_H, WIN_T1);
  window_attach_handler(win, handle_key, handle_mouse);

  glfwMakeContextCurrent(win);

  GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");
  HW_REGISTER(ID_GL_SHADER_IDX, (void *) &shd);

  ENABLE_PRIMITIVES();
  FRAME_TARGET_FPS(300);

  SEED_RANDOM(9001);

  FRAME_ARENA = arena_init(FRAME_MEMORY_SIZE, PAGE_PHYSICALLY);
  gen_n_particle_system(200);

  while (!glfwWindowShouldClose(win)) {
    BEGIN_FRAME();
      ptree = bhtree_build_in_arena(FRAME_ARENA, particles, NUM_PS);
      BEGIN_PHYSICS(dt, 8);
        bhtree_integrate(VERLET_POS, ptree, dt);
        bhtree_apply_boundaries(ptree);
        bhtree_apply_collisions(ptree);
        bhtree_integrate(VERLET_VEL, ptree, dt);
        bhtree_clear_forces(ptree);
      END_PHYSICS();

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      OPEN_SHADER(shd);
        bhtree_draw(ptree);
        /* draw_rectangle_boundary(vec2sub(particles[0].q, */
        /*                                 (vec2){2 * particles[0].geom.circ.R, */
        /*                                        2 * particles[0].geom.circ.R}), */
        /*                         vec2add(particles[0].q, */
        /*                                 (vec2){2 * particles[0].geom.circ.R, */
        /*                                        2 * particles[0].geom.circ.R}), */
        /*                         0x00FF00FF); */
        bhtree_draw_quads(ptree, 0xFFFFFFFF);
      CLOSE_SHADER();

      glfwSwapBuffers(win);
      glfwPollEvents();

      arena_reset(FRAME_ARENA);
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
  if (key == GLFW_KEY_C) {
    for (size_t n = 0; n < NUM_PS; n++) memset(particles, 0, MAX_PARTICLES);
    NUM_PS = 0;
    ptree = NULL;
  }
}

void handle_mouse(GLFWwindow *win, int button, int act, int mods) {
  (void) mods;
  if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
    double x, y;
    glfwGetCursorPos(win, &x, &y);
    printf("Generating particle @ (%f, %f)\n", x, y);
    if (NUM_PS == MAX_PARTICLES) PANIC_WITH(MAIN_EXCEEDED_MAX_BODIES);
    particles[NUM_PS] = new_physics_entity(
      (vec2){x, WIN_H - y},
      (vec2){(double)get_random(-SPD, SPD), (double)get_random(-SPD, SPD)},
      (vec2){0.0f, 0.0f},
      (double)get_random(100, 100),
      get_random_color_from_palette()
    );
    physics_entity_bind_geometry(&particles[NUM_PS], GEOM_CIRCLE, (Geometry){
        .circ.R = 0.08 * particles[NUM_PS].m
    });
    NUM_PS++;
    printf("NUMBER OF PARTICLES: %zu\n", NUM_PS);
  }
}
