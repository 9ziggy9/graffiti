#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <time.h>
#include <math.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"
#include "alloc.h"
#include "frames.h"
#include "colors.h"

void glfwerr_cb(int, const char *);
void handle_key(GLFWwindow *, int, int, int, int);
GLint get_random(int, int);
GLuint get_random_color(void);
GLuint get_random_color_from_palette(void);

typedef struct {
  vec2 p; vec2 dp_dt; vec2 d2p_dt2;
  GLfloat m; GLfloat rad; GLuint color;
} kinematic_circle;

void physics_apply_boundaries(kinematic_circle *);

void physics_apply_naive_collisions(kinematic_circle *circ,
                                    kinematic_circle *circs,
                                    int num_circs)
{
  for (int i = 0; i < num_circs; i++) {
    if (circ == &circs[i]) continue;

    vec2 diff = (vec2) { circs[i].p.x - circ->p.x, circs[i].p.y - circ->p.y };

    if (vec2mag(diff) < circ->rad + circs[i].rad) {
      vec2 n = vec2norm(diff);
      vec2 t = (vec2) { -n.y, n.x };

      vec2 v1 = (vec2) {vec2dot(circ->dp_dt, n), vec2dot(circ->dp_dt, t)};
      vec2 v2 = (vec2) {vec2dot(circs[i].dp_dt, n), vec2dot(circs[i].dp_dt, t)};

      GLfloat v1_n = (v1.x * (circ->m - circs[i].m) + 2 * circs[i].m * v2.x)
                      / (circ->m + circs[i].m);
      GLfloat v2_n = (v2.x * (circs[i].m - circ->m) + 2 * circ->m * v1.x)
                      / (circ->m + circs[i].m);

      circ->dp_dt    = vec2add(vec2scale(v1_n, n), vec2scale(v1.y, t));
      circs[i].dp_dt = vec2add(vec2scale(v2_n, n), vec2scale(v2.y, t));
    }
  }
}

void physics_apply_collision_simple_impulse(kinematic_circle *circs,
                                            int num_circs)
{
  vec2 impulses[num_circs];

  for (int i = 0; i < num_circs; i++) impulses[i] = (vec2) { 0.0f, 0.0f };

  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].p.x - circs[i].p.x,
                           circs[j].p.y - circs[i].p.y };

      GLfloat dist = vec2mag(diff);
      GLfloat overlap = circs[i].rad + circs[j].rad - dist;

      if (overlap > 0.0f) {
        vec2 n = vec2norm(diff);
        vec2 t = (vec2) { -n.y, n.x };

        vec2 v1 = (vec2) {vec2dot(circs[i].dp_dt, n),
                          vec2dot(circs[i].dp_dt, t)};
        vec2 v2 = (vec2) {vec2dot(circs[j].dp_dt, n),
                          vec2dot(circs[j].dp_dt, t)};

        GLfloat v1_n = (v1.x * (circs[i].m - circs[j].m)
                        + 2 * circs[j].m * v2.x) / (circs[i].m + circs[j].m);
        GLfloat v2_n = (v2.x * (circs[j].m - circs[i].m)
                        + 2 * circs[i].m * v1.x) / (circs[i].m + circs[j].m);

        vec2 impulse_i = vec2add(vec2scale(v1_n, n), vec2scale(v1.y, t));
        vec2 impulse_j = vec2add(vec2scale(v2_n, n), vec2scale(v2.y, t));

        impulse_i = vec2sub(impulse_i, circs[i].dp_dt);
        impulse_j = vec2sub(impulse_j, circs[j].dp_dt);

        impulses[i] = vec2add(impulses[i], impulse_i);
        impulses[j] = vec2add(impulses[j], impulse_j);

        GLfloat corr = overlap / (circs[i].m + circs[j].m);
        circs[i].p = vec2add(circs[i].p, vec2scale(-corr * circs[i].m, n));
        circs[j].p = vec2add(circs[j].p, vec2scale(corr * circs[j].m, n));
      }
    }
  }
  for (int i = 0; i < num_circs; i++) circs[i].dp_dt = vec2add(circs[i].dp_dt,
                                                               impulses[i]);
}

void physics_apply_collision(kinematic_circle *circs, int num_circs) {
  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].p.x - circs[i].p.x,
                           circs[j].p.y - circs[i].p.y };

      GLfloat dist = vec2mag(diff);
      GLfloat overlap = circs[i].rad + circs[j].rad - dist;

      if (overlap > 0.0f) {
        vec2 n = vec2norm(diff);
        vec2 t = (vec2) { -n.y, n.x };

        vec2 v1 = (vec2) {vec2dot(circs[i].dp_dt, n),
                          vec2dot(circs[i].dp_dt, t)};
        vec2 v2 = (vec2) {vec2dot(circs[j].dp_dt, n),
                          vec2dot(circs[j].dp_dt, t)};

        GLfloat v1_n = (v1.x * (circs[i].m - circs[j].m)
                        + 2 * circs[j].m * v2.x) / (circs[i].m + circs[j].m);
        GLfloat v2_n = (v2.x * (circs[j].m - circs[i].m)
                        + 2 * circs[i].m * v1.x) / (circs[i].m + circs[j].m);

        vec2 impulse_i = vec2add(vec2scale(v1_n, n), vec2scale(v1.y, t));
        vec2 impulse_j = vec2add(vec2scale(v2_n, n), vec2scale(v2.y, t));

        impulse_i = vec2sub(impulse_i, circs[i].dp_dt);
        impulse_j = vec2sub(impulse_j, circs[j].dp_dt);

        circs[i].dp_dt = vec2add(circs[i].dp_dt, impulse_i);
        circs[j].dp_dt = vec2add(circs[j].dp_dt, impulse_j);

        GLfloat corr = overlap / (circs[i].m + circs[j].m);
        circs[i].p = vec2add(circs[i].p, vec2scale(-corr * circs[i].m, n));
        circs[j].p = vec2add(circs[j].p, vec2scale(corr * circs[j].m, n));
      }
    }
  }
}

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
  FRAME_TARGET_FPS(144);

  #define NUM_CIRCS 500

  kinematic_circle circs[NUM_CIRCS];
  srand((GLuint)time(NULL));

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

void physics_apply_boundaries(kinematic_circle *circ) {
  if (circ->p.x - circ->rad <= 0.0) {
    circ->dp_dt.x *= -1;
    circ->p.x = circ->rad;
  } else if (circ->p.x + circ->rad >= WIN_W) {
    circ->dp_dt.x *= -1;
    circ->p.x = WIN_W - circ->rad;
  }
  if (circ->p.y - circ->rad <= 0.0) {
    circ->dp_dt.y *= -1;
    circ->p.y = circ->rad;
  } else if (circ->p.y + circ->rad >= WIN_H) {
    circ->dp_dt.y *= -1;
    circ->p.y = WIN_H - circ->rad;
  }
}

GLint get_random(int low, int high) {return (rand() % (high - low + 1)) + low;}

GLuint get_random_color(void) {
  return ((GLuint)get_random(0, 255) << 24)
       | ((GLuint)get_random(0, 255) << 16)
       | ((GLuint)get_random(0, 255) << 8)
       | 0xFF; 
}

GLuint get_random_color_from_palette(void) {
  static const GLuint color_palette[] = {
    CLR_RED    , CLR_GREEN  , CLR_BLUE   , CLR_YELLOW , CLR_MAGENTA, CLR_CYAN  ,
    CLR_ORANGE , CLR_PURPLE , CLR_FGREEN , CLR_NBLUE  , CLR_GRAY   , CLR_BROWN ,
    CLR_LGREEN , CLR_SBLUE  , CLR_PINK   , CLR_AQUA   ,
  };
  return color_palette[get_random(0, sizeof(color_palette) /
                                     sizeof(color_palette[0]) - 1)];
}
