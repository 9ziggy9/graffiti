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
    if (circ == &circs[i]) continue; // skip self-collision

    GLfloat dx = circs[i].p.x - circ->p.x;
    GLfloat dy = circs[i].p.y - circ->p.y;
    GLfloat dist = sqrtf(dx * dx + dy * dy);
    GLfloat sum_rad = circ->rad + circs[i].rad;

    if (dist < sum_rad) {
      // calculate normal vector
      GLfloat nx = dx / dist;
      GLfloat ny = dy / dist;

      // calculate tangent vector
      GLfloat tx = -ny;
      GLfloat ty = nx;

      // project velocities onto tangent and normal vectors
      GLfloat v1n = circ->dp_dt.x * nx + circ->dp_dt.y * ny;
      GLfloat v1t = circ->dp_dt.x * tx + circ->dp_dt.y * ty;
      GLfloat v2n = circs[i].dp_dt.x * nx + circs[i].dp_dt.y * ny;
      GLfloat v2t = circs[i].dp_dt.x * tx + circs[i].dp_dt.y * ty;

      // apply elastic collision
      GLfloat v1n_new = (v1n * (circ->m - circs[i].m) + 2 * circs[i].m * v2n) /
                        (circ->m + circs[i].m);
      GLfloat v2n_new = (v2n * (circs[i].m - circ->m) + 2 * circ->m * v1n) /
                        (circ->m + circs[i].m);

      // convert back to Cartesian coordinates
      circ->dp_dt.x = v1n_new * nx + v1t * tx;
      circ->dp_dt.y = v1n_new * ny + v1t * ty;
      circs[i].dp_dt.x = v2n_new * nx + v2t * tx;
      circs[i].dp_dt.y = v2n_new * ny + v2t * ty;
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

  #define NUM_CIRCS 10
  #define RAD 10.0f

  kinematic_circle circs[NUM_CIRCS];
  srand((GLuint)time(NULL));

  for (int n = 0; n < NUM_CIRCS; n++) {
    circs[n].p.x       = (GLfloat) get_random(0, WIN_W);
    circs[n].p.y       = (GLfloat) get_random(0, WIN_H);
    circs[n].dp_dt.x   = (GLfloat) get_random(-500, 500);
    circs[n].dp_dt.y   = (GLfloat) get_random(-500, 500);
    circs[n].d2p_dt2.x = 0.0f;
    circs[n].d2p_dt2.y = 0.0f;
    circs[n].rad       = RAD;
    circs[n].m         = (GLfloat) get_random(20, 50);
    circs[n].color     = get_random_color_from_palette();
  }

  while (!glfwWindowShouldClose(win)) {

    BEGIN_PHYSICS(dt);
      for (int n = 0; n < NUM_CIRCS; n++) {
        circs[n].dp_dt.x += (GLfloat)0.5 * circs[n].d2p_dt2.x * dt;
        circs[n].dp_dt.y += (GLfloat)0.5 * circs[n].d2p_dt2.y * dt;
        circs[n].p.x += circs[n].dp_dt.x * dt;
        circs[n].p.y += circs[n].dp_dt.y * dt;
        physics_apply_boundaries(&circs[n]);
        physics_apply_naive_collisions(&circs[n], circs, NUM_CIRCS);
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
