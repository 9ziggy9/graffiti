#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "config.h"
#include "log.h"
#include "shader.h"
#include "primitives.h"

#define WIN_CENTER ((vec2){WIN_W * 0.5f, WIN_H * 0.5f})

void glfwerr_cb(int error, const char *desc) {
  (void) error;
  fprintf(stderr, COLOR_RED"[GLFW ERROR]"COLOR_RESET": %s\n", desc);
}

void handle_key(GLFWwindow *win, int key, int scode, int act, int mods) {
  (void) scode; (void) mods;
  if (key == GLFW_KEY_ESCAPE && act == GLFW_PRESS) {
    glfwSetWindowShouldClose(win, GLFW_TRUE);
  }
}

int main(void) {
  if (!glfwInit()) PANIC_WITH(WINDOW_ERR_INIT_FAIL);
  glfwSetErrorCallback(glfwerr_cb);

  GLFWwindow *win = glfwCreateWindow(WIN_W, WIN_H, WIN_T, NULL, NULL);
  glScissor(50, 50, 50, 50);
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }

  glfwMakeContextCurrent(win);
  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetKeyCallback(win, handle_key);

  const GLuint shd = compile_simple_shader("./glsl/base.vs", "./glsl/base.fs");

  float angle = 0.0f;
  while (!glfwWindowShouldClose(win)) {
    glClear(GL_COLOR_BUFFER_BIT);

    OPEN_SHADER(shd);
    draw_eqtriangle(WIN_CENTER, 0.4f, angle,
                    0xFF0000FF, 0x00FF00FF, 0x0000FFFF);
    CLOSE_SHADER();

    angle += 0.005f;

    glfwSwapBuffers(win);
    glfwPollEvents();
  }

  glDeleteProgram(shd);
  glfwDestroyWindow(win);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
