#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "log.h"
#include "shader.h"
#include "primitives.h"

#define WIN_T "test"
#define WIN_W 640
#define WIN_H 480

#define SHADER_VS_BASE "./glsl/base.vs"
#define SHADER_FS_BASE "./glsl/base.fs"

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
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }

  glfwMakeContextCurrent(win);
  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);

  glfwSetKeyCallback(win, handle_key);

  const GLuint shd = compile_simple_shader(SHADER_VS_BASE, SHADER_FS_BASE);

  while (!glfwWindowShouldClose(win)) {
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shd);
    draw_tricolor(1.0f, 0xFF0000FF, 0x00FF00FF, 0x0000FFFF);

    glfwSwapBuffers(win);
    glfwPollEvents();
  }

  glDeleteProgram(shd);
  glfwDestroyWindow(win);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
