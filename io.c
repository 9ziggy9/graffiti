#include "io.h"
#include "log.h"

GLFWwindow *window_create(int w, int h, const char *title) {
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

  GLFWwindow* win = glfwCreateWindow(w, h, title, NULL, NULL);
  if (!win) { glfwTerminate(); PANIC_WITH(WINDOW_ERR_CREATE_FAIL); }

  glfwMakeContextCurrent(win);

  if (glewInit() != GLEW_OK) PANIC_WITH(WINDOW_ERR_INIT_FAIL);
  SUCCESS_LOG("successfully initialized window");

  glViewport(0, 0, w, h);

  glEnable(GL_MULTISAMPLE);
  glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

  glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  return win;
}

void window_attach_handler(GLFWwindow *win, GLFWkeyfun fn) {
  glfwSetKeyCallback(win, fn);
  SUCCESS_LOG("successfully attached window handler");
}
