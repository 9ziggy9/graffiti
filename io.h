#ifndef IO_H_
#define IO_H_
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define WINS_INIT(ERR_CB)                               \
  do {                                                  \
    if (!glfwInit()) PANIC_WITH(WINDOW_ERR_INIT_FAIL);  \
    glfwSetErrorCallback(ERR_CB);                       \
  } while(0)                                            \

GLFWwindow *window_create(int, int, const char *);
void window_attach_handler(GLFWwindow *, GLFWkeyfun, GLFWmousebuttonfun);

#endif // IO_H_
