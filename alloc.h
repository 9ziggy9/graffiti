#ifndef ALLOC_H_
#define ALLOC_H_
#include "log.h"
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

typedef enum {
  ID_STD_PTR,
  ID_GL_WIN_PTR,
  ID_GL_SHADER_IDX,
  ID_GL_VAO_PTR,
  ID_GL_VBO_PTR,
} res_id_t;

struct watcher { void *id; res_id_t type; };

typedef struct {
  struct watcher *data;
  size_t cap;
  size_t sz;
} HeapWatch;

#define INITIAL_HW_CAP 16
#define MAX_HW_CAP     64

static inline HeapWatch *__hw_access(void) {
  static HeapWatch hw = {NULL, 0, 0};
  return &hw;
}

static inline void __hw_resize(void) {
  HeapWatch *hw = __hw_access();
  if (hw->sz >= hw->cap) {
    size_t new_cap = 2 * hw->cap;
    if (hw->cap > MAX_HW_CAP) PANIC_WITH(HWALLOC_ERR_EXCEEDS_MAX_MEM);
    struct watcher *new_data =
      (struct watcher *) realloc(hw->data, new_cap * sizeof(struct watcher));
    if (new_data == NULL) PANIC_WITH(HWALLOC_ERR_EXCEEDS_MAX_MEM);
    hw->data = new_data;
    hw->cap = new_cap;
  }
}

static inline void __hw_free(struct watcher *watcher) {
  if (watcher->id == NULL) PANIC_WITH(HWALLOC_ERR_NULL_ID);
  switch (watcher->type) {
  case ID_STD_PTR: {
    INFO_LOG("tearing down standard allocation ...");
    free(watcher->id);
    watcher->id = NULL;
  } break;
  case ID_GL_SHADER_IDX: {
    INFO_LOG("tearing down standard shader program ...");
    GLuint shader_id = *((GLuint *) watcher->id);
    if (shader_id == 0) PANIC_WITH(HWALLOC_ERR_NULL_ID);
    glDeleteProgram(*((GLuint *) watcher->id));
    *((GLuint *) watcher->id) = 0;
  } break;
  case ID_GL_WIN_PTR: {
    INFO_LOG("tearing down GLFW window ...");
    GLFWwindow *win_ptr = (GLFWwindow *) watcher->id;
    if  (win_ptr == NULL) PANIC_WITH(HWALLOC_ERR_NULL_ID);
    glfwDestroyWindow(win_ptr);
    win_ptr = NULL;
  } break;
  case ID_GL_VAO_PTR: {
    INFO_LOG("tearing vao buffer ...");
    GLuint *vao_ptr = (GLuint *) watcher->id;
    if (vao_ptr == NULL) PANIC_WITH(HWALLOC_ERR_NULL_ID);
    glDeleteVertexArrays(1, vao_ptr);
    vao_ptr = NULL;
  } break;
  case ID_GL_VBO_PTR: {
    INFO_LOG("tearing vbo buffer ...");
    GLuint *vbo_ptr = (GLuint *) watcher->id;
    if (vbo_ptr == NULL) PANIC_WITH(HWALLOC_ERR_NULL_ID);
    glDeleteBuffers(1, vbo_ptr);
    vbo_ptr = NULL;
  } break;
  default:
    PANIC_WITH(HWALLOC_ERR_UNKNOWN_ID_TYPE);
  }
}

#define HW_INIT()                                                    \
  do {                                                               \
    INFO_LOG("initializing heap watch list ...");                    \
    HeapWatch *hw = __hw_access();                                   \
    if (hw->data == NULL) {                                          \
      hw->cap  = INITIAL_HW_CAP;                                     \
      hw->sz   = 0;                                                  \
      hw->data =                                                     \
        (struct watcher *) malloc(hw->cap * sizeof(struct watcher)); \
      if (hw->data == NULL) PANIC_WITH(HWALLOC_ERR_OUT_OF_MEM);      \
    } else {                                                         \
      PANIC_WITH(HWALLOC_ERR_SINGLE_INIT_VIOLATION);                 \
    }                                                                \
    SUCCESS_LOG("heap watch list successfully initialized");         \
  } while (0)

#define HW_REGISTER(TYPE, ID)      \
  do {                             \
    HeapWatch *hw = __hw_access(); \
    __hw_resize();                 \
    hw->data[hw->sz].type = TYPE;  \
    hw->data[hw->sz++].id   = ID;  \
  } while(0)

#define HW_TEARDOWN()                                                \
  do {                                                               \
    INFO_LOG("performing watch list teardown of resources ...");     \
    HeapWatch *hw = __hw_access();                                   \
    if (hw->data == NULL) PANIC_WITH(HWALLOC_ERR_NULL_WATCHLIST);    \
    INFO_LOG("accessing heap watchlist ... ");                       \
    INFO_LOG("entering deallocation dispatcher ...");                \
    for (size_t i = 0; i < hw->sz; i++) __hw_free(&hw->data[i]);     \
    free(hw->data);                                                  \
    hw->data = NULL; hw->sz = 0; hw->cap = 0;                        \
    SUCCESS_LOG("heap watch list teardown completed without error"); \
  } while (0)

#endif // ALLOC_H_
