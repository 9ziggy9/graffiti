#ifndef ALLOC_H_
#define ALLOC_H_
#include "log.h"
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdbool.h>
#include <sys/mman.h>

#define PAGE_VIRTUALLY  false
#define PAGE_PHYSICALLY true

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

void HW_INIT(void);
void HW_REGISTER(res_id_t, void *);
void HW_TEARDOWN(void);

typedef struct {
  size_t size;
  size_t used;
  void *mem;
} MemoryArena;

MemoryArena *arena_init(size_t, bool);
void *arena_alloc(MemoryArena *, size_t);
void arena_reset(MemoryArena *);
void arena_free(MemoryArena *);

#endif // ALLOC_H_
