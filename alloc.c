#include "alloc.h"
#include "log.h"

static inline HeapWatch *__hw_access(void) {
  static HeapWatch __hw = {NULL, 0, 0};
  return &__hw;
}

static inline void __hw_resize(void) {
  HeapWatch *hw = __hw_access();
  if (hw->sz >= hw->cap) {
    printf("REALLOCING __hw_resize()\n");
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

void HW_INIT(void) {
  INFO_LOG("initializing heap watch list ...");
  HeapWatch *hw = __hw_access();
  if (hw->data == NULL && hw->cap == 0) {
    hw->cap  = INITIAL_HW_CAP;
    hw->sz   = 0;
    hw->data = (struct watcher *) malloc(hw->cap * sizeof(struct watcher)); 
    if (hw->data == NULL) PANIC_WITH(HWALLOC_ERR_OUT_OF_MEM);
  } else PANIC_WITH(HWALLOC_ERR_SINGLE_INIT_VIOLATION);
  SUCCESS_LOG("heap watch list successfully initialized");
}

void HW_REGISTER(res_id_t type, void *id) {
  HeapWatch *hw = __hw_access();
  __hw_resize();
  hw->data[hw->sz].type = type;
  hw->data[hw->sz++].id = id;
}

void HW_TEARDOWN(void) {
  INFO_LOG("performing watch list teardown of resources ...");
  HeapWatch *hw = __hw_access();
  if (hw->data == NULL) PANIC_WITH(HWALLOC_ERR_NULL_WATCHLIST);
  INFO_LOG("accessing heap watchlist ... ");
  INFO_LOG("entering deallocation dispatcher ...");
  for (int i = (int)hw->sz - 1; i >= 0; i--) __hw_free(&hw->data[i]);
  free(hw->data);
  hw->data = NULL; hw->sz = 0; hw->cap = 0;
  SUCCESS_LOG("heap watch list teardown completed without error");
}

#define PAGE_SIZE 4096
#define TOUCH_PAGES(ARENA)                          \
  do {                                              \
    for (size_t i = 0; i < bytes; i += PAGE_SIZE) { \
      ((char *)ARENA->mem)[i] = 0;                  \
    }                                               \
  } while(0)


MemoryArena *arena_init(size_t bytes, bool page_strat) {
  MemoryArena *arena = mmap(NULL, sizeof(MemoryArena), PROT_READ | PROT_WRITE,
                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (arena == MAP_FAILED) PANIC_WITH(ARENA_INIT_MMAP_ARENA_FAIL);
  arena->size = bytes;
  arena->used = 0;
  arena->mem = mmap(NULL, bytes, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (arena->mem == NULL) PANIC_WITH(ARENA_INIT_MMAP_MEM_FAIL);
  if (page_strat) TOUCH_PAGES(arena); // force physical pages
  return arena;
}

void *arena_alloc(MemoryArena *arena, size_t size) {
    if (arena->used + size > arena->size) PANIC_WITH(ARENA_ALLOC_SIZE_OVERFLOW);
    void *ptr = (char*)arena->mem + arena->used;
    arena->used += size;
    return ptr;
}

void arena_reset(MemoryArena *arena) { arena->used = 0; }

void arena_free(MemoryArena *arena) {
    if (arena->mem != NULL) munmap(arena->mem, arena->size);
    arena->mem = NULL;
    arena->size = 0;
    arena->used = 0;
    if (arena != NULL) munmap(arena, sizeof(MemoryArena));
    arena = NULL;
}

#if 0 // deprecated
MemoryArena *arena_init(size_t bytes, bool page_strat) {
  MemoryArena *arena = malloc(sizeof(MemoryArena));
  if (!arena) PANIC_WITH(ARENA_INIT_STRUCT_MALLOC_FAIL);
  arena->size = bytes;
  arena->used = 0;
  arena->mem  = malloc(bytes);
  if (arena->mem == NULL) PANIC_WITH(ARENA_INIT_MEM_MALLOC_FAIL);
  if (page_strat) TOUCH_PAGES(arena); // force physical pages

  return arena;
}
#endif
