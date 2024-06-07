#ifndef ALLOC_H_
#define ALLOC_H_
#include "log.h"
#include <stdlib.h>

struct watcher {
  void *loc;
  void (*td)(void *); 
};

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

static inline void __hw_remove(size_t idx) {
  HeapWatch *hw = __hw_access();
  if (hw->data[idx].td != NULL) hw->data[idx].td(hw->data[idx].loc);
  for (size_t j = idx; j < hw->sz - 1; j++) hw->data[j] = hw->data[j + 1];
  hw->sz--;
}

#define HW_INIT()                                                    \
  do {                                                               \
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
  } while (0)

#define HW_REGISTER(LOC, TD)       \
  do {                             \
    HeapWatch *hw = __hw_access(); \
    __hw_resize();                 \
    hw->data[hw->sz].loc  = LOC;   \
    hw->data[hw->sz++].td = TD;    \
  } while(0)

#define HW_FREE(loc)                                         \
  do {                                                       \
    HeapWatch *hw = __hw_access();                           \
    for (size_t i = 0; i < hw->sz; i++) {                    \
      if (hw->data[i].loc == loc) { __hw_remove(i); break; } \
    }                                                        \
  } while (0)

#define HW_TEARDOWN()                                              \
  do {                                                             \
    HeapWatch *hw = __hw_access();                                 \
    for (size_t i = 0; i < hw->sz; i++) {                          \
      if (hw->data[i].td != NULL) hw->data[i].td(hw->data[i].loc); \
    }                                                              \
    free(hw->data);                                                \
    hw->data = NULL; hw->sz = 0; hw->cap = 0;                      \
  } while (0)

#endif // ALLOC_H_
