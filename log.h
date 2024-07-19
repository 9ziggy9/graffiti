#ifndef LOG_H_
#define LOG_H_
#include <stdio.h>
#include <stdlib.h>

#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

// consider using abort instead of exit--will allow for callbacks I think
#define PANIC_WITH(CODE)                                                  \
  do {                                                                    \
    fprintf(stderr, COLOR_RED"[PANIC]" COLOR_RESET " {%s} ", __FILE__);   \
    fprintf(stderr, "%s() :: [line %d] -> ", __func__, __LINE__);         \
    fprintf(stderr, "%s\n", ""#CODE"");                                   \
    exit(CODE);                                                           \
  } while(0)

#define INFO_LOG(MSG)                                                     \
  do {                                                                    \
    fprintf(stdout, COLOR_CYAN"[INFO]" COLOR_RESET" %s \n", MSG);         \
  } while(0)                                                              \

#define SUCCESS_LOG(MSG)                                                  \
  do {                                                                    \
    fprintf(stdout, COLOR_GREEN"[SUCCESS]" COLOR_RESET" %s \n", MSG);     \
  } while(0)                                                              \

typedef enum {
  WINDOW_ERR_INIT_FAIL = 100,
  WINDOW_ERR_CREATE_FAIL,
  SHADER_ERR_OPEN_FAIL,
  SHADER_ERR_ALLOC_FAIL,
  SHADER_ERR_ACTIVE_SHADER,
  SHADER_ERR_INACTIVE_SHADER,
  HWALLOC_ERR_OUT_OF_MEM,
  HWALLOC_ERR_SINGLE_INIT_VIOLATION,
  HWALLOC_ERR_EXCEEDS_MAX_MEM,
  HWALLOC_ERR_UNKNOWN_ID_TYPE,
  HWALLOC_ERR_NULL_ID,
  HWALLOC_ERR_NULL_WATCHLIST,
  PRIMITIVES_ALREADY_ENABLED,
  QUADTREE_ERR_REALLOC_FAIL,
  ARENA_INIT_STRUCT_MALLOC_FAIL,
  ARENA_INIT_MEM_MALLOC_FAIL,
  ARENA_ALLOC_SIZE_OVERFLOW,
  ARENA_INIT_MMAP_ARENA_FAIL,
  ARENA_INIT_MMAP_MEM_FAIL,
  OCC_QUAD_BAD_CONVERSION,
  BH_ILLEGAL_BODY_ACCESS,
  BH_CHILD_NODE_DOES_NOT_EXIST,
  MAIN_EXCEEDED_MAX_BODIES,
  HASH_INIT_FAIL,
} err_t;

#endif // LOG_H_
