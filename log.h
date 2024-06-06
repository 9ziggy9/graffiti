#ifndef LOG_H_
#define LOG_H_
#include <GLFW/glfw3.h>
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
    fprintf(stderr, COLOR_RED"[PANIC]" COLOR_RESET" {%s} ", __FILE__);    \
    fprintf(stderr, "%s() :: [line %d] -> ", __func__, __LINE__);         \
    fprintf(stderr, "%s\n", ""#CODE"");                                   \
    exit(CODE);                                                           \
  } while(0)                                                              \

typedef enum {
  WINDOW_ERR_INIT_FAIL = 100,
  WINDOW_ERR_CREATE_FAIL,
  SHADER_ERR_OPEN_FAIL,
  SHADER_ERR_ALLOC_FAIL,
  SHADER_ERR_ACTIVE_SHADER,
  SHADER_ERR_INACTIVE_SHADER,
} err_t;

#endif // LOG_H_
