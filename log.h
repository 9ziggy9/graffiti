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

#define PANIC_WITH(CODE)                                                  \
  do {                                                                    \
    fprintf(stdout, COLOR_RED"[PANIC]" COLOR_RESET" %s() :: ", __func__); \
    fprintf(stdout, "%s\n", ""#CODE"");                                   \
    exit(CODE);                                                           \
  } while(0)                                                              \

typedef enum {
  WINDOW_ERR_INIT_FAIL = 100,
  WINDOW_ERR_CREATE_FAIL,
  SHADER_ERR_OPEN_FAIL,
  SHADER_ERR_ALLOC_FAIL,
} win_err_t;

#endif // LOG_H_
