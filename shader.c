#include <stdbool.h>
#include "shader.h"
#include "log.h"

static GLuint __ACTIVE_SHADER = 0;

void OPEN_SHADER(GLuint shader_id) {
  if (__ACTIVE_SHADER != 0) PANIC_WITH(SHADER_ERR_ACTIVE_SHADER);
  glUseProgram(shader_id);
  __ACTIVE_SHADER = shader_id;
}

void CLOSE_SHADER(void) {
  if (__ACTIVE_SHADER == 0) PANIC_WITH(SHADER_ERR_INACTIVE_SHADER);
  __ACTIVE_SHADER = 0;
}

GLuint SHADER(void) { return __ACTIVE_SHADER; }

static const char *load_shader_src_from_path(const char *filename) {
  FILE* file = fopen(filename, "r");
  if (!file) PANIC_WITH(SHADER_ERR_OPEN_FAIL);

  fseek(file, 0, SEEK_END);
  size_t length = (size_t) ftell(file);
  fseek(file, 0, SEEK_SET);

  char *shader = malloc(length + 1);
  if (!shader) PANIC_WITH(SHADER_ERR_ALLOC_FAIL);

  fread(shader, 1, length, file);
  shader[length] = '\0';

  fclose(file);

  return shader;
}

GLuint compile_simple_shader(const char *vert_path, const char *frag_path) {
  const char *vert_src = load_shader_src_from_path(vert_path);
  const char *frag_src = load_shader_src_from_path(frag_path);

  GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vert_shader, 1, &vert_src, NULL);
  glCompileShader(vert_shader);

  GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(frag_shader, 1, &frag_src, NULL);
  glCompileShader(frag_shader);

  GLuint prog = glCreateProgram();
  glAttachShader(prog, vert_shader);
  glAttachShader(prog, frag_shader);
  glLinkProgram(prog);

  glDeleteShader(vert_shader);
  glDeleteShader(frag_shader);

  free((void *) vert_src);
  free((void *) frag_src);

  return prog;
}
