#ifndef SHADER_H_
#define SHADER_H_
#include <GL/glew.h>

void OPEN_SHADER(GLuint);
void CLOSE_SHADER(void);
GLuint SHADER();
GLuint compile_simple_shader(const char *, const char *);

#endif // SHADER_H_
