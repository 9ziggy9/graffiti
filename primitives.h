#ifndef PRIMITIVES_H_
#define PRIMITIVES_H_
#include <GL/glew.h>
#include "nerd.h"

#define COLOR_UNIFORM(HEX) HEX,HEX,HEX
#define COLOR_NORM(HEX) {                    \
    ((float)((HEX >> 24) & 0xFF) / 255.0f),  \
    ((float)((HEX >> 16) & 0xFF) / 255.0f),  \
    ((float)((HEX >> 8)  & 0xFF) / 255.0f),  \
    ((float)((HEX >> 0)  & 0xFF) / 255.0f) }

struct vertex { GLfloat pos[3]; GLfloat color[4]; };

void draw_eqtriangle(vec2, GLfloat, GLfloat, GLuint, GLuint, GLuint);

#endif // PRIMITIVES_H_
