#ifndef NERD_H_
#define NERD_H_
#include <GL/glew.h>

typedef GLfloat mat4[4][4];
typedef struct vec2 { GLfloat x; GLfloat y; } vec2;

#define ID_MAT4          \
         {{1, 0, 0, 0},  \
          {0, 1, 0, 0},  \
          {0, 0, 1, 0},  \
          {0, 0, 0, 1}}

#define ORTHO_PROJ                     \
         {{2.0f / WIN_W,   0.0f,  0.0f, 0.0f}, \
          {0.0f,   2.0f / WIN_H,  0.0f, 0.0f}, \
          {0.0f,   0.0f, -1.0f, 0.0f}, \
          {-1.0f, -1.0f,  0.0f, 1.0f}}

#define WIN_CENTER ((vec2){WIN_W * 0.5f, WIN_H * 0.5f})

#endif // NERD_H_
