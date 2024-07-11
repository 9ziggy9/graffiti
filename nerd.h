#ifndef NERD_H_
#define NERD_H_
#include <GL/glew.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

typedef GLfloat mat4[4][4];

typedef struct vec2 { double x; double y; } vec2;

static inline double vec2dot(vec2 v1, vec2 v2) {
  return v1.x * v2.x + v1.y * v2.y;
}

static inline vec2 vec2scale(double s, vec2 v) {
  return (vec2) { s * v.x, s * v.y };
}

static inline double vec2mag(vec2 v) { return sqrt(vec2dot(v, v)); }

static inline vec2 vec2add(vec2 v1, vec2 v2) {
  return (vec2) { v1.x + v2.x, v1.y + v2.y };
}

static inline vec2 vec2sub(vec2 v1, vec2 v2) {
  return (vec2) {v1.x - v2.x, v1.y - v2.y};
}

static inline double vec2dist(vec2 v1, vec2 v2) {
  return vec2mag(vec2sub(v1, v2));
}

static inline vec2 vec2norm(vec2 v) {
  double len = vec2mag(v);
  return (vec2) { v.x / len, v.y / len };
}

// TODO: this normalizes even if r is a unit, possible optimize
static inline vec2 vec2proj(vec2 v, vec2 r) {
  const vec2 u = vec2norm(r);
  return vec2scale(vec2dot(v, u), u);
}

static inline double vec2area(vec2 diag) { return diag.x * diag.y; }

#define X_HAT (vec2) {1.0, 0.0}
#define Y_HAT (vec2) {0.0, 1.0}

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

#define SEED_RANDOM(N) do { srand(N); } while(0)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

GLint get_random(int, int);
GLuint get_random_color(void);
GLuint get_random_color_from_palette(void);

#endif // NERD_H_
