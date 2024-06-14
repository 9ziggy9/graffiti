#ifndef PHYSICS_H_
#define PHYSICS_H_
#include <GL/glew.h>
#include "nerd.h"

typedef struct {
  vec2 p; vec2 dp_dt; vec2 d2p_dt2;
  GLfloat m;
  GLfloat rad;
  GLuint color;
} KinematicCircle;

typedef struct { KinematicCircle *circs; size_t sz; size_t cap; } SpHashCell;

void physics_apply_boundaries(KinematicCircle *);
void physics_apply_collision(KinematicCircle *, int);
void physics_resolve_collision(KinematicCircle *, KinematicCircle *,
                               vec2, GLfloat);
#endif // PHYSICS_H_
