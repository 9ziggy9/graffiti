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

#define CONST_GRAVITY 2000.0f

typedef struct { KinematicCircle *circs; size_t sz; size_t cap; } SpHashCell;

void physics_apply_boundaries(KinematicCircle *, int);
void physics_apply_collision(KinematicCircle *, int);
void physics_apply_gravity(KinematicCircle *, int);

double physics_compute_gravitational_potential_energy(KinematicCircle *, int);
double physics_compute_kinetic_energy(KinematicCircle *, int);

#endif // PHYSICS_H_
