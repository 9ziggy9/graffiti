#ifndef PHYSICS_H_
#define PHYSICS_H_
#include <GL/glew.h>
#include "nerd.h"

typedef enum {
  BOUNDARY_INF_BOX,
  BOUNDARY_TOROID,
} boundary_t;

typedef enum {
  GEOM_NONE,
  GEOM_CIRCLE,
} geometry_t;

typedef union {
  void *none;
  struct { double R; } circ;
} Geometry;

typedef struct {
  vec2 q; vec2 dq_dt; vec2 d2q_dt2;
  double m;
  GLuint color;
  geometry_t geom_t;
  Geometry geom;
} PhysicsEntity;

#define CONST_GRAVITY 0.9f

PhysicsEntity new_physics_entity(vec2, vec2, vec2, double, GLuint);
void physics_entity_bind_geometry(PhysicsEntity *, geometry_t, Geometry);

void physics_apply_boundaries(PhysicsEntity *, int, boundary_t);
void physics_apply_collision(PhysicsEntity *, int);
void physics_apply_pairwise_gravity(PhysicsEntity *, int);

double physics_compute_gravitational_potential_energy(PhysicsEntity *, int);
double physics_compute_kinetic_energy(PhysicsEntity *, int);

#endif // PHYSICS_H_
