#ifndef PHYSICS_H_
#define PHYSICS_H_
#include <GL/glew.h>
#include <stdarg.h>
#include "nerd.h"

typedef enum { BOUNDARY_INF_BOX, BOUNDARY_TOROID } boundary_t;

typedef enum { GEOM_NONE, GEOM_CIRCLE } geometry_t;

typedef union {
  void *none;
  struct { double R; } circ;
} Geometry;

typedef struct PhysicsEntity {
  vec2 q; vec2 dq_dt; vec2 d2q_dt2;
  double m;
  GLuint color;
  geometry_t geom_t;
  Geometry geom;
} PhysicsEntity;

typedef void (*force_fn)(PhysicsEntity *, PhysicsEntity *);

typedef struct PhysicsSystem {
  PhysicsEntity *entities;
  size_t num_entities;
  force_fn *forces;
  size_t forces_total;
} PhysicsSystem;

void forces_apply_internal(PhysicsSystem *);
void forces_apply_external(PhysicsSystem *);

void force_pairwise_gravity(PhysicsEntity *, PhysicsEntity *);
void force_pairwise_impulsive_collision(PhysicsEntity *, PhysicsEntity *);

PhysicsEntity new_physics_entity(vec2, vec2, vec2, double, GLuint);
void physics_entity_bind_geometry(PhysicsEntity *, geometry_t, Geometry);

#define __NULL_SENT(...) __VA_ARGS__, NULL
#define new_physics_system(N, E, ...) \
  _new_physics_system(N, E, __NULL_SENT(__VA_ARGS__))
PhysicsSystem _new_physics_system(size_t, PhysicsEntity *, ...);

#define BEGIN_PHYSICS(DT)                   \
  static double __t0 = 0.0f;                \
  static double __acc = 0.0f;               \
  static const double __dt = 1.0f / 300.0f; \
  static const double DT = __dt;            \
  double __t1 = glfwGetTime();              \
  double __delta_t = __t1 - __t0;           \
  __t0 = __t1;                              \
  __acc += __delta_t;                       \
  while (__acc >= __dt) {                   
#define END_PHYSICS()                       \
    __acc -= __dt;                          \
  }


#endif // PHYSICS_H_
