#ifndef TREE_H_
#define TREE_H_
#include "physics.h"
#include <stdbool.h>

#define MAX_CHILDREN 4

// TODO: make sure different methods are exclusive
typedef enum {
  VERLET_POS = 1,
  VERLET_VEL = 2,
  VERLET_ACC = 4,
} integration_flag;

typedef struct BHNode {
  bool is_partitioned;
  struct BHNode *children[MAX_CHILDREN];
  PhysicsEntity   *bodies[MAX_CHILDREN];
  size_t body_total;  // total physical objects
  vec2 min; vec2 max; // spatial bounds
  vec2 cm;            // center of mass
  double m;           // total mass
} BHNode;

static inline bool body_in_bounds(BHNode *n, PhysicsEntity *b) {
  return b->q.x >= n->min.x && b->q.x <= n->max.x
      && b->q.y >= n->min.y && b->q.y <= n->max.y;
}

static inline bool node_is_partitioned(BHNode *n) {
  return n->children[0] != NULL;
}

BHNode *bhtree_create(vec2, vec2);
void bhtree_insert(BHNode *, PhysicsEntity *);
void bhtree_print(BHNode *);
void bhtree_draw(BHNode *);
void bhtree_clear_forces(BHNode *);
void bhtree_integrate(integration_flag, BHNode *, double);

#endif // TREE_H_
