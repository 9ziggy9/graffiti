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
void bhtree_integrate(integration_flag, BHNode *, double);

typedef void BH_MAPPING;

#define BH_TREE_MAP_1(FN, CODE)                        \
  BH_MAPPING FN(BHNode *node) {                        \
    if (!node) return;                                 \
    for (size_t n = 0; n < node->body_total; n++) CODE \
    for (size_t n = 0; n < MAX_CHILDREN; n++)          \
      FN(node->children[n]);                           \
  }                                                    \

BH_MAPPING bhtree_draw(BHNode *);
BH_MAPPING bhtree_clear_forces(BHNode *);
BH_MAPPING bhtree_apply_boundaries(BHNode *);
BH_MAPPING bhtree_print(BHNode *);


#endif // TREE_H_
