#ifndef TREE_H_
#define TREE_H_
#include "physics.h"
#include "alloc.h"

#include <stdbool.h>

#define MAX_CHILDREN 4
#define NUM_QUADS 4

typedef enum {
  VERLET_POS = 1,
  VERLET_VEL = 2,
  VERLET_ACC = 4,
} integration_flag;

typedef enum {
  OCC_0  = 0, // 0b0000
  OCC_NW = 1, // 0b0001
  OCC_NE = 2, // 0b0010
  OCC_SW = 4, // 0b0100
  OCC_SE = 8, // 0b1000
} OccState;

static inline size_t occ_count(OccState state) {
  return ((state & OCC_SE) >> 3)
       + ((state & OCC_SW) >> 2)
       + ((state & OCC_NE) >> 1)
       + ((state & OCC_NW) >> 0);
}

typedef struct BHNode {
  bool is_partitioned;
  struct BHNode *children[MAX_CHILDREN];
  PhysicsEntity   *bodies[NUM_QUADS];
  size_t  body_total; // total physical objects
  OccState occ_state; // maps to quadrants
  vec2 min; vec2 max; // spatial bounds
  vec2 cm;            // center of mass
  double m;           // total mass
} BHNode;

static inline bool body_in_bounds(vec2 min, vec2 max, vec2 pos) {
  return pos.x >= min.x && pos.x < max.x
      && pos.y >= min.y && pos.y < max.y;
}

static inline bool node_is_partitioned(BHNode *n) {
  return n->children[0] != NULL;
}

BHNode *bhtree_create(MemoryArena *, vec2, vec2);
BHNode *bhtree_build_in_arena(MemoryArena *, PhysicsEntity *, size_t);

typedef void BH_NODE_MAPPING;
#define BH_NODE_MAP(FN, CODE)                          \
  BH_NODE_MAPPING FN(BHNode *node) {                   \
    if (!node) return;                                 \
    for (size_t n = 0; n < node->body_total; n++) CODE \
    for (size_t n = 0; n < MAX_CHILDREN; n++)          \
      FN(node->children[n]);                           \
  }                                                    
BH_NODE_MAPPING bhtree_draw(BHNode *);
BH_NODE_MAPPING bhtree_draw_quads(BHNode *, GLuint);
BH_NODE_MAPPING bhtree_clear_forces(BHNode *);
BH_NODE_MAPPING bhtree_apply_boundaries(BHNode *);

#define EACH_QUAD(MIN, MAX, CODE) {                         \
  do {                                                      \
  vec2 delta_r = vec2sub(MAX, MIN);                         \
  vec2 delta_x = vec2scale(0.5, vec2proj(delta_r, X_HAT));  \
  vec2 delta_y = vec2scale(0.5, vec2proj(delta_r, Y_HAT));  \
  vec2 delta_q = vec2add(delta_x, delta_y);                 \
  for (size_t i = 0; i < 2; i++) {                          \
    for (size_t j = 0; j < 2; j++) {                        \
      vec2 dmin = vec2add(vec2scale((double)i, delta_x),    \
                          vec2scale((double)j, delta_y));   \
      vec2 __qmin = vec2add(MIN, dmin);                     \
      vec2 __qmax = vec2add(__qmin, delta_q);               \
      CODE;                                                 \
    }                                                       \
  }                                                         \
  } while(0);                                               \
}

void bhtree_apply_sink_force(BHNode *, force_sink);
void bhtree_apply_pairconst_force(BHNode *, force_fn);
void bhtree_insert(MemoryArena *, BHNode *, PhysicsEntity *);
void bhtree_integrate(integration_flag, BHNode *, double);

#endif // TREE_H_

#if 0 // deprecated
typedef struct {
  BHNode *root;
  force_fn *frcs_int;
  force_fn *frcs_ext;
  size_t frcs_int_tot;
  size_t frcs_ext_tot;
} BHSystem;

BHSystem new_bh_system(MemoryArena *, BHNode *);
void bhtree_apply_internal_forces(BHSystem *, BHNode *, double);
#endif
