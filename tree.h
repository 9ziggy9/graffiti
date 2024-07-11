#ifndef TREE_H_
#define TREE_H_
#include <stdbool.h>
#include <math.h>

#include "physics.h"
#include "alloc.h"

#define MAX_CHILDREN 4
#define NUM_QUADS 4

typedef enum {
  VERLET_POS = 1,
  VERLET_VEL = 2,
  VERLET_ACC = 4,
} integration_flag;

typedef enum {
  OCC_0  = 0, // 0b0000
  OCC_SW = 1, // 0b0001
  OCC_NE = 2, // 0b0010
  OCC_NW = 4, // 0b0100
  OCC_SE = 8, // 0b1000
} OccState;

typedef enum {
  QUAD_SW = 0,
  QUAD_SE,
  QUAD_NW,
  QUAD_NE,
} Quad;

static inline OccState quad_to_occ(Quad q) {
 switch (q) {
  case QUAD_SW: return OCC_SW;
  case QUAD_NE: return OCC_NE;
  case QUAD_NW: return OCC_NW;
  case QUAD_SE: return OCC_SE;
  default: PANIC_WITH(OCC_QUAD_BAD_CONVERSION);
  }
}

static inline Quad occ_to_quad(const OccState s) {
  switch (s) {
  case OCC_SW: return QUAD_SW;
  case OCC_NE: return QUAD_NE;
  case OCC_NW: return QUAD_NW;
  case OCC_SE: return QUAD_SE;
  default: PANIC_WITH(OCC_QUAD_BAD_CONVERSION);
  }
}

typedef struct BHNode {
  struct BHNode *children[MAX_CHILDREN];
  PhysicsEntity   *bodies[NUM_QUADS];
  bool is_partitioned;
  size_t body_total;  // total physical objects
  OccState occ_state; // maps to quadrants
  vec2 min; vec2 max; // spatial bounds
  vec2 cm;            // center of mass
  double m;           // total mass
} BHNode;

static inline bool body_in_bounds(vec2 min, vec2 max, vec2 pos) {
  return pos.x >= min.x && pos.x < max.x
      && pos.y >= min.y && pos.y < max.y;
}

typedef void BH_NODE_MAPPING;
#define BH_NODE_MAP(FN, CODE)                                  \
  BH_NODE_MAPPING FN(BHNode *node) {                           \
    if (!node) return;                                         \
    for (size_t n = 0; n < NUM_QUADS; n++) CODE                \
    if (node->occ_state & OCC_SW) FN(node->children[QUAD_SW]); \
    if (node->occ_state & OCC_NW) FN(node->children[QUAD_NW]); \
    if (node->occ_state & OCC_NE) FN(node->children[QUAD_NE]); \
    if (node->occ_state & OCC_SE) FN(node->children[QUAD_SE]); \
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

static inline bool bh_condition(BHNode *n1, BHNode *n2, double theta) {
  return (vec2sub(n1->max, n1->min).x / vec2dist(n1->cm, n2->cm)) < theta;
}

BHNode *bhtree_create(MemoryArena *, vec2, vec2);
BHNode *bhtree_init(size_t N, PhysicsEntity[static N], MemoryArena[static 1]);

void bhtree_insert(MemoryArena *, BHNode *, PhysicsEntity *);
void bhtree_integrate(integration_flag, BHNode *, double);

typedef struct { vec2 nw, ne, sw, se; } BoundingBox;
BoundingBox generate_bounding_box(vec2, double);
void draw_bounding_box(BoundingBox, GLuint);

#define bhtree_apply_collisions(N) _bhtree_apply_collisions(N, N)
void _bhtree_apply_collisions(BHNode *node, BHNode *root);
void bhtree_apply_singular_gravity(BHNode *, vec2);

typedef struct {
  size_t length;
  BHNode **nodes;
} BHNodeRef;

BHNodeRef get_collision_nodes(MemoryArena[static 1], BHNode *, BoundingBox);
void least_bounding_node(BHNode *, BHNode **, BoundingBox);

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
