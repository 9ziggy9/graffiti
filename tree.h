#ifndef TREE_H_
#define TREE_H_
#include "physics.h"
#include <stdbool.h>

#define MAX_BODIES_PER_NODE 4

typedef struct QTreeNode {
  bool is_partitioned;
  struct QTreeNode *children[MAX_BODIES_PER_NODE];
  PhysicsEntity *bodies[MAX_BODIES_PER_NODE];
  int body_total;
  vec2 min; vec2 max;
} QTreeNode;

static inline vec2 compute_node_center(QTreeNode *n) {
  return vec2scale(0.5, vec2sub(n->max, n->min));
}

static inline bool body_in_bounds(QTreeNode *n, PhysicsEntity *b) {
  return b->q.x >= n->min.x && b->q.x <= n->max.x
      && b->q.y >= n->min.y && b->q.y <= n->max.y;
}

QTreeNode *qtree_create(vec2, vec2);
void qtree_insert(QTreeNode *, PhysicsEntity *);
void qtree_print_json(QTreeNode *, int);

#endif // TREE_H_
