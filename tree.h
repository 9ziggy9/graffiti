#ifndef TREE_H_
#define TREE_H_
#include "physics.h"
#include <stdbool.h>

#define QTREE_NODE_CAPACITY 4

typedef struct qtree_node {
  struct qtree_node *nw, *ne, *sw, *se;
  vec2 center;
  KinematicCircle *circs;
  GLfloat width, height;
  size_t num_circs; size_t cap_circs;
} qtree_node;

qtree_node *qtree_create(vec2, GLfloat, GLfloat);
void qtree_insert(qtree_node *, KinematicCircle *);
void qtree_update(qtree_node *, KinematicCircle *);
void qtree_apply_collisions(qtree_node *);

static inline bool is_within_bounds(vec2 p, vec2 center,
                                    GLfloat width, GLfloat height)
{
  return (p.x >= center.x - width / 2 && p.x <= center.x + width / 2) &&
         (p.y >= center.y - height / 2 && p.y <= center.y + height / 2);
}

static inline bool circle_overlaps_quadrant(KinematicCircle *c, qtree_node *n) {
  return (c->p.x - c->rad <= n->center.x + n->width  / 2 &&
          c->p.x + c->rad >= n->center.x - n->width  / 2 &&
          c->p.y - c->rad <= n->center.y + n->height / 2 &&
          c->p.y + c->rad >= n->center.y - n->height / 2);
}

static inline void insert_into_subtree(qtree_node *n, KinematicCircle *c) {
  if (circle_overlaps_quadrant(c, n->nw)) qtree_insert(n->nw, c);
  if (circle_overlaps_quadrant(c, n->ne)) qtree_insert(n->ne, c);
  if (circle_overlaps_quadrant(c, n->sw)) qtree_insert(n->sw, c);
  if (circle_overlaps_quadrant(c, n->se)) qtree_insert(n->se, c);
}

#endif // TREE_H_
