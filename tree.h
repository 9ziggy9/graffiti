#ifndef TREE_H_
#define TREE_H_
#include "physics.h"

typedef struct qtree_node {
  struct qtree_node *nw, *ne, *sw, *se;
  GLfloat width, height;
  KinematicCircle *circ; vec2 center;
} qtree_node;

qtree_node *qtree_create(vec2 center, GLfloat width, GLfloat height);
void qtree_insert(qtree_node *node, KinematicCircle *circ);
void qtree_free(qtree_node *node);
void qtree_update(qtree_node *, KinematicCircle *, vec2);
void qtree_query(qtree_node *, KinematicCircle *, KinematicCircle **, int *);
void qtree_apply_collision(qtree_node *, KinematicCircle *);

#endif // TREE_H_
