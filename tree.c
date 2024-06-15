#include <stdlib.h>

#include "tree.h"
#include "log.h"
#include "nerd.h"

qtree_node *qtree_create(vec2 center, GLfloat width, GLfloat height) {
  qtree_node *node = (qtree_node *) malloc(sizeof(qtree_node));
  node->center = center;
  node->width = width;
  node->height = height;
  node->circs = NULL;
  node->num_circs = 0;
  node->cap_circs = 0;
  node->nw = node->ne = node->sw = node->se = NULL;
  return node;
}

static void insert_into_leaf_node(qtree_node *n, KinematicCircle *c) {
  if (n->num_circs >= n->cap_circs) {
    n->cap_circs = (n->cap_circs == 0) ? 1 : n->cap_circs * 2;
    n->circs = (KinematicCircle *)
      realloc(n->circs, sizeof(KinematicCircle) * n->cap_circs);
    if (n->circs == NULL) PANIC_WITH(QUADTREE_ERR_REALLOC_FAIL);
  }
  n->circs[n->num_circs++] = *c;
}

static void subdivide_node(qtree_node *n) {
  n->nw = qtree_create((vec2){n->center.x - n->width / 4,
                              n->center.y + n->height / 4},
                              n->width / 2, n->height / 2);
  n->ne = qtree_create((vec2){n->center.x + n->width / 4,
                              n->center.y + n->height / 4},
                              n->width / 2, n->height / 2);
  n->sw = qtree_create((vec2){n->center.x - n->width / 4,
                              n->center.y - n->height / 4},
                              n->width / 2, n->height / 2);
  n->se = qtree_create((vec2){n->center.x + n->width / 4,
                              n->center.y - n->height / 4},
                              n->width / 2, n->height / 2);
}

void qtree_insert(qtree_node *n, KinematicCircle *c) {
  if (!circle_overlaps_quadrant(c, n)) return;
  if (n->nw == NULL) {
    insert_into_leaf_node(n, c);
    if (n->num_circs > QTREE_NODE_CAPACITY) {
      subdivide_node(n);
      for (size_t i = 0; i < n->num_circs; i++) {
        KinematicCircle c = n->circs[i];
        insert_into_subtree(n, &c);
      }
      free(n->circs);
      n->circs = NULL;
      n->num_circs = 0;
      n->cap_circs = 0;
    }
  } else {
    insert_into_subtree(n, c);
  }
}

static void remove_circle_from_node(qtree_node *n, KinematicCircle *c) {
  if (n->circs != NULL) {
    for (size_t i = 0; i < n->num_circs; i++) {
      if (&n->circs[i] == c) {
        n->circs[i] = n->circs[--n->num_circs];
        if (n->num_circs == 0) {
          free(n->circs);
          n->circs = NULL;
          n->cap_circs = 0;
        }
        break;
      }
    }
  }
}

static void update_circle_in_subtree(qtree_node *n, KinematicCircle *c) {
  if (circle_overlaps_quadrant(c, n->nw)) qtree_insert(n->nw, c);
  if (circle_overlaps_quadrant(c, n->ne)) qtree_insert(n->ne, c);
  if (circle_overlaps_quadrant(c, n->sw)) qtree_insert(n->sw, c);
  if (circle_overlaps_quadrant(c, n->se)) qtree_insert(n->se, c);
}

void qtree_update(qtree_node *n, KinematicCircle *c) {
  remove_circle_from_node(n, c);
  if (n->nw != NULL) update_circle_in_subtree(n, c);
  else insert_into_leaf_node(n, c);
}

static void apply_collision(KinematicCircle *c1, KinematicCircle *c2) {
  vec2 diff = (vec2) { c2->p.x - c1->p.x, c2->p.y - c1->p.y };
  GLfloat overlap = c1->rad + c2->rad - vec2mag(diff);
  if (overlap > 0.0f) physics_resolve_collision(c1, c2, diff, overlap);
}

static void apply_collisions_in_node(qtree_node *n) {
  if (n->circs != NULL) {
    for (size_t i = 0; i < n->num_circs; i++) {
      for (size_t j = i + 1; j < n->num_circs; j++) {
        apply_collision(&n->circs[i], &n->circs[j]);
      }
    }
  }
}

static void apply_collisions_between_nodes(qtree_node *n1, qtree_node *n2) {
  if (n1->circs != NULL && n2->circs != NULL) {
    for (size_t i = 0; i < n1->num_circs; i++) {
      for (size_t j = 0; j < n2->num_circs; j++) {
        apply_collision(&n1->circs[i], &n2->circs[j]);
      }
    }
  }
}

void qtree_apply_collisions(qtree_node *n) {
  apply_collisions_in_node(n);
  if (n->nw != NULL) {
    qtree_apply_collisions(n->nw);
    qtree_apply_collisions(n->ne);
    qtree_apply_collisions(n->sw);
    qtree_apply_collisions(n->se);
    apply_collisions_between_nodes(n->nw, n->ne);
    apply_collisions_between_nodes(n->nw, n->sw);
    apply_collisions_between_nodes(n->ne, n->se);
    apply_collisions_between_nodes(n->sw, n->se);
  }
}
