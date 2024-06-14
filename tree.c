#include <stdlib.h>
#include <stdbool.h>

#include "tree.h"
#include "nerd.h"

qtree_node *qtree_create(vec2 center, GLfloat w, GLfloat h) {
  qtree_node *node = (qtree_node *) malloc(sizeof(qtree_node));
  node->nw = node->ne = node->sw = node->se = NULL;
  node->circ = NULL;
  node->center = center; node->width = w; node->height = h;
  return node;
}

void qtree_insert(qtree_node *node, KinematicCircle *circ) {
  if (node->circ == NULL) { node->circ = circ; return; }
  if (node->nw == NULL) {
    GLfloat half_w = 0.5f * node->width;
    GLfloat half_h = 0.5f * node->height;
    node->nw = qtree_create((vec2){node->center.x - half_w,
                                   node->center.y + half_h}, half_w, half_h);
    node->ne = qtree_create((vec2){node->center.x + half_w,
                                   node->center.y + half_h}, half_w, half_h);
    node->sw = qtree_create((vec2){node->center.x - half_w,
                                   node->center.y - half_h}, half_w, half_h);
    node->se = qtree_create((vec2){node->center.x + half_w,
                                   node->center.y - half_h}, half_w, half_h);
  }
  vec2 dp = vec2sub(circ->p, node->center);
  GLfloat left   = circ->p.x - circ->rad;
  GLfloat right  = circ->p.x + circ->rad;
  GLfloat top    = circ->p.y + circ->rad;
  GLfloat bottom = circ->p.y - circ->rad;
  if (left < node->center.x && top > node->center.y)
    qtree_insert(node->nw, circ);
  if (right > node->center.x && top > node->center.y)
    qtree_insert(node->ne, circ);
  if (left < node->center.x && bottom < node->center.y)
    qtree_insert(node->sw, circ);
  if (right > node->center.x && bottom < node->center.y)
    qtree_insert(node->se, circ);
}

static bool exited_quadrant(vec2 p, vec2 p_lst, vec2 center) {
  vec2 dp     = vec2sub(p, center);
  vec2 dp_lst = vec2sub(p_lst, center);
  return (dp.x < 0 && dp_lst.x >= 0) || (dp.x >= 0 && dp_lst.x < 0)
      || (dp.y < 0 && dp_lst.y >= 0) || (dp.y >= 0 && dp_lst.y < 0);
}

void qtree_update(qtree_node *node, KinematicCircle *circ, vec2 p_lst) {
  if (exited_quadrant(circ->p, p_lst, node->center)) {
    if (node->circ == circ) node->circ = NULL;
    else if (node->nw != NULL) {
      qtree_update(node->nw, circ, p_lst);
      qtree_update(node->ne, circ, p_lst);
      qtree_update(node->sw, circ, p_lst);
      qtree_update(node->se, circ, p_lst);
    }
    qtree_insert(node, circ);
  } else if (node->nw != NULL) {
    qtree_update(node->nw, circ, p_lst);
    qtree_update(node->ne, circ, p_lst);
    qtree_update(node->sw, circ, p_lst);
    qtree_update(node->se, circ, p_lst);
  }
}

void qtree_query(qtree_node *node,
                 KinematicCircle *circ,
                 KinematicCircle **collisions,
                 int *num_collisions)
{
  if (node->circ != NULL && node->circ != circ) {
    collisions[(*num_collisions)++] = node->circ;
  }
  if (node->nw != NULL) {
    vec2 dp = vec2sub(circ->p, node->center);
    if (dp.x < 0 && dp.y > 0)
      qtree_query(node->nw, circ, collisions, num_collisions);
    if (dp.x > 0 && dp.y > 0)
      qtree_query(node->ne, circ, collisions, num_collisions);
    if (dp.x < 0 && dp.y < 0)
      qtree_query(node->sw, circ, collisions, num_collisions);
    if (dp.x > 0 && dp.y < 0)
      qtree_query(node->se, circ, collisions, num_collisions);
  }
}

void qtree_apply_collision(qtree_node *node, KinematicCircle *circ) {
  KinematicCircle *collisions[100];
  int num_collisions = 0;
  qtree_query(node, circ, collisions, &num_collisions);
  for (int i = 0; i < num_collisions; i++) {
    vec2 diff = vec2sub(collisions[i]->p, circ->p);
    GLfloat overlap = circ->rad + collisions[i]->rad - vec2mag(diff);
    if (overlap > 0.0f) {
      physics_resolve_collision(circ, collisions[i], diff, overlap);
    }
  }
}

void qtree_free(qtree_node *node) {
  if (node->nw != NULL) {
    qtree_free(node->nw); qtree_free(node->ne);
    qtree_free(node->sw); qtree_free(node->se);
  }
  free(node);
}
