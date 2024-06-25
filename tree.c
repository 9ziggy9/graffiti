#include <stdio.h>
#include "tree.h"
#include "primitives.h"

BHNode *bhtree_create(vec2 min, vec2 max) {
  BHNode *node = (BHNode *) malloc(sizeof(BHNode));
  node->min = min; node->max = max;
  node->body_total = 0; node->is_partitioned = false;
  node->cm = (vec2){0.0, 0.0}; node->m = 0.0;
  for (int n = 0; n < MAX_CHILDREN; n++) {
    node->children[n] = NULL;
    node->bodies[n]   = NULL;
  }
  return node;
}

static void node_partition(BHNode *n) {
  vec2 delta_r = vec2sub(n->max, n->min);
  vec2 delta_x = vec2scale(0.5, vec2proj(delta_r, X_HAT));
  vec2 delta_y = vec2scale(0.5, vec2proj(delta_r, Y_HAT));
  vec2 delta_q = vec2add(delta_x, delta_y);
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 2; j++) {
      vec2 dmin = vec2add(vec2scale((double)i, delta_x),
                          vec2scale((double)j, delta_y));
      vec2 new_min = vec2add(n->min, dmin);
      vec2 new_max = vec2add(new_min, delta_q);
      n->children[i + 2 * j] = bhtree_create(new_min, new_max);
    }
  }
  n->is_partitioned = true;
}

static vec2 compute_cm(PhysicsEntity *bodies[], size_t body_total) {
  double M = 0;
  vec2 cm = (vec2){0.0, 0.0};
  for (size_t n = 0; n < body_total; n++) {
    cm = vec2add(cm, vec2scale(bodies[n]->m, bodies[n]->q));
    M += bodies[n]->m;
  }
  return vec2scale((1 / M), cm);
}

void bhtree_insert(BHNode *node, PhysicsEntity *body) {
  if (!body_in_bounds(node, body)) return;
  if (node->body_total < MAX_CHILDREN) {
    node->bodies[node->body_total++] = body;
    node->m += body->m;
    node->cm = compute_cm(node->bodies, node->body_total);
  } else {
    if (!node_is_partitioned(node)) node_partition(node);
    for (size_t n = 0; n < MAX_CHILDREN; n++)
      bhtree_insert(node->children[n], body);
  }
}

void bhtree_draw(BHNode *node) {
  if (!node) return;

  for (int i = 0; i < node->body_total; i++) {
    PhysicsEntity *body = node->bodies[i];
    draw_circle(body->q, (GLfloat) body->geom.circ.R, body->color);
  }

  for (int i = 0; i < MAX_CHILDREN; i++) {
    bhtree_draw(node->children[i]);
  }
}

void bhtree_print(BHNode *node) {
#define PRINT_VEC2(v) printf("{\"x\": %.2f, \"y\": %.2f}", v.x, v.y);
  if (!node) return;
  printf("{\n\"min\": "); PRINT_VEC2(node->min);
  printf(",\n\"max\": "); PRINT_VEC2(node->max);

  printf(",\ncm: "); PRINT_VEC2(node->cm);
  printf("\n\"bodies\": [");
  for (size_t i = 0; i < node->body_total; i++) {
    if (i > 0) printf(", ");
    PRINT_VEC2(node->bodies[i]->q);
  }
  printf("],\n\"children\": [");
  for (size_t i = 0; i < MAX_CHILDREN; i++) {
    if (node->children[i]) {
      if (i > 0) printf(", ");
      bhtree_print(node->children[i]);
    } else {
      if (i > 0) printf(", ");
      printf("null");
    }
  }
  printf("]\n}");
  if (node->children[MAX_CHILDREN - 1]) {
    printf(",\n");
  }
#undef PRINT_VEC2
}
