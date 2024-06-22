#include <stdio.h>
#include "tree.h"
#include "log.h"

QTreeNode *qtree_create(vec2 min, vec2 max) {
  QTreeNode *node = (QTreeNode *) malloc(sizeof(QTreeNode));
  node->min = min; node->max = max;
  node->body_total = 0; node->is_partitioned = false;
  for (int n = 0; n < MAX_BODIES_PER_NODE; n++) {
    node->children[n] = NULL;
    node->bodies[n]   = NULL;
  }
  return node;
}

static void node_partition(QTreeNode *n) {
  vec2 delta_r = vec2sub(n->max, n->min);
  vec2 delta_x = vec2scale(0.5, vec2proj(delta_r, X_HAT));
  vec2 delta_y = vec2scale(0.5, vec2proj(delta_r, Y_HAT));
  vec2 delta_q = vec2add(delta_x, delta_y);
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 2; j++) {
      vec2 dmin = vec2add(vec2scale(i, delta_x), vec2scale(j, delta_y));
      n->children[i + 2 * j] = qtree_create(vec2add(n->min, dmin), delta_q);
    }
  }
  n->is_partitioned = true;
}

static size_t quad_idx(QTreeNode *n, PhysicsEntity *b) {
  size_t quad = 0;
  if (b->q.x > n->center.x) quad += 1;
  if (b->q.y > n->center.y) quad += 2;
  return quad;
}

static void node_insert_at(QTreeNode *node, PhysicsEntity *body) {
  if (node->body_total < MAX_BODIES_PER_NODE) {
    node->bodies[node->body_total++] = body; return;
  }
  if (!node->is_partitioned) {
    node_partition(node);
    for (int n = 0; n < MAX_BODIES_PER_NODE; n++) {
      node_insert_at(node->children[quad_idx(node, body)], node->bodies[n]);
    }
    node->body_total = 0;
  }
  node_insert_at(node->children[quad_idx(node, body)], body);
}

void qtree_insert(QTreeNode *root, PhysicsEntity *body) {
  if (body_in_bounds(root, body)) node_insert_at(root, body);
}

void qtree_print_json(QTreeNode *node, int depth) {
  if (node == NULL) { INFO_LOG(" < EMPTY QUADTREE > "); return; }
  INFO_LOG(" START: < QUADTREE > ");
  printf("{\n\"center\":{\"x\":%f,\"y\":%f},\n",node->center.x,node->center.y);
  printf("\"radius\": %f,\n", node->radius);
  printf("\"body_total\": %d,\n", node->body_total);
  printf("\"is_partitioned\": %s,\n", node->is_partitioned ? "true" : "false");
  printf("\"children\": [\n");
  for (int i = 0; i < MAX_BODIES_PER_NODE; i++) {
    if (node->children[i] != NULL) {
      for (int j = 0; j < depth + 1; j++) printf("  ");
      printf("[\n");
      qtree_print_json(node->children[i], depth + 1);
      for (int j = 0; j < depth + 1; j++) printf("  ");
      printf("]\n");
    }
  }
  printf("]\n}\n");
  INFO_LOG(" END: < QUADTREE > ");
}
