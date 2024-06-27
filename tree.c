#include "tree.h"
#include "primitives.h"
#include "config.h"

#include <stdio.h>

BH_TREE_MAP_1(bhtree_draw, {
  PhysicsEntity *body = node->bodies[n];
  draw_circle(body->q, (GLfloat) body->geom.circ.R, body->color);
})

BH_TREE_MAP_1(bhtree_clear_forces, {
  node->bodies[n]->d2q_dt2 = ((vec2){0.0f, 0.0f});
})

BH_TREE_MAP_1(bhtree_apply_boundaries, {
  PhysicsEntity *body = node->bodies[n];
  if (body->q.x - body->geom.circ.R <= 0.0) {
    body->dq_dt.x *= -1;
    body->q.x = body->geom.circ.R;
  } else if (body->q.x + body->geom.circ.R >= WIN_W) {
    body->dq_dt.x *= -1;
    body->q.x = WIN_W - body->geom.circ.R;
  }
  if (body->q.y - body->geom.circ.R <= 0.0) {
    body->dq_dt.y *= -1;
    body->q.y = body->geom.circ.R;
  } else if (body->q.y + body->geom.circ.R >= WIN_H) {
    body->dq_dt.y *= -1;
    body->q.y = WIN_H - body->geom.circ.R;
  }
})

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

void bhtree_integrate(integration_flag flag, BHNode *node, double dt)
{
  if (!node) return;
  for (size_t n = 0; n < node->body_total; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (flag & VERLET_POS) {
      body->q.x += body->dq_dt.x * dt + 0.5 * body->d2q_dt2.x * dt * dt;
      body->q.y += body->dq_dt.y * dt + 0.5 * body->d2q_dt2.y * dt * dt;
    }
    if (flag & VERLET_VEL) {
      body->dq_dt.x += 0.5 * body->d2q_dt2.x * dt;
      body->dq_dt.y += 0.5 * body->d2q_dt2.y * dt;
    }
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++)
    bhtree_integrate(flag, node->children[n], dt);
}

BHSystem _new_bh_system(BHNode *tree, ...) {
  va_list args;
  va_start(args, tree);
    size_t forces_total = 0; 
    force_fn force;
    while ((force = va_arg(args, force_fn)) != NULL) forces_total++;
  va_end(args);
    
  force_fn *forces = (force_fn *)malloc(forces_total * sizeof(force_fn));

  va_start(args, tree); 
    for (size_t i = 0; i < forces_total; i++) forces[i] = va_arg(args,force_fn);
  va_end(args);

  return (BHSystem) { tree, forces, forces_total };
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
