#include "tree.h"
#include "primitives.h"
#include "config.h"

#include <stdio.h>

BH_NODE_MAP(bhtree_draw, {
  PhysicsEntity *body = node->bodies[n];
  draw_circle(body->q, (GLfloat) body->geom.circ.R, body->color);
})

BH_NODE_MAP(bhtree_clear_forces, {
  node->bodies[n]->d2q_dt2 = ((vec2){0.0f, 0.0f});
})

BH_NODE_MAP(bhtree_apply_boundaries, {
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

static void node_partition(MemoryArena *arena, BHNode *node) {
  EACH_QUAD(node->min, node->max, {
      node->children[i + 2 * j] = bhtree_create(arena, __qmin, __qmax);
  });
  node->is_partitioned = true;
}

static void draw_quad(BHNode *node, vec2 pos) {
  EACH_QUAD(node->min, node->max, {
    if (body_in_bounds(__qmin, __qmax, pos))
      draw_rectangle_boundary(__qmin, __qmax, 0xFF0000FF);
  });
}

static OccState map_to_quadrant(BHNode *node, vec2 pos) {
  EACH_QUAD(node->min, node->max, {
    if (body_in_bounds(__qmin, __qmax, pos)) return pow(2, i + 2 * j);
  });
  return OCC_0;
}

void bhtree_draw_quads(BHNode *node, GLuint color) {
  if (!node) return;
  for (size_t b = 0; b < node->body_total; b++)
    draw_quad(node, node->bodies[b]->q);
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    BHNode *child = node->children[n];
    if (child) bhtree_draw_quads(child, color);
  }
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

BHNode *bhtree_create(MemoryArena *arena, vec2 min, vec2 max) {
  BHNode *node = (BHNode *) arena_alloc(arena, sizeof(BHNode));
  node->min = min; node->max = max;
  node->body_total = 0; node->is_partitioned = false;
  node->occ_state = OCC_0;
  node->cm = (vec2){0.0, 0.0}; node->m = 0.0;
  for (int n = 0; n < MAX_CHILDREN; n++) {
    node->children[n] = NULL;
    node->bodies[n]   = NULL;
  }
  return node;
}

void bhtree_insert(MemoryArena *arena, BHNode *node, PhysicsEntity *body) {
  if (!body_in_bounds(node->min, node->max, body->q)) return;
  if (node->body_total < NUM_QUADS) {
    printf("BODY MAPPING TO QUAD: %zu\n",
           (size_t)map_to_quadrant(node, body->q));
    node->bodies[node->body_total++] = body;
    node->m += body->m;
    node->cm = compute_cm(node->bodies, node->body_total);
    return;
  }
  if (node_is_partitioned(node)) {
    for (size_t n = 0; n < MAX_CHILDREN; n++)
      bhtree_insert(arena, node->children[n], body);
    return;
  }
  node_partition(arena, node); 
  return;
}

BHNode *
bhtree_build_in_arena(MemoryArena *arena, PhysicsEntity *particles, size_t N) {
  BHNode *bh = bhtree_create(arena, (vec2){0.0, 0.0}, (vec2){WIN_W, WIN_H});
  for (size_t n = 0; n < N; n++) bhtree_insert(arena, bh, &particles[n]);
  return bh;
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

void bhtree_apply_sink_force(BHNode *node, force_sink F) {
  if (!node) return;
  for (size_t n = 0; n < node->body_total; n++) {
    F(node->bodies[n], 10000, (vec2){(double)WIN_W / 2, (double)WIN_H / 2});
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_sink_force(node->children[n], F);
  }
}

void bhtree_apply_pairconst_force(BHNode *node, force_fn F) {
  if (!node) return;
  for (size_t i = 0; i < node->body_total; i++) {
    for (size_t j = i + 1; j < node->body_total; j++) {
      F(node->bodies[i], node->bodies[j]);
      F(node->bodies[j], node->bodies[i]);
    }
  }
  for (size_t c = 0; c < MAX_CHILDREN; c++)
    bhtree_apply_pairconst_force(node->children[c], F);
}

#if 0 // deprecated
BHSystem new_bh_system(BHNode *tree) {
  va_list args;
  va_start(args, tree);
    size_t forces_total = 0; 
    force_fn force;
    while ((force = va_arg(args, force_fn)) != NULL) forces_total++;
  va_end(args);
    
  force_fn *forces =
    (force_fn *)arena_alloc(arena, forces_total * sizeof(force_fn));

  va_start(args, tree); 
    for (size_t i = 0; i < forces_total; i++) forces[i] = va_arg(args,force_fn);
  va_end(args);

  return (BHSystem) { tree, forces, forces_total };
}
#endif
