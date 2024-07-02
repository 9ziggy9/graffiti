#include "tree.h"
#include "primitives.h"
#include "config.h"

#include <stdio.h>

BH_NODE_MAP(bhtree_draw, {
  PhysicsEntity *body = node->bodies[n];
  if (body) draw_circle(body->q, (GLfloat) body->geom.circ.R, body->color);
})

BH_NODE_MAP(bhtree_clear_forces, {
  PhysicsEntity *body = node->bodies[n];
  if (body) node->bodies[n]->d2q_dt2 = ((vec2){0.0f, 0.0f});
})

BH_NODE_MAP(bhtree_apply_boundaries, {
  PhysicsEntity *body = node->bodies[n];
  if (body) {
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
  }
})

static void node_partition(MemoryArena *arena, BHNode *node) {
  EACH_QUAD(node->min, node->max, {
      node->children[i + 2 * j] = bhtree_create(arena, __qmin, __qmax);
  });
  node->is_partitioned = true;
}

static void draw_quad(BHNode *node) {
  EACH_QUAD(node->min, node->max, {
    draw_rectangle_boundary(__qmin, __qmax, 0xFF0000FF);
  });
}

static Quad quad_map(BHNode *node, vec2 pos) {
  EACH_QUAD(node->min, node->max, {
      if (body_in_bounds(__qmin, __qmax, pos)) return i + 2 * j;
  });
  return -1;
}

void quad_state_log(OccState state) {
  printf("QUAD STATE: { ");
  if ((state & OCC_NW) >> 0) printf("NW, ");
  if ((state & OCC_NE) >> 1) printf("NE, ");
  if ((state & OCC_SW) >> 2) printf("SW, ");
  if ((state & OCC_SE) >> 3) printf("SE, ");
  printf("}\n");
}

void bhtree_draw_quads(BHNode *node, GLuint color) {
  if (!node) return;
  draw_quad(node);
  if (node->is_partitioned) {
    for (size_t n = 0; n < MAX_CHILDREN; n++) {
      bhtree_draw_quads(node->children[n], color);
    }
  }
}

static vec2 compute_cm(PhysicsEntity *bodies[], size_t body_total) {
  double M = 0;
  vec2 cm = (vec2){0.0, 0.0};
  for (size_t n = 0; n < body_total; n++) {
    PhysicsEntity *body = bodies[n];
    if (body) {
      cm = vec2add(cm, vec2scale(body->m, body->q));
      M += body->m;
    }
  }
  return vec2scale((1 / M), cm);
}

BHNode *bhtree_create(MemoryArena *arena, vec2 min, vec2 max) {
  BHNode *node = (BHNode *) arena_alloc(arena, sizeof(BHNode));
  node->min = min; node->max = max;
  node->body_total = 0; node->is_partitioned = false;
  node->occ_state = OCC_0;
  node->cm = (vec2){0.0, 0.0}; node->m = 0.0;
  for (int n = 0; n < MAX_CHILDREN; n++) node->children[n] = NULL;
  for (int n = 0; n < NUM_QUADS; n++)      node->bodies[n] = NULL;
  return node;
}

void bhtree_insert(MemoryArena *arena, BHNode *node, PhysicsEntity *body) {
  if (!body_in_bounds(node->min, node->max, body->q)) return;
  PhysicsEntity *cobody = NULL;
  const Quad quad_target = quad_map(node, body->q);
  if (node->body_total < NUM_QUADS) {
    OccState occ_collision = node->occ_state & quad_to_occ(quad_target);
    if (occ_collision) {
      cobody = node->bodies[quad_target];
      node->bodies[quad_target] = NULL;
      node->occ_state &= ~quad_to_occ(quad_target);
      node->m -= cobody->m;
      node->body_total--;
      node->cm = compute_cm(node->bodies, node->body_total);
      goto _l_bhtree_part;
    }
    node->bodies[quad_target] = body;
    node->occ_state |= quad_to_occ(quad_target);
    node->m += body->m;
    node->body_total++;
    node->cm = compute_cm(node->bodies, node->body_total);
    return;
  }
_l_bhtree_part:
  if (!(node->is_partitioned)) node_partition(arena, node); 
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    if (cobody) bhtree_insert(arena, node->children[n], cobody);
    bhtree_insert(arena, node->children[n], body);
  }
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
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (body) {
      if (flag & VERLET_POS) {
        body->q.x += body->dq_dt.x * dt + 0.5 * body->d2q_dt2.x * dt * dt;
        body->q.y += body->dq_dt.y * dt + 0.5 * body->d2q_dt2.y * dt * dt;
      }
      if (flag & VERLET_VEL) {
        body->dq_dt.x += 0.5 * body->d2q_dt2.x * dt;
        body->dq_dt.y += 0.5 * body->d2q_dt2.y * dt;
      }
    }
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++)
    bhtree_integrate(flag, node->children[n], dt);
}

void bhtree_apply_sink_force(BHNode *node, force_sink F) {
  if (!node) return;
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (body) F(body, 10000, (vec2){(double)WIN_W / 2, (double)WIN_H / 2});
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_sink_force(node->children[n], F);
  }
}

void bhtree_apply_pairconst_force(BHNode *node, force_fn F) {
  if (!node) return;
  for (size_t i = 0; i < NUM_QUADS; i++) {
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
