#include <stdio.h>

#include "tree.h"
#include "log.h"
#include "primitives.h"
#include "config.h"

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

static void draw_quad(BHNode *node) {
  if (!node) return;
  EACH_QUAD(node->min, node->max, {
    draw_rectangle_boundary(__qmin, __qmax, 0xFF0000FF);
  });
}

static Quad quad_map(BHNode *node, vec2 pos) {
  EACH_QUAD(node->min, node->max, {
      if (body_in_bounds(__qmin, __qmax, pos)) return i + 2 * j;
  });
  printf("HERE IS YOUR PROBLEM: RETURNING -1 in QUAD MAP!\n");
  return -1;
}

void bhtree_draw_quads(BHNode *node, GLuint color) {
  if (!node) return;
  draw_quad(node);
  draw_circle(node->cm, 3.0, 0xFFFFFFFF);
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_draw_quads(node->children[n], color);
  }
}

BHNode *bhtree_create(MemoryArena *arena, vec2 min, vec2 max) {
  BHNode *node = (BHNode *) arena_alloc(arena, sizeof(BHNode));
  node->min = min; node->max = max;
  node->body_total = 0; node->is_partitioned = false;
  node->occ_state = OCC_0;
  node->cm = (vec2){-1.0, -1.0};
  node->m = 0.0;
  for (int n = 0; n < MAX_CHILDREN; n++) node->children[n] = NULL;
  for (int n = 0; n < NUM_QUADS;    n++)   node->bodies[n] = NULL;
  return node;
}

BHNode *
bhtree_init(size_t N,
            PhysicsEntity particles[static N],
            MemoryArena arena[static 1])
{
  BHNode *bh = bhtree_create(arena, (vec2){0.0, 0.0}, (vec2){WIN_W, WIN_H});
  for (size_t n = 0; n < N; n++) bhtree_insert(arena, bh, &particles[n]);
  return bh;
}

static vec2 quad_partition_min(vec2 min, vec2 max, Quad q) {
  vec2 delta_r = vec2sub(max, min);
  vec2 delta_x = vec2scale(0.5, vec2proj(delta_r, X_HAT));
  vec2 delta_y = vec2scale(0.5, vec2proj(delta_r, Y_HAT));
  vec2 delta_q = vec2add(delta_x, delta_y);
  switch (q) {
  case QUAD_NW: return vec2add(min, delta_y);
  case QUAD_NE: return vec2add(min, delta_q);
  case QUAD_SW: return min;
  case QUAD_SE: return vec2add(min, delta_x);
  default: PANIC_WITH(OCC_QUAD_BAD_CONVERSION);
  }
}

static vec2 quad_partition_max(vec2 min, vec2 max, Quad q) {
  vec2 delta_r = vec2sub(max, min);
  vec2 delta_x = vec2scale(0.5, vec2proj(delta_r, X_HAT));
  vec2 delta_y = vec2scale(0.5, vec2proj(delta_r, Y_HAT));
  vec2 delta_q = vec2add(delta_x, delta_y);
  vec2 center  = vec2add(min, delta_q);
  switch (q) {
  case QUAD_NW: return vec2add(center, delta_y);
  case QUAD_NE: return vec2add(center, delta_q);
  case QUAD_SW: return center;
  case QUAD_SE: return vec2add(center, delta_x);
  default: PANIC_WITH(OCC_QUAD_BAD_CONVERSION);
  }
}

static BHNode *child_partition(MemoryArena *arena, BHNode *parent, Quad q) {
  vec2 min = quad_partition_min(parent->min, parent->max, q);
  vec2 max = quad_partition_max(parent->min, parent->max, q);
  parent->children[q] = bhtree_create(arena, min, max);
  parent->is_partitioned = true;
  return parent->children[q];
}

static void update_cm(BHNode *node, PhysicsEntity *body) {
  node->m += body->m;
  node->body_total++;
  node->cm = vec2scale(1 / (node->m),
                       vec2add(vec2scale(node->m - body->m, node->cm),
                               vec2scale(body->m, body->q)));
}

static void insert_body(BHNode *node, PhysicsEntity *body) {
  const Quad quad_target = quad_map(node, body->q);
  node->bodies[quad_target] = body;
  node->occ_state |= quad_to_occ(quad_target);
}

static PhysicsEntity *remove_body_at_quad(BHNode *node, Quad quad) {
  PhysicsEntity *body = node->bodies[quad];
  if (body) node->bodies[quad] = NULL;
  return body;
}

void bhtree_insert(MemoryArena *arena, BHNode *node, PhysicsEntity *body) {
  if (!node) return;
  if (!body_in_bounds(node->min, node->max, body->q)) return;

  update_cm(node, body);

  const Quad quad_target = quad_map(node, body->q);
  if (!(node->occ_state & quad_to_occ(quad_target))) {
    insert_body(node, body);
    return;
  }

  PhysicsEntity *cobody = remove_body_at_quad(node, quad_target);
  if (cobody) {
    BHNode *child = child_partition(arena, node, quad_target);
    if (!child) PANIC_WITH(BH_CHILD_NODE_DOES_NOT_EXIST);
    bhtree_insert(arena, child, body);
    bhtree_insert(arena, child, cobody);
    return;
  }

  for (size_t n = 0; n < MAX_CHILDREN; n++)
    bhtree_insert(arena, node->children[n], body);

  return;
}

void bhtree_draw(BHNode *node) {
  if (!node) return;
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (body) draw_circle(body->q, (GLfloat)body->geom.circ.R, body->color);
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) bhtree_draw(node->children[n]);
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

BoundingBox generate_bounding_box(vec2 pos, double l) {
  return (BoundingBox) {
    (vec2){pos.x - l, pos.y + l},
    (vec2){pos.x + l, pos.y + l},
    (vec2){pos.x - l, pos.y - l},
    (vec2){pos.x + l, pos.y - l},
  };
}

void draw_bounding_box(BoundingBox bbox, GLuint color) {
  draw_rectangle_filled(bbox.sw, bbox.ne, color);
}

static BHNodeRef bhnoderef_init(MemoryArena arena[static 1]) {
#define MAX_NODES 100
  return (BHNodeRef) {
    .nodes = (BHNode **) arena_alloc(arena, MAX_NODES * sizeof(BHNode *)),
  };
#undef MAX_NODES
}

static void bhnoderef_append(BHNodeRef *ref, BHNode *node) {
  ref->nodes[ref->length++] = node;
}

static bool bbox_total_bound(BHNode *node, BoundingBox box) {
  return body_in_bounds(node->min, node->max, box.sw)
      && body_in_bounds(node->min, node->max, box.nw)
      && body_in_bounds(node->min, node->max, box.ne)
      && body_in_bounds(node->min, node->max, box.se);
}

static bool bbox_corner_bound(BHNode *node, BoundingBox box) {
  return body_in_bounds(node->min, node->max, box.sw)
      || body_in_bounds(node->min, node->max, box.nw)
      || body_in_bounds(node->min, node->max, box.ne)
      || body_in_bounds(node->min, node->max, box.se);
}

static bool bbox_outbound(BoundingBox box) {
  return
     (box.sw.x < 0 || box.sw.x >= WIN_W || box.sw.y < 0 || box.sw.y >= WIN_H)
  || (box.nw.x < 0 || box.nw.x >= WIN_W || box.nw.y < 0 || box.nw.y >= WIN_H)
  || (box.ne.x < 0 || box.ne.x >= WIN_W || box.ne.y < 0 || box.ne.y >= WIN_H)
  || (box.se.x < 0 || box.se.x >= WIN_W || box.se.y < 0 || box.se.y >= WIN_H);
}

static void __acc_collision_nodes(BHNodeRef *ref, BHNode *node,
                                  BoundingBox box, double theta)
{
  if (!node) return;
  if (bbox_corner_bound(node, box)) {
    if (bbox_total_bound(node, box)) {
      if (node->occ_state & OCC_SW)
        __acc_collision_nodes(ref, node->children[QUAD_SW], box, theta);
      if (node->occ_state & OCC_NW)
        __acc_collision_nodes(ref, node->children[QUAD_NW], box, theta);
      if (node->occ_state & OCC_NE)
        __acc_collision_nodes(ref, node->children[QUAD_NE], box, theta);
      if (node->occ_state & OCC_SE)
        __acc_collision_nodes(ref, node->children[QUAD_SE], box, theta);
    } else {
      vec2 ndiag = vec2sub(node->max, node->min);
      vec2 bdiag = vec2sub(box.ne, box.sw);
      if ((vec2area(ndiag) / vec2area(bdiag)) > theta) {
        bhnoderef_append(ref, node);
      }
    }
  }
}

BHNodeRef
get_collision_nodes(MemoryArena arena[static 1], BHNode *root, BoundingBox box)
{
  BHNodeRef ref = bhnoderef_init(arena);
  __acc_collision_nodes(&ref, root, box, 0.5);
  return ref;
}

// TODO: strange behavior at boundaries!
void least_bounding_node(BHNode *node, BHNode **lnode, BoundingBox box) {
  if (!node) return;
  if (bbox_total_bound(node, box)) {
    *lnode = node;
    if (node->occ_state & OCC_SW)
      least_bounding_node(node->children[QUAD_SW], lnode, box);
    if (node->occ_state & OCC_NW)
      least_bounding_node(node->children[QUAD_NW], lnode, box);
    if (node->occ_state & OCC_NE)
      least_bounding_node(node->children[QUAD_NE], lnode, box);
    if (node->occ_state & OCC_SE)
      least_bounding_node(node->children[QUAD_SE], lnode, box);
  }
}

static void
bhtree_apply_subcollisions(size_t i, PhysicsEntity *p_i, BHNode *node)
{
  if (!node) return;
  for (size_t j = 0; j < NUM_QUADS; j++) {
    PhysicsEntity *p_j = node->bodies[j];
    if (p_i != p_j && p_j) force_pairwise_impulsive_collision(p_i, p_j);
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_subcollisions(-1, p_i, node->children[n]);
  }
}

void _bhtree_apply_collisions(BHNode *node, BHNode *root) {
  if (!node) return;
  for (size_t i = 0; i < NUM_QUADS; i++) {
    PhysicsEntity *p_i = node->bodies[i];
    if (p_i) {
      BoundingBox pbox = generate_bounding_box(p_i->q, p_i->geom.circ.R);
      BHNode *lnode = root;
      least_bounding_node(root, &lnode, pbox);
      bhtree_apply_subcollisions(i, p_i, lnode);
    }
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    _bhtree_apply_collisions(node->children[n], root);
  }
}

void bhtree_apply_singular_gravity(BHNode *node, vec2 sink_source) {
  (void) sink_source;
  if (!node) return;
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (body) force_singular_gravity(body, sink_source);
  }
  if (node->is_partitioned) {
    for (size_t n = 0; n < MAX_CHILDREN; n++)
      bhtree_apply_singular_gravity(node->children[n], sink_source);
  }
}

//  ------ DEAD ZONE --------

#if 0 // deprecated
static void
bhtree_apply_subcollisions(size_t i, PhysicsEntity *p_i, BHNode *node)
{
  if (!node) return;
  for (size_t j = i + 1; j < NUM_QUADS; j++) {
    PhysicsEntity *p_j = node->bodies[j];
    if (p_i && p_j) force_pairwise_impulsive_collision(p_i, p_j);
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_subcollisions(-1, p_i, node->children[n]);
  }
}

void bhtree_apply_collisions(BHNode *node) {
  if (!node) return;
  for (size_t i = 0; i < NUM_QUADS; i++) {
    PhysicsEntity *p_i = node->bodies[i];
    bhtree_apply_subcollisions(i, p_i, node);
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_collisions(node->children[n]);
  }
}

void bhtree_apply_singular_gravity(BHNode *node, vec2 sink_source) {
  (void) sink_source;
  if (!node) return;
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *body = node->bodies[n];
    if (body) force_singular_gravity(body, sink_source);
  }
  if (node->is_partitioned) {
    for (size_t n = 0; n < MAX_CHILDREN; n++)
      bhtree_apply_singular_gravity(node->children[n], sink_source);
  }
}

static vec2 da_gravity(PhysicsEntity *p_i, PhysicsEntity *p_j) {
  vec2 r    = vec2sub(p_j->q, p_i->q);
  vec2 rhat = vec2scale(1 / vec2mag(r), r);
  double r2 = vec2dot(r, r);
  if (r2 <= 0) return (vec2){0,0};
  return vec2scale(5000 * p_j->m * (1 / r2), rhat);
}

static void quadrant_gravity(PhysicsEntity *body, BHNode *node) {
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *cobody = node->bodies[n];
    if (body == cobody) continue;
    if (cobody) {
      vec2 da = da_gravity(body, cobody);
      body->d2q_dt2 = vec2add(body->d2q_dt2, da);
    }
  }
}

void bhtree_apply_pairwise_gravity(PhysicsEntity *body, BHNode *node) {
  if (!node) return;
  quadrant_gravity(body, node);
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_pairwise_gravity(body, node->children[n]);
  }
}

void bhtree_apply_pairwise_collisions(PhysicsEntity *body, BHNode *node) {
  if (!node) return;
  for (size_t n = 0; n < NUM_QUADS; n++) {
    PhysicsEntity *subbody = node->bodies[n];
    if (subbody) force_pairwise_impulsive_collision(body, subbody);
  }
  for (size_t n = 0; n < MAX_CHILDREN; n++) {
    bhtree_apply_pairwise_collisions(body, node->children[n]);
  }
}

void bhtree_apply_collisions(BHNode *root, PhysicsEntity *body, GLuint color) {
  if (!root) return;
  for (int i = 0; i < NUM_QUADS; i++) {
    body = root->bodies[i];
    if (!body) continue;
    body->color = color;
  }
  bhtree_apply_collisions(root->children[3], body, color);
}

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

static void node_partition(MemoryArena *arena, BHNode *node) {
  EACH_QUAD(node->min, node->max, {
      node->children[i + 2 * j] = bhtree_create(arena, __qmin, __qmax);
  });
  node->is_partitioned = true;
}

void quad_state_log(OccState state) {
  printf("QUAD STATE: { ");
  if ((state & OCC_NW) >> 0) printf("NW, ");
  if ((state & OCC_NE) >> 1) printf("NE, ");
  if ((state & OCC_SW) >> 2) printf("SW, ");
  if ((state & OCC_SE) >> 3) printf("SE, ");
  printf("}\n");
}

#endif
