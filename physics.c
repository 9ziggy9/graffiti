#include "config.h"
#include "physics.h"

PhysicsEntity
new_physics_entity(vec2 q0, vec2 dq0_dt, vec2 d2q0_dt2, double m, GLuint clr) {
  return (PhysicsEntity){ q0, dq0_dt, d2q0_dt2, m, clr, GEOM_NONE, {
      .none = NULL,
  }};
}

void
physics_entity_bind_geometry(PhysicsEntity *entity, geometry_t type, Geometry g)
{
  entity->geom_t = type;
  entity->geom   = g;
}

void physics_apply_pairwise_gravity(PhysicsEntity *ps, int num_ps) {
  static const double G = CONST_GRAVITY;
  for (int i = 0; i < num_ps; i++) {
    for (int j = i + 1; j < num_ps; j++) {
      PhysicsEntity *ci = &ps[i];
      PhysicsEntity *cj = &ps[j];
      vec2 rvec = vec2sub(cj->q, ci->q);
      double r2 = vec2dot(rvec, rvec);
      vec2 F_ij = vec2scale(G * ci->m * cj->m * (1.0f / r2), rvec);
      ci->d2q_dt2 = vec2add(ci->d2q_dt2, vec2scale(1.0f / ci->m, F_ij));
      cj->d2q_dt2 = vec2add(cj->d2q_dt2, vec2scale(-1.0f / cj->m, F_ij));
    }
  }
}

// TODO: unhardcode circular geometry
void physics_apply_boundaries(PhysicsEntity *ps, int num_ps) {
  for (int i = 0; i < num_ps; i++) {
    PhysicsEntity *p = &ps[i];
    if (p->q.x - p->geom.circ.R <= 0.0) {
      p->dq_dt.x *= -1;
      p->q.x = p->geom.circ.R;
    } else if (p->q.x + p->geom.circ.R >= WIN_W) {
      p->dq_dt.x *= -1;
      p->q.x = WIN_W - p->geom.circ.R;
    }
    if (p->q.y - p->geom.circ.R <= 0.0) {
      p->dq_dt.y *= -1;
      p->q.y = p->geom.circ.R;
    } else if (p->q.y + p->geom.circ.R >= WIN_H) {
      p->dq_dt.y *= -1;
      p->q.y = WIN_H - p->geom.circ.R;
    }
  }
}

static void physics_resolve_collision(PhysicsEntity *c1,
                                      PhysicsEntity *c2,
                                      vec2 diff,
                                      double overlap,
                                      double e // coefficient of restitution
                                      )
{
  vec2 n = vec2norm(diff);
  vec2 v1 = c1->dq_dt;
  vec2 v2 = c2->dq_dt;

  double v_rel = vec2dot(vec2sub(v2, v1), n);
  if (v_rel < 0.0f) {
    double j = (1.0f + e) * v_rel / (1.0f / c1->m + 1.0f / c2->m);
    vec2 impulse = vec2scale(j, n);
    c1->dq_dt = vec2add(c1->dq_dt, vec2scale(1.0f / c1->m, impulse));
    c2->dq_dt = vec2sub(c2->dq_dt, vec2scale(1.0f / c2->m, impulse));
  }

  double corr = overlap / (c1->m + c2->m);
  c1->q = vec2add(c1->q, vec2scale(-corr * c1->m, n));
  c2->q = vec2add(c2->q, vec2scale(corr * c2->m, n));
}

// TODO: unhardcode circular geometry
void physics_apply_collision(PhysicsEntity *ps, int num_ps) {
  for (int i = 0; i < num_ps; i++) {
    for (int j = i + 1; j < num_ps; j++) {
      vec2 diff = (vec2) { ps[j].q.x - ps[i].q.x, ps[j].q.y - ps[i].q.y };
      double overlap = ps[i].geom.circ.R + ps[j].geom.circ.R - vec2mag(diff);
      if (overlap <= 0.0f) continue;
      physics_resolve_collision(&ps[i], &ps[j], diff, overlap, 0.33f);
    }
  }
}

double physics_compute_kinetic_energy(PhysicsEntity *circs, int num_circs) {
  double total_ke = 0.0f;
  for (int i = 0; i < num_circs; i++) {
    double v2 = vec2dot(circs[i].dq_dt, circs[i].dq_dt);
    total_ke += 0.5f * circs[i].m * v2;
  }
  return total_ke;
}

double physics_compute_gravitational_potential_energy(PhysicsEntity *circs,
                                                       int num_circs)
{
  static const double G = CONST_GRAVITY;
  double total_pe = 0.0f;
  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].q.x - circs[i].q.x,
                           circs[j].q.y - circs[i].q.y };
      double r = vec2mag(diff);
      if (r > 0.0f) {
        total_pe -= G * circs[i].m * circs[j].m / r;
      }
    }
  }
  return total_pe;
}
