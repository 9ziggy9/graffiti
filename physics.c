#include "config.h"
#include "physics.h"

void physics_apply_pairwise_gravity(KinematicCircle *circs, int num_circs) {
  static const double G = CONST_GRAVITY;
  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      KinematicCircle *ci = &circs[i];
      KinematicCircle *cj = &circs[j];
      vec2 rvec = vec2sub(cj->p, ci->p);
      double r2 = vec2dot(rvec, rvec);
      vec2 F_ij = vec2scale(G * ci->m * cj->m * (1.0f / r2), rvec);
      ci->d2p_dt2 = vec2add(ci->d2p_dt2, vec2scale(1.0f / ci->m, F_ij));
      cj->d2p_dt2 = vec2add(cj->d2p_dt2, vec2scale(-1.0f / cj->m, F_ij));
    }
  }
}

void physics_apply_boundaries(KinematicCircle *circs, int num_circs) {
  for (int i = 0; i < num_circs; i++) {
    KinematicCircle *circ = &circs[i];
    if (circ->p.x - circ->rad <= 0.0) {
      circ->dp_dt.x *= -1;
      circ->p.x = circ->rad;
    } else if (circ->p.x + circ->rad >= WIN_W) {
      circ->dp_dt.x *= -1;
      circ->p.x = WIN_W - circ->rad;
    }
    if (circ->p.y - circ->rad <= 0.0) {
      circ->dp_dt.y *= -1;
      circ->p.y = circ->rad;
    } else if (circ->p.y + circ->rad >= WIN_H) {
      circ->dp_dt.y *= -1;
      circ->p.y = WIN_H - circ->rad;
    }
  }
}

static void physics_resolve_collision(KinematicCircle *c1,
                                      KinematicCircle *c2,
                                      vec2 diff,
                                      double overlap,
                                      double e // coefficient of restitution
                                      )
{
  vec2 n = vec2norm(diff);
  vec2 v1 = c1->dp_dt;
  vec2 v2 = c2->dp_dt;

  double v_rel = vec2dot(vec2sub(v2, v1), n);
  if (v_rel < 0.0f) {
    double j = (1.0f + e) * v_rel / (1.0f / c1->m + 1.0f / c2->m);
    vec2 impulse = vec2scale(j, n);
    c1->dp_dt = vec2add(c1->dp_dt, vec2scale(1.0f / c1->m, impulse));
    c2->dp_dt = vec2sub(c2->dp_dt, vec2scale(1.0f / c2->m, impulse));
  }

  double corr = overlap / (c1->m + c2->m);
  c1->p = vec2add(c1->p, vec2scale(-corr * c1->m, n));
  c2->p = vec2add(c2->p, vec2scale(corr * c2->m, n));
}

void physics_apply_collision(KinematicCircle *circs, int num_circs) {
  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].p.x - circs[i].p.x,
                           circs[j].p.y - circs[i].p.y };
      double overlap = circs[i].rad + circs[j].rad - vec2mag(diff);
      if (overlap > 0.0f) physics_resolve_collision(&circs[i], &circs[j],
                                                    diff, overlap, 0.33f);
    }
  }
}

double physics_compute_kinetic_energy(KinematicCircle *circs, int num_circs) {
  double total_ke = 0.0f;
  for (int i = 0; i < num_circs; i++) {
    double v2 = vec2dot(circs[i].dp_dt, circs[i].dp_dt);
    total_ke += 0.5f * circs[i].m * v2;
  }
  return total_ke;
}

double physics_compute_gravitational_potential_energy(KinematicCircle *circs,
                                                       int num_circs)
{
  static const double G = CONST_GRAVITY;
  double total_pe = 0.0f;
  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].p.x - circs[i].p.x,
                           circs[j].p.y - circs[i].p.y };
      double r = vec2mag(diff);
      if (r > 0.0f) {
        total_pe -= G * circs[i].m * circs[j].m / r;
      }
    }
  }
  return total_pe;
}


#if 0 // DEPRECATED
void physics_apply_naive_collisions(KinematicCircle *circ,
                                    KinematicCircle *circs,
                                    int num_circs)
{
  for (int i = 0; i < num_circs; i++) {
    if (circ == &circs[i]) continue;

    vec2 diff = (vec2) { circs[i].p.x - circ->p.x, circs[i].p.y - circ->p.y };

    if (vec2mag(diff) < circ->rad + circs[i].rad) {
      vec2 n = vec2norm(diff);
      vec2 t = (vec2) { -n.y, n.x };

      vec2 v1 = (vec2) {vec2dot(circ->dp_dt, n), vec2dot(circ->dp_dt, t)};
      vec2 v2 = (vec2) {vec2dot(circs[i].dp_dt, n), vec2dot(circs[i].dp_dt, t)};

      double v1_n = (v1.x * (circ->m - circs[i].m) + 2 * circs[i].m * v2.x)
                      / (circ->m + circs[i].m);
      GLfloat v2_n = (v2.x * (circs[i].m - circ->m) + 2 * circ->m * v1.x)
                      / (circ->m + circs[i].m);

      circ->dp_dt    = vec2add(vec2scale(v1_n, n), vec2scale(v1.y, t));
      circs[i].dp_dt = vec2add(vec2scale(v2_n, n), vec2scale(v2.y, t));
    }
  }
}

void physics_apply_collision_simple_impulse(KinematicCircle *circs,
                                            int num_circs)
{
  vec2 impulses[num_circs];

  for (int i = 0; i < num_circs; i++) impulses[i] = (vec2) { 0.0f, 0.0f };

  for (int i = 0; i < num_circs; i++) {
    for (int j = i + 1; j < num_circs; j++) {
      vec2 diff = (vec2) { circs[j].p.x - circs[i].p.x,
                           circs[j].p.y - circs[i].p.y };

      GLfloat dist = vec2mag(diff);
      GLfloat overlap = circs[i].rad + circs[j].rad - dist;

      if (overlap > 0.0f) {
        vec2 n = vec2norm(diff);
        vec2 t = (vec2) { -n.y, n.x };

        vec2 v1 = (vec2) {vec2dot(circs[i].dp_dt, n),
                          vec2dot(circs[i].dp_dt, t)};
        vec2 v2 = (vec2) {vec2dot(circs[j].dp_dt, n),
                          vec2dot(circs[j].dp_dt, t)};

        GLfloat v1_n = (v1.x * (circs[i].m - circs[j].m)
                        + 2 * circs[j].m * v2.x) / (circs[i].m + circs[j].m);
        GLfloat v2_n = (v2.x * (circs[j].m - circs[i].m)
                        + 2 * circs[i].m * v1.x) / (circs[i].m + circs[j].m);

        vec2 impulse_i = vec2add(vec2scale(v1_n, n), vec2scale(v1.y, t));
        vec2 impulse_j = vec2add(vec2scale(v2_n, n), vec2scale(v2.y, t));

        impulse_i = vec2sub(impulse_i, circs[i].dp_dt);
        impulse_j = vec2sub(impulse_j, circs[j].dp_dt);

        impulses[i] = vec2add(impulses[i], impulse_i);
        impulses[j] = vec2add(impulses[j], impulse_j);

        GLfloat corr = overlap / (circs[i].m + circs[j].m);
        circs[i].p = vec2add(circs[i].p, vec2scale(-corr * circs[i].m, n));
        circs[j].p = vec2add(circs[j].p, vec2scale(corr * circs[j].m, n));
      }
    }
  }
  for (int i = 0; i < num_circs; i++) circs[i].dp_dt = vec2add(circs[i].dp_dt,
                                                               impulses[i]);
}
#endif
