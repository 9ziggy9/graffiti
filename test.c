#include <stdio.h>
#include <stdbool.h>

typedef enum {
  OCC_0  = 0, // 0b0000
  OCC_NW = 1, // 0b0001
  OCC_NE = 2, // 0b0010
  OCC_SW = 4, // 0b0100
  OCC_SE = 8, // 0b1000
} OccState;

/* static void q_is_occupied(OccState state, OccState quad) { */
/*   if (state & quad) { */
/*     printf("Quad flag occupied %d\n", quad); */
/*     return; */
/*   } */
/*   printf("Quad empty.\n"); */
/* } */

static size_t q_occ_count(OccState state) {
  return ((state & OCC_SE) >> 3)
       + ((state & OCC_SW) >> 2)
       + ((state & OCC_NE) >> 1)
       + ((state & OCC_NW) >> 0);
}

int main(void) {
  OccState s = OCC_0;
  s |= OCC_NW | OCC_NE | OCC_SE | OCC_SW;
  printf("TOTAL BODIES: %zu\n", q_occ_count(s));
  return 0;
}
