#include "nerd.h"
#include "colors.h"

GLint get_random(int low, int high) {return (rand() % (high - low + 1)) + low;}

GLuint get_random_color(void) {
  return ((GLuint)get_random(0, 255) << 24)
       | ((GLuint)get_random(0, 255) << 16)
       | ((GLuint)get_random(0, 255) << 8)
       | 0xFF; 
}

GLuint get_random_color_from_palette(void) {
  static const GLuint color_palette[] = {
    CLR_RED    , CLR_GREEN  , CLR_BLUE   , CLR_YELLOW , CLR_MAGENTA, CLR_CYAN  ,
    CLR_ORANGE , CLR_PURPLE , CLR_FGREEN , CLR_NBLUE  , CLR_GRAY   , CLR_BROWN ,
    CLR_LGREEN , CLR_SBLUE  , CLR_PINK   , CLR_AQUA   ,
  };
  return color_palette[get_random(0, sizeof(color_palette) /
                                     sizeof(color_palette[0]) - 1)];
}
