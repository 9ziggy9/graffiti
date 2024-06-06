#include "primitives.h"
#include "config.h"
#include "shader.h"
#include <math.h>

void draw_eqtriangle(vec2 pos,
                     GLfloat size,
                     GLfloat theta,
                     GLuint color_hex1,
                     GLuint color_hex2,
                     GLuint color_hex3)
{
  static GLuint vbo = 0, vao = 0;

  if (vao == 0) {
    struct vertex vertices[] = {
      {{-size, -size, 0.0f}, COLOR_NORM(color_hex1)},
      {{size, -size, 0.0f},  COLOR_NORM(color_hex2)},
      {{0.0f,  size, 0.0f},  COLOR_NORM(color_hex3)},
    };
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(struct vertex),
                          (void *)offsetof(struct vertex, pos));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(struct vertex),
                          (void *)offsetof(struct vertex, color));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  mat4 model = ID_MAT4;
  GLfloat cos_theta = (GLfloat) cos(theta);
  GLfloat sin_theta = (GLfloat) sin(theta);
  model[0][0] = cos_theta;
  model[0][1] = -sin_theta;
  model[1][0] = sin_theta;
  model[1][1] = cos_theta;
  model[3][0] = pos.x / WIN_W * 2 - 1;
  model[3][1] = pos.y / WIN_H * 2 - 1;

  GLint modelUniformLocation = glGetUniformLocation(SHADER(), "model");
  glUniformMatrix4fv(modelUniformLocation, 1, GL_FALSE, (GLfloat*) model);

  glBindVertexArray(vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);

  // TODO: these should be cleaned up somewhere--note that I am caching these
  // via the static declaration so at to save on resources.
  /* glDeleteVertexArrays(1, &vao); */
  /* glDeleteBuffers(1, &vbo); */
}
