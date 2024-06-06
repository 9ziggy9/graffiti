#include "primitives.h"
#include "config.h"
#include "shader.h"
#include <math.h>

#define COLOR_NORM(HEX)                     \
    ((float)((HEX >> 24) & 0xFF) / 255.0f), \
    ((float)((HEX >> 16) & 0xFF) / 255.0f), \
    ((float)((HEX >> 8)  & 0xFF) / 255.0f), \
    ((float)((HEX >> 0)  & 0xFF) / 255.0f)
 
void draw_eqtriangle(vec2 pos,
                     GLfloat size,
                     GLfloat theta,
                     GLuint color_hex1,
                     GLuint color_hex2,
                     GLuint color_hex3)
{
  GLfloat vertices[] = {
    -size, -size, 0.0f,
     size, -size, 0.0f,
     0.0f,  size, 0.0f,
  };

  GLfloat colors[] = {
    COLOR_NORM(color_hex1),
    COLOR_NORM(color_hex2),
    COLOR_NORM(color_hex3),
  };

  GLuint vbo, vao;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glBindVertexArray(vao);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices) + sizeof(colors),
               NULL, GL_STATIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
  glBufferSubData(GL_ARRAY_BUFFER, sizeof(vertices), sizeof(colors), colors);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                        3 * sizeof(GLfloat), (void*)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                        (void*)sizeof(vertices));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

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

  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}
