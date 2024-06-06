#include "primitives.h"

#define COLOR(HEX)                          \
    ((float)((HEX >> 24) & 0xFF) / 255.0f), \
    ((float)((HEX >> 16) & 0xFF) / 255.0f), \
    ((float)((HEX >> 8)  & 0xFF) / 255.0f), \
    ((float)((HEX >> 0)  & 0xFF) / 255.0f)

#define VERTEX(X, Y, Z) X, Y, Z

void draw_triangle(GLfloat size, GLuint color_hex) {
  GLfloat vertices[] = {
    -size, -size, 0.0f,
     size, -size, 0.0f,
     0.0f,  size, 0.0f,
  };

  GLfloat colors[] = {
    COLOR(color_hex),
    COLOR(color_hex),
    COLOR(color_hex),
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

  glBindVertexArray(vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);

  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void draw_tricolor(GLfloat size,
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
    COLOR(color_hex1),
    COLOR(color_hex2),
    COLOR(color_hex3),
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

  glBindVertexArray(vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);

  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}
