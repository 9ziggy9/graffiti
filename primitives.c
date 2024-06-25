#include <math.h>
#include <stdbool.h>

#include "primitives.h"
#include "config.h"
#include "shader.h"
#include "alloc.h"

typedef enum {
  ATTR_POS = 1, // 0b0001
  ATTR_CLR = 2 //  0b0010
} attr_flag;

static bool PRIMITIVES_ENABLED = false;
static GLuint VBO = 0, VAO = 0;

static void BUFFER_CLEAR(void) {
  if (VAO != 0) { glDeleteVertexArrays(1, &VAO); VAO = 0; }
  if (VBO != 0) { glDeleteVertexArrays(1, &VBO); VBO = 0; }
}

static void BUFFER_DEFINE_VERTEX_ATTRIBUTES(attr_flag attrs) {
  if (attrs & ATTR_POS) {
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(struct vertex),
                          (void *)offsetof(struct vertex, pos));
    glEnableVertexAttribArray(0);
  }
  if (attrs & ATTR_CLR) {
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(struct vertex),
                          (void *)offsetof(struct vertex, color));
    glEnableVertexAttribArray(1);
  }
}

static void BUFFER_BIND(void) {
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
}

static void BUFFER_UNBIND(void) {
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

static void BUFFER_DRAW(GLenum prim, GLint fst, GLsizei lst) {
  glBindVertexArray(VAO);
  glDrawArrays(prim, fst, lst);
  glBindVertexArray(0);
}

void ENABLE_PRIMITIVES(void) {
  if (PRIMITIVES_ENABLED) PANIC_WITH(PRIMITIVES_ALREADY_ENABLED);
  HW_REGISTER(ID_GL_VAO_PTR, &VAO);
  HW_REGISTER(ID_GL_VBO_PTR, &VBO);
  PRIMITIVES_ENABLED = true;
  INFO_LOG("primitive drawable shapes enabled");
}

void draw_eqtriangle(vec2 pos,
                     GLfloat size,
                     GLfloat theta,
                     GLuint color_hex1,
                     GLuint color_hex2,
                     GLuint color_hex3)
{
  BUFFER_CLEAR();
  struct vertex vertices[] = {
    {{-size, -size, 0.0f}, COLOR_NORM(color_hex1)},
    {{size, -size, 0.0f},  COLOR_NORM(color_hex2)},
    {{0.0f,  size, 0.0f},  COLOR_NORM(color_hex3)},
  };

  BUFFER_BIND();
    BUFFER_DEFINE_VERTEX_ATTRIBUTES(ATTR_POS | ATTR_CLR);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  BUFFER_UNBIND();

  mat4 model = ID_MAT4;
  static GLfloat ASPECT = (GLfloat) WIN_W / WIN_H;
  GLfloat cos_theta = (GLfloat) cos(theta);
  GLfloat sin_theta = (GLfloat) sin(theta);
  model[0][0] = (GLfloat)cos_theta / ASPECT;
  model[0][1] = -sin_theta;
  model[1][0] = (GLfloat)sin_theta / ASPECT;
  model[1][1] = cos_theta;
  model[3][0] = (GLfloat)pos.x / WIN_W * 2 - 1;
  model[3][1] = (GLfloat)pos.y / WIN_H * 2 - 1;

  GLint model_uni_loc = glGetUniformLocation(SHADER(), "model");
  glUniformMatrix4fv(model_uni_loc, 1, GL_FALSE, (GLfloat*) model);

  BUFFER_DRAW(GL_TRIANGLES, 0, 3);
}

void draw_circle(vec2 pos, GLfloat radius, GLuint color_hex) {
  const int num_segments = 100;
  const float color[] = COLOR_NORM(color_hex);

  BUFFER_CLEAR();
  struct vertex vertices[num_segments + 2];
  for (int i = 0; i <= num_segments; i++) {
    double theta = 2.0f * M_PI * i / num_segments;
    vertices[i].pos[0] = (GLfloat)cos(theta);
    vertices[i].pos[1] = (GLfloat)sin(theta);
    vertices[i].pos[2] = 0.0f;
    vertices[i].color[0] = color[0];
    vertices[i].color[1] = color[1];
    vertices[i].color[2] = color[2];
    vertices[i].color[3] = color[3];
  }

  BUFFER_BIND();
    BUFFER_DEFINE_VERTEX_ATTRIBUTES(ATTR_POS | ATTR_CLR);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(vertices),
                vertices, GL_STATIC_DRAW);
  BUFFER_UNBIND();

  mat4 model = ID_MAT4;
  model[0][0] = (GLfloat)radius / WIN_W * 2;
  model[1][1] = (GLfloat)radius / WIN_H * 2;
  model[3][0] = (GLfloat)pos.x / WIN_W * 2 - 1;
  model[3][1] = (GLfloat)pos.y / WIN_H * 2 - 1;

  GLint model_uni_loc = glGetUniformLocation(SHADER(), "model");
  glUniformMatrix4fv(model_uni_loc, 1, GL_FALSE, (GLfloat*) model);

  BUFFER_DRAW(GL_TRIANGLE_FAN, 0, num_segments + 2);
}

void draw_circle_boundary(vec2 pos, GLfloat radius, GLuint color_hex) {
  const int num_segments = 100;
  BUFFER_CLEAR();
  struct vertex vertices[num_segments];
  float color[] = COLOR_NORM(color_hex);
  for (int i = 0; i < num_segments; i++) {
    double theta = 2.0f * M_PI * i / num_segments;
    vertices[i].pos[0] = (GLfloat)cos(theta);
    vertices[i].pos[1] = (GLfloat)sin(theta);
    vertices[i].pos[2] = 0.0;
    vertices[i].color[0] = color[0];
    vertices[i].color[1] = color[1];
    vertices[i].color[2] = color[2];
    vertices[i].color[3] = color[3];
  }

  BUFFER_BIND();
    BUFFER_DEFINE_VERTEX_ATTRIBUTES(ATTR_POS | ATTR_CLR);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(vertices),
                vertices, GL_STATIC_DRAW);
  BUFFER_UNBIND();

  mat4 model = ID_MAT4;
  model[0][0] = radius / WIN_W * 2;
  model[1][1] = radius / WIN_H * 2;
  model[3][0] = (GLfloat)pos.x / WIN_W * 2 - 1;
  model[3][1] = (GLfloat)pos.y / WIN_H * 2 - 1;

  GLint model_uni_loc = glGetUniformLocation(SHADER(), "model");
  glUniformMatrix4fv(model_uni_loc, 1, GL_FALSE, (GLfloat*) model);

  BUFFER_DRAW(GL_LINE_LOOP, 0, num_segments);
}
