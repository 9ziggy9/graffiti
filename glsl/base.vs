#version 330 core
precision highp float;

layout (location = 0) in vec3 position;
layout (location = 1) in vec4 color;

out vec4 vertexColor;

uniform mat4 model;

void main() {
    gl_Position = model * vec4(position, 1.0);
    vertexColor = color;
}
