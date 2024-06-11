#version 330 core
precision highp float;

in vec4 vertexColor;

out vec4 fragmentColor;

void main() {
    fragmentColor = vec4(vertexColor.rgb, vertexColor.a);
}
