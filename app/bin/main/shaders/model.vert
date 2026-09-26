#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;

out vec3 fragNormal;
out vec3 fragPos;

void main() {
    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
    fragPos = worldPosition.xyz;
    
    mat3 normalMatrix = transpose(inverse(mat3(modelMatrix)));
    fragNormal = normalMatrix * normal;
    
    gl_Position = projectionMatrix * viewMatrix * worldPosition;
}
