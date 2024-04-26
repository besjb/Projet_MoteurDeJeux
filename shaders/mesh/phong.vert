#version 330 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoords;

uniform mat4 modelTransform;
uniform mat4 transform;

out vec3 position;
out vec3 normal;
out vec2 texCoords;

void main(){
    gl_Position = transform * vec4(inPosition, 1.0);
    position = (modelTransform * vec4(inPosition, 1.0)).xyz;
    normal = normalize((inverse(transpose(modelTransform)) * vec4(inNormal, 1.0)).xyz);
    texCoords = inTexCoords;
}