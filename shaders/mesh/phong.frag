#version 330 core

in vec3 position;
in vec3 normal;
in vec2 texCoords;

out vec4 color;

uniform vec3 materialAmbient;
uniform vec3 materialDiffuse;
uniform vec3 materialSpecular;
uniform float materialShininess;

uniform vec3 lightPosition;
uniform vec3 lightColor;

uniform vec3 cameraPosition;

uniform sampler2D meshTexture;

void main() {

    // Phong

    // Ambient
    vec3 ambient = materialAmbient;

    // Diffuse
    vec3 lightDir = normalize(lightPosition - position);
    vec3 diffuse = max(dot(normal, lightDir), 0.0) * materialDiffuse;
    
    // Specular
    vec3 viewDir = normalize(cameraPosition - position);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), materialShininess);
    vec3 specular = spec * materialSpecular;

    color = vec4(lightColor * (ambient + diffuse + specular) * texture(meshTexture, texCoords).rgb, texture(meshTexture, texCoords).a);
}