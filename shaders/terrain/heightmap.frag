#version 330 core

in vec3 position;
in vec3 normal;
in vec2 texCoords;
in float altitude;

out vec3 color;

uniform vec3 materialAmbient;
uniform vec3 materialDiffuse;
uniform vec3 materialSpecular;
uniform float materialShininess;

uniform vec3 lightPosition;
uniform vec3 lightColor;

uniform vec3 cameraPosition;

uniform sampler2D grassTexture;
uniform sampler2D rockTexture;
uniform sampler2D snowRocksTexture;

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
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), materialShininess) ;
    vec3 specular = spec * materialSpecular;

    float grassWeight = max(0.0, 1.0 - 2.0 * altitude);
    float snowRocksWeight = max(0.0, 2.0 * altitude - 1.0);
    float rockWeight = max(0.0, 1.0 - grassWeight - snowRocksWeight);

    vec4 grassColor = texture(grassTexture, 4.0 * texCoords) * grassWeight;
    vec4 rockColor = texture(rockTexture, 4.0 * texCoords) * rockWeight;
    vec4 snowRocksColor = texture(snowRocksTexture, 4.0 * texCoords) * snowRocksWeight;

    color = lightColor * (ambient + diffuse + specular) * (grassColor + rockColor + snowRocksColor).rgb;
}