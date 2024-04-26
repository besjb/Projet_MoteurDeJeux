#include "Material.hpp"
#include "Uniform.hpp"
#include "Scene.hpp"
#include "LightSource.hpp"

Material::Material(
    ShaderProgram* shaderProgram,
    const glm::vec3& ambientColor,
    const glm::vec3& diffuseColor,
    const glm::vec3& specularColor,
    float shininess
) : 
    shaderProgram{shaderProgram},
    ambientColor{ambientColor},
    diffuseColor{diffuseColor},
    specularColor{specularColor},
    shininess{shininess},
    ambientColorUniform{*shaderProgram, "materialAmbient"},
    diffuseColorUniform{*shaderProgram, "materialDiffuse"},
    specularColorUniform{*shaderProgram, "materialSpecular"},
    shininessUniform{*shaderProgram, "materialShininess"},
    lightPositionUniform{*shaderProgram, "lightPosition"},
    lightColorUniform{*shaderProgram, "lightColor"}
{}

glm::vec3 Material::getAmbientColor() const {
    return ambientColor;
}

glm::vec3 Material::getDiffuseColor() const {
    return diffuseColor;
}

glm::vec3 Material::getSpecularColor() const {
    return specularColor;
}

float Material::getShininess() const {
    return shininess;
}

ShaderProgram* Material::getShaderProgram() const {
    return shaderProgram;
}

Material& Material::setAmbientColor(const glm::vec3& ambientColor) {
    this->ambientColor = ambientColor;
    return *this;
}

Material& Material::setDiffuseColor(const glm::vec3& diffuseColor) {
    this->diffuseColor = diffuseColor;
    return *this;
}

Material& Material::setSpecularColor(const glm::vec3& specularColor) {
    this->specularColor = specularColor;
    return *this;
}


Material& Material::setShininess(float shininess) {
    this->shininess = shininess;
    return *this;
}


Material& Material::setShaderProgram(ShaderProgram* shaderProgram) {
    this->shaderProgram = shaderProgram;
    return *this;
}

void Material::setUniforms(const Scene& scene, const glm::mat4& transform) {
    shaderProgram->use();
    ambientColorUniform.set(ambientColor);
    diffuseColorUniform.set(diffuseColor);
    specularColorUniform.set(specularColor);
    shininessUniform.set(shininess);

    const LightSource& lightSource{scene.getLightSource()};
    lightPositionUniform.set(lightSource.getLightPosition());
    lightColorUniform.set(lightSource.getLightColor());
}