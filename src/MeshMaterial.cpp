#include "MeshMaterial.hpp"
#include "Scene.hpp"

MeshMaterial::MeshMaterial(
    ShaderProgram* shaderProgram,
    GLuint textureLocation,
    const glm::vec3& ambientColor,
    const glm::vec3& diffuseColor,
    const glm::vec3& specularColor,
    float shininess
) :
    Material{shaderProgram, ambientColor, diffuseColor, specularColor, shininess},
    cameraPositionUniform{*shaderProgram, "cameraPosition"},
    modelTransformUniform{*shaderProgram, "modelTransform"},
    transformUniform{*shaderProgram, "transform"},
    textureUniform{*shaderProgram, "meshTexture"},
    textureLocation{textureLocation}
{}

GLuint MeshMaterial::getTextureLocation() const {
    return textureLocation;
}

MeshMaterial& MeshMaterial::setTextureLocation(GLuint textureLocation) {
    this->textureLocation = textureLocation;
    return *this;
}

void MeshMaterial::setUniforms(const Scene& scene, const glm::mat4& transform) {
    Material::setUniforms(scene, transform);
    const Camera& camera{scene.getCamera()};
    cameraPositionUniform.set(camera.getPosition());
    modelTransformUniform.set(transform);
    transformUniform.set(camera.getViewProjectionMatrix() * transform);
    textureUniform.set(0);
}