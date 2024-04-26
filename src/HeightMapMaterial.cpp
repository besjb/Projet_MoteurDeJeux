#include "HeightMapMaterial.hpp"
#include "Scene.hpp"

HeightMapMaterial::HeightMapMaterial(
    ShaderProgram* shaderProgram,
    GLuint grassTextureLocation,
    GLuint rockTextureLocation,
    GLuint snowRocksTextureLocation,
    const glm::vec3& ambientColor,
    const glm::vec3& diffuseColor,
    const glm::vec3& specularColor,
    float shininess
) :
    Material{shaderProgram, ambientColor, diffuseColor, specularColor, shininess},
    cameraPositionUniform{*shaderProgram, "cameraPosition"},
    modelTransformUniform{*shaderProgram, "modelTransform"},
    transformUniform{*shaderProgram, "transform"},
    grassTextureUniform{*shaderProgram, "grassTexture"},
    rockTextureUniform{*shaderProgram, "rockTexture"},
    snowRocksTextureUniform{*shaderProgram, "snowRocksTexture"},
    grassTextureLocation{grassTextureLocation},
    rockTextureLocation{rockTextureLocation},
    snowRocksTextureLocation{snowRocksTextureLocation}
{}

GLuint HeightMapMaterial::getGrassTextureLocation() const {
    return grassTextureLocation;
}

GLuint HeightMapMaterial::getRockTextureLocation() const {
    return rockTextureLocation;
}

GLuint HeightMapMaterial::getSnowRocksTextureLocation() const {
    return snowRocksTextureLocation;
}

HeightMapMaterial& HeightMapMaterial::setGrassTextureLocation(GLuint grassTextureLocation) {
    this->grassTextureLocation = grassTextureLocation;
    return *this;
}

HeightMapMaterial& HeightMapMaterial::setRockTextureLocation(GLuint rockTextureLocation) {
    this->rockTextureLocation = rockTextureLocation;
    return *this;
}

HeightMapMaterial& HeightMapMaterial::setSnowRocksTextureLocation(GLuint snowRocksTextureLocation) {
    this->snowRocksTextureLocation = snowRocksTextureLocation;
    return *this;
}

void HeightMapMaterial::setUniforms(const Scene& scene, const glm::mat4& transform) {
    Material::setUniforms(scene, transform);
    const Camera& camera{scene.getCamera()};
    cameraPositionUniform.set(camera.getPosition());
    modelTransformUniform.set(transform);
    transformUniform.set(camera.getViewProjectionMatrix() * transform);
    grassTextureUniform.set(0);
    rockTextureUniform.set(1);
    snowRocksTextureUniform.set(2);
}