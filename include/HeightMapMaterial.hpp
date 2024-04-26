#pragma once

#include "Material.hpp"
#include "Uniform.hpp"

#include <GL/glew.h>

class Scene;

class HeightMapMaterial : public Material {
public:

    HeightMapMaterial(
        ShaderProgram* shaderProgram,
        GLuint grassTextureLocation,
        GLuint rockTextureLocation,
        GLuint snowRocksTextureLocation,
        const glm::vec3& ambientColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& diffuseColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& specularColor = glm::vec3{1.0f, 1.0f, 1.0f},
        float shininess = 0.0f
    );

    GLuint getGrassTextureLocation() const;

    GLuint getRockTextureLocation() const;

    GLuint getSnowRocksTextureLocation() const;

    HeightMapMaterial& setGrassTextureLocation(GLuint grassTextureLocation);

    HeightMapMaterial& setRockTextureLocation(GLuint rockTextureLocation);

    HeightMapMaterial& setSnowRocksTextureLocation(GLuint snowRocksTextureLocation);

    void setUniforms(const Scene& scene, const glm::mat4& transform) override;

private:

    Uniform cameraPositionUniform;
    Uniform modelTransformUniform;
    Uniform transformUniform;
    Uniform grassTextureUniform;
    Uniform rockTextureUniform;
    Uniform snowRocksTextureUniform;

    GLuint grassTextureLocation;
    GLuint rockTextureLocation;
    GLuint snowRocksTextureLocation;

};