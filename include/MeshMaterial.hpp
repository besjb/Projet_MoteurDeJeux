#pragma once

#include "Material.hpp"
#include "Uniform.hpp"

#include <GL/glew.h>

class Scene;

class MeshMaterial : public Material {
public:

    MeshMaterial(
        ShaderProgram* shaderProgram,
        GLuint textureLocation,
        const glm::vec3& ambientColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& diffuseColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& specularColor = glm::vec3{1.0f, 1.0f, 1.0f},
        float shininess = 0.0f
    );

    GLuint getTextureLocation() const;

    MeshMaterial& setTextureLocation(GLuint textureLocation);

    void setUniforms(const Scene& scene, const glm::mat4& transform) override;

private:

    Uniform cameraPositionUniform;
    Uniform modelTransformUniform;
    Uniform transformUniform;
    Uniform textureUniform;

    GLuint textureLocation;

};