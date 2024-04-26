#pragma once

#include "ShaderProgram.hpp"
#include "Uniform.hpp"

#include <glm/glm.hpp>

class Scene;

class Material {
public:

    Material(
        ShaderProgram* shaderProgram,
        const glm::vec3& ambientColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& diffuseColor = glm::vec3{1.0f, 1.0f, 1.0f},
        const glm::vec3& specularColor = glm::vec3{1.0f, 1.0f, 1.0f},
        float shininess = 1.0f
    );

    glm::vec3 getAmbientColor() const;

    glm::vec3 getDiffuseColor() const;

    glm::vec3 getSpecularColor() const;

    float getShininess() const;

    ShaderProgram* getShaderProgram() const;

    Material& setAmbientColor(const glm::vec3& ambientColor);

    Material& setDiffuseColor(const glm::vec3& diffuseColor);

    Material& setSpecularColor(const glm::vec3& specularColor);

    Material& setShininess(float shininess);

    Material& setShaderProgram(ShaderProgram* shaderProgram);

    virtual void setUniforms(const Scene& scene, const glm::mat4& transform);

protected:

    glm::vec3 ambientColor;
    glm::vec3 diffuseColor;
    glm::vec3 specularColor;
    float shininess;

    Uniform ambientColorUniform;
    Uniform diffuseColorUniform;
    Uniform specularColorUniform;
    Uniform shininessUniform;
    Uniform lightPositionUniform;
    Uniform lightColorUniform;

    ShaderProgram* shaderProgram;

};