#pragma once

#include <glm/glm.hpp>

class LightSource {
public:

    LightSource(
        glm::vec3 lightPosition = {},
        glm::vec3 lightColor = {1.0f, 1.0f, 1.0f}
    );

    glm::vec3 getLightPosition() const;

    glm::vec3 getLightColor() const;

    LightSource& setLightPosition(const glm::vec3& lightPosition);

    LightSource& setLightColor(const glm::vec3& lightColor);

private:

    glm::vec3 lightPosition;
    glm::vec3 lightColor;

};