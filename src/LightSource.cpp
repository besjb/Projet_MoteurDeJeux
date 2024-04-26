#include "LightSource.hpp"

LightSource::LightSource(
    glm::vec3 lightPosition,
    glm::vec3 lightColor
) :
    lightPosition{lightPosition},
    lightColor{lightColor}
{}

glm::vec3 LightSource::getLightPosition() const {
    return lightPosition;
}

glm::vec3 LightSource::getLightColor() const {
    return lightColor;
}

LightSource& LightSource::setLightPosition(const glm::vec3& lightPosition) {
    this->lightPosition = lightPosition;
    return *this;
}

LightSource& LightSource::setLightColor(const glm::vec3& lightColor) {
    this->lightColor = lightColor;
    return *this;
}