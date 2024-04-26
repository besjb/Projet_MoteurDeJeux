#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>

class Transform {
public:

    Transform(const glm::vec3& translation = {}, const glm::quat& rotation = glm::identity<glm::quat>(), float scale = 1.0f);

    Transform(const Transform& transform);

    Transform& scale(float scale);

    Transform& rotate(const glm::quat& rotation);

    Transform& translate(const glm::vec3& translation);

    Transform& setScale(float scale);

    Transform& setRotation(const glm::quat& rotation);

    Transform& setTranslation(const glm::vec3& translation);

    Transform inverse() const;

    Transform interpolate(const Transform& transform, float t) const;

    Transform multiply(const Transform& transform) const;

    glm::vec3 getTranslation() const;

    glm::quat getRotation() const;

    float getScale() const;

    glm::mat4 toMat4() const;

private:

    glm::vec3 translation;
    glm::quat rotation;
    float scaleFactor;

};