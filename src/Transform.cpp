#include "Transform.hpp"

Transform::Transform(const glm::vec3& translation, const glm::quat& rotation, float scale) :
    translation{translation},
    rotation{rotation},
    scaleFactor{scale}
{}

Transform::Transform(const Transform& transform) :
    translation{transform.translation},
    rotation{transform.rotation},
    scaleFactor{transform.scaleFactor}
{}

Transform& Transform::translate(const glm::vec3& translation) {
    this->translation += translation;
    return *this;
}

Transform& Transform::rotate(const glm::quat& rotation) {
    this->rotation *= rotation;
    translation = rotation * translation;
    return *this;
}

Transform& Transform::scale(float scale) {
    translation *= scale;
    scaleFactor *= scale;
    return *this;
}

Transform& Transform::setScale(float scale) {
    scaleFactor = scale;
    return *this;
}

Transform& Transform::setRotation(const glm::quat& rotation) {
    this->rotation = rotation;
    return *this;
}

Transform& Transform::setTranslation(const glm::vec3& translation) {
    this->translation = translation;
    return *this;
}

Transform Transform::inverse() const {
    return {
        -translation,
        glm::conjugate(rotation),
        1.0f / scaleFactor
    };
}

Transform Transform::interpolate(const Transform& transform, float t) const {
    return {
        glm::mix(translation, transform.translation, t),
        glm::slerp(rotation, transform.rotation, t),
        scaleFactor * t + (1.0f - t) * transform.scaleFactor
    };
}

Transform Transform::multiply(const Transform& transform) const {
    return {
        translation + scaleFactor * (rotation * transform.translation),
        rotation * transform.rotation,
        scaleFactor * transform.scaleFactor
    };
}

glm::vec3 Transform::getTranslation() const {
    return translation;
}

glm::quat Transform::getRotation() const {
    return rotation;
}

float Transform::getScale() const {
    return scaleFactor;
}

glm::mat4 Transform::toMat4() const {
    glm::mat4 translate{glm::translate(glm::mat4{1.0}, translation)};
    glm::mat4 rotate{glm::mat4_cast(rotation)};
    glm::mat4 scale{glm::scale(glm::mat4{1.0}, glm::vec3{scaleFactor, scaleFactor, scaleFactor})};

    return translate * rotate * scale;
}