#include "Camera.hpp"
#include <glm/gtx/vector_angle.hpp>

#include <iostream>

Camera::Camera(
    const glm::vec3& position,
    const glm::quat& rotation,
    float fov,
    float aspectRatio,
    float nearClip,
    float farClip
) :
    position{position},
    rotation{rotation},
    fov{fov},
    aspectRatio{aspectRatio},
    nearClip{nearClip},
    farClip{farClip}
{}

void Camera::setAspectRatio(float aspectRatio) {
    this->aspectRatio = aspectRatio;
}

void Camera::setFov(float aspectRatio) {
    this->fov = aspectRatio;
}

void Camera::setPosition(const glm::vec3& position) {
    this->position = position;
}

void Camera::setPosition(float x, float y, float z) {
    this->position = {x, y, z};
}

void Camera::move(const glm::vec3& offset) {
    this->position += offset;
}

void Camera::move(float x, float y, float z) {
    this->position += glm::vec3{x, y, z};
}

void Camera::moveRelative(const glm::vec3& offset) {
    this->position += offset * this->rotation;
}

void Camera::moveRelative(float x, float y, float z) {
    this->position += glm::vec3{x, y, z} * this->rotation;
}

void Camera::setRotation(const glm::quat& rotation) {
    this->rotation = rotation;
}

void Camera::setRotation(float x, float y, float z) {
    this->rotation = glm::fquat({x, y, z});
}

void Camera::rotate(const glm::quat& rotation) {
    this->rotation = rotation * this->rotation;
}

void Camera::rotate(float x, float y, float z) {
    this->rotation = glm::fquat({x, y, z}) * this->rotation;
}

float Camera::getAspectRatio() const {
    return aspectRatio;
}

float Camera::getFov() const {
    return fov;
}

glm::vec3 Camera::getPosition() const {
    return position;
}

glm::quat Camera::getRotation() const {
    return rotation;
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::translate(glm::toMat4(rotation), -position);
}

glm::mat4 Camera::getProjectionMatrix() const {
    return glm::perspective(glm::radians(fov), aspectRatio, nearClip, farClip);
}

glm::mat4 Camera::getViewProjectionMatrix() const {
    return getProjectionMatrix() * getViewMatrix();
}

/*void Camera::update(const glm::vec3& carPosition, const glm::quat& carRotation) {
    glm::vec3 offset = glm::vec3(-5.0f, 1.5f, 0.0f);

    glm::vec3 newPosition = carPosition + glm::rotate(glm::identity<glm::quat>(), offset);
    
    glm::vec3 forwardDirection = glm::normalize(glm::rotate(glm::inverse(glm::identity<glm::quat>()), glm::vec3(1.0f, 0.0f, 0.0f)));
    
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::quat newRotation = glm::quatLookAt(-forwardDirection, up);
    
    setPosition(newPosition);
    setRotation(newRotation);
}*/

void Camera::update(const glm::vec3& carPosition, const glm::quat& carRotation) {
    glm::vec3 offset = glm::vec3(-5.0f, 1.5f, 0.0f);

    float carYaw = atan2(2.0f * (carRotation.y * carRotation.w + carRotation.x * carRotation.z),
                         1.0f - 2.0f * (carRotation.y * carRotation.y + carRotation.x * carRotation.x));

    if (carYaw > glm::pi<float>()) {
        carYaw -= 2.0f * glm::pi<float>();
    } else if (carYaw < -glm::pi<float>()) {
        carYaw += 2.0f * glm::pi<float>();
    }

    glm::quat rotationCamera = glm::normalize(glm::angleAxis(carYaw, glm::vec3(0.0f, 1.0f, 0.0f)));

    glm::vec3 newPosition = carPosition + glm::rotate(rotationCamera, offset);

    glm::vec3 forwardDirection = glm::normalize(glm::rotate(glm::inverse(rotationCamera), glm::vec3(1.0f, 0.0f, 0.0f)));

    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

    glm::quat newRotation = glm::quatLookAt(-forwardDirection, up);

    setPosition(newPosition);
    setRotation(newRotation);
}

void Camera::updateCamera2(const glm::vec3& ballPosition, const glm::vec3& carPosition) {
    glm::vec3 offset = glm::vec3(-5.0f, 0.0f, 0.0f);
    glm::vec3 offsetHauteur = glm::vec3(0.0f, 1.0f, 0.0f);

    glm::vec3 axisToBallCar = glm::normalize(ballPosition - carPosition);

    glm::vec3 newCameraPosition = carPosition - axisToBallCar * glm::length(offset) + offsetHauteur;

    glm::vec3 directionToBall = glm::normalize(ballPosition - newCameraPosition);

    glm::quat newRotation = glm::quatLookAt(directionToBall, glm::vec3(0.0f, 1.0f, 0.0f));

    setPosition(newCameraPosition);
    setRotation(glm::inverse(newRotation));
}



void Camera::fovEffectBoosting(bool isTurboBoosting){
    if(isTurboBoosting){
        if(getFov() < 65)
            setFov(getFov() + 0.25);
    }else{
        if(getFov() > 60)
            setFov(getFov() - 0.125);
    }
}