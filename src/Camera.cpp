#include "Camera.hpp"

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

    //std::cout << getFov() << std::endl;
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


void Camera::updateCamera2(const glm::vec3& ballPosition, const glm::vec3& carPosition, const glm::quat& carRotation) {
    glm::vec3 offset = glm::vec3(-3.0f, 1.0f, 0.0f);

    glm::vec3 directionToBallFromCar = glm::normalize(ballPosition - carPosition);
    
    glm::vec3 offsetDirectionToBallFromCar = directionToBallFromCar + offset;
    
    glm::vec3 rotatedDirectionToBallFromCar = glm::rotate(carRotation, offsetDirectionToBallFromCar);

    glm::vec3 cameraPosition = carPosition + offsetDirectionToBallFromCar;
    
    glm::vec3 directionToBall = glm::normalize(ballPosition - cameraPosition);
    
    glm::quat newRotation = glm::quatLookAt(directionToBall, glm::vec3(0.,1.,0.));
    
    setRotation(glm::inverse(newRotation));
    setPosition(cameraPosition);
}

/*void Camera::updateCamera2(const glm::vec3& ballPosition, const glm::vec3& carPosition) {
    glm::vec3 offset = glm::vec3(-4.0f, 0.0f, 0.0f);

    // Calculate direction from car to ball
    glm::vec3 directionToBallFromCar = glm::normalize(ballPosition - carPosition);

    // Apply offset from car along the direction to the ball
    glm::vec3 cameraPosition = (directionToBallFromCar * offset) + glm::vec3(0.0f, 1.0f, 0.0f);

    // Calculate the rotation needed for camera alignment
    glm::vec3 rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
    float rotationAngle = glm::acos(glm::dot(glm::vec3(0.0f, 1.0f, 0.0f), directionToBallFromCar));
    glm::quat rotationQuat = glm::angleAxis(rotationAngle, rotationAxis);

    // Calculate the new rotation for the camera to look at the ball
    glm::quat newRotation = glm::quatLookAt(directionToBallFromCar, glm::vec3(0.0f, 1.0f, 0.0f));

    // Set the rotation and position of the camera
    setRotation(glm::inverse(newRotation * rotationQuat)); // Inverse the rotation to match the camera's rotation
    setPosition(cameraPosition);
}*/

void Camera::fovEffectBoosting(bool isTurboBoosting){
    if(isTurboBoosting){
        if(getFov() < 65)
            setFov(getFov() + 0.25);
    }else{
        if(getFov() > 60)
            setFov(getFov() - 0.25);
    }
}