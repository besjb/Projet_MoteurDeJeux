#include "Car.hpp"
#include "RocketLeague.hpp"
#include "MeshMaterial.hpp"
#include "RocketLeaguePhysics.hpp"

#include <glm/gtx/string_cast.hpp>

extern RocketLeague* globalRocketLeague;

Car::Car(TransformTree* transformTree) :
    transformTree{transformTree},
    mass{1.0f},
    position{glm::vec3(0.0f)},
    rotation{glm::identity<glm::quat>()},
    velocity{glm::vec3(0.0f)},
    front{glm::vec3(1.,0.,0.)},
    angularVelocity{glm::vec3(0.0f)},
    forwardVelocity{0.0f},
    forwardAcceleration{0.0f},
    forwardDeceleration{0.0f},
    turnSensitivity{0.0f}
{}

Car& Car::setMass(float mass) {
    this->mass = mass;
    return *this;
}

Car& Car::setPosition(const glm::vec3& position) {
    this->position = position;
    return *this;
}

Car& Car::setRotation(const glm::quat& rotation) {
    this->rotation = rotation;
    return *this;
}

Car& Car::setVelocity(const glm::vec3& velocity) {
    this->velocity = velocity;
    return *this;
}

Car& Car::setFront(const glm::vec3& front) {
    this->front = front;
    return *this;
}

Car& Car::setAngularVelocity(const glm::vec3 angularVelocity) {
    this->angularVelocity = angularVelocity;
    return *this;
}

Car& Car::setForwardVelocity(float forwardVelocity) {
    this->forwardVelocity = forwardVelocity;
    return *this;
}

Car& Car::setForwardAcceleration(float forwardAcceleration) {
    this->forwardAcceleration = forwardAcceleration;
    return *this;
}

Car& Car::setForwardDeceleration(float forwardDeceleration) {
    this->forwardDeceleration = forwardDeceleration;
    return *this;
}

Car& Car::setTurnSensitivity(float turnSensitivity) {
    this->turnSensitivity = turnSensitivity;
    return *this;
}

TransformTree* Car::getTransformTree() const {
    return transformTree;
}

float Car::getMass() const {
    return mass;
}

glm::vec3 Car::getPosition() const {
    return position;
}

glm::quat Car::getRotation() const {
    return rotation;
}

glm::vec3 Car::getVelocity() const {
    return velocity;
}

glm::vec3 Car::getFront() const{
    return front;
}

glm::vec3 Car::getAngularVelocity() const {
    return angularVelocity;
}

float Car::getForwardVelocity() const {
    return forwardVelocity;
}

float Car::getForwardAcceleration() const {
    return forwardAcceleration;
}

float Car::getForwardDeceleration() const {
    return forwardDeceleration;
}

float Car::getTurnSensitivity() const {
    return turnSensitivity;
}

void Car::updatePhysics(float delta) {
    rotation = glm::quat{delta * angularVelocity} * rotation;
    rotation *= front;
    velocity += globalRocketLeague->getGravity() * delta;
    velocity += rotation * glm::vec3(forwardAcceleration, 0.0f, 0.0f) * delta;
    if (glm::length(velocity) > 25.0f) {
        velocity *= 25.0f / glm::length(velocity);
    }
    glm::vec3 nextPosition{position + velocity * delta};

    bool intersects{true};
    std::size_t i{};
    while (intersects && position != nextPosition && i < 10) {
        std::optional<Intersection> intersectionOpt{collideCarArena(position, nextPosition, rotation)};
        intersects = intersectionOpt.has_value();
        if (intersects) {
            Intersection intersection{intersectionOpt.value()};
            position += velocity * intersection.t * delta;
            const float normalVelocity = glm::dot(velocity, intersection.normal);
            const float jn = -(1.0f + 0.0f) * normalVelocity;
            velocity += jn * intersection.normal;
            delta *= (1.0f - intersection.t);
            nextPosition = position + velocity * delta;
            ++i;
        }
    }
    position = nextPosition;

    transformTree->transform
        .setTranslation(position)
        .setRotation(rotation);


    
}