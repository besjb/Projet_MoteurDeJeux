#include "Car.hpp"
#include "RocketLeague.hpp"
#include "MeshMaterial.hpp"
#include "RocketLeaguePhysics.hpp"

#include <algorithm>

#include <glm/gtx/string_cast.hpp>

extern RocketLeague* globalRocketLeague;

Car::Car(TransformTree* transformTree) :
    transformTree{transformTree},
    mass{1.0f},
    position{glm::vec3(0.0f)},
    rotation{glm::identity<glm::quat>()},
    velocity{glm::vec3(0.0f)},
    centrifuge{glm::vec3(0.0f)},
    angularVelocity{glm::vec3(0.0f)},
    forwardVelocity{0.0f},
    forwardAcceleration{0.0f},
    forwardDeceleration{0.0f},
    turnSensitivity{0.0f},
    doWheelsCollide{false},
    jumpTime{0.0f},
    doubleJumpTime{0.0f},
    directionalDoubleJumpTime{0.0f},
    boosting{false},
    turboBoosting{false},
    turboBoostCooldown{0.0f},
    drifting{false}
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

Car& Car::setAngularVelocity(const glm::vec3 angularVelocity) {
    this->angularVelocity = angularVelocity;
    return *this;
}

Car& Car::setMovementAcceleration(const glm::vec3& movementAcceleration) {
    this->movementAcceleration = movementAcceleration;
    return *this;
}

Car& Car::setMovementAngle(const glm::vec3 movementAngle) {
    this->movementAngle = movementAngle;
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

glm::vec3 Car::getAngularVelocity() const {
    return angularVelocity;
}

glm::vec3 Car::getMovementAcceleration() const {
    return movementAcceleration;
}

glm::vec3 Car::getMovementAngle() const {
    return movementAngle;
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

glm::vec3 Car::getUpVector() const {
    return rotation * glm::vec3{0.0f, 1.0f, 0.0f};
}

glm::vec3 Car::getFrontVector() const {
    return rotation * glm::vec3{1.0f, 0.0f, 0.0f};
}

bool Car::wheelsCollide() const {
    return doWheelsCollide;
}

void Car::startJump() {
    velocity += centrifuge;
    jumpTime = 0.15f;
}

void Car::stopJump() {
    jumpTime = -1.0f;
    doubleJumpTime = 1.5f;
}

bool Car::isJumping() const {
    return jumpTime > 0.0f;
}

bool Car::canDoubleJump() const {
    return doubleJumpTime > 0.0f;
}

void Car::doubleJump() {
    doubleJumpTime = -1.0f;
    if (movementAcceleration != glm::vec3(0.0f)) {
        velocity.y = 0.0f;
        directionalDoubleJumpTime = 0.55f;
        glm::vec3 r{std::clamp(movementAcceleration.x - movementAcceleration.y, -14.0f, 14.0f), 0.0f, movementAcceleration.z};
        r = glm::normalize(r) * 14.0f;
        const glm::vec3 d{-0.25f * movementAcceleration.z, 0.0f, std::clamp(- movementAcceleration.y + movementAcceleration.x, -3.5f, 3.5f)};
        velocity += rotation * d;
        movementAngle = 0.65f * r;
    }
    else {        
        velocity += 4.5f * getUpVector();
    }
}

void Car::startBoost() {
    boosting = true;
}

void Car::stopBoost() {
    boosting = false;
    if (turboBoosting) {
        turboBoostCooldown = 1.0f;
    }
}

bool Car::isBoosting() const {
    return boosting;
}

bool Car::isTurboBoosting() const {
    return turboBoosting;
}

void Car::startDrifting() {
    drifting = true;
}

void Car::stopDrifting() {
    drifting = false;
}

bool Car::isDrifting() {
    return drifting;
}

void Car::updateAnimations(float delta) {
    if (jumpTime > 0.0f) {
        velocity += 45.0f * getUpVector() * delta;
        jumpTime -= delta;
        if (jumpTime <= 0.0f) {
            stopJump();
        }
    }
    if (doubleJumpTime > 0.0f) {
        doubleJumpTime -= delta;
    }
    if (turboBoostCooldown > 0.0f) {
        turboBoostCooldown -= delta;
        if (turboBoostCooldown <= 0.0f) {
            turboBoosting = false;
        }
    }
    if (directionalDoubleJumpTime > 0.0f) {
        directionalDoubleJumpTime -= delta;
    }
}

void Car::updatePhysics(float delta) {
    if (directionalDoubleJumpTime <= 0.0f) {
        velocity += globalRocketLeague->getGravity() * delta;
    }

    rotation = glm::quat{delta * angularVelocity} * rotation;
    const float velocityLength{glm::length(velocity)};
    if (doWheelsCollide) {
        directionalDoubleJumpTime = -1.0f;
        movementAngle = glm::vec3(0.0f);

        float frontSpeed{glm::dot(velocity, getFrontVector())};
        if (boosting) {
            velocity += rotation * glm::vec3(28.0f, 0.0f, 0.0f) * delta;
        }
        else {
            velocity += rotation * glm::vec3(forwardAcceleration, 0.0f, 0.0f) * delta;
        }
        
        float turnSpeed = std::min(std::abs(frontSpeed * 0.25f), 2.0f);
        if (frontSpeed < 0.0f) {
            turnSpeed = -turnSpeed;
        }

        glm::quat turn{turn = delta * turnSensitivity * turnSpeed * getUpVector()};
        rotation = rotation * turn;
        if (!drifting) {
            velocity = turn * velocity;
        }

        const glm::vec3 side{glm::cross(getFrontVector(), getUpVector())};

        // Décélération
        if (forwardAcceleration == 0.0f) {
            velocity -= delta * sign(frontSpeed) * getFrontVector() * std::max(1.0f, std::abs(glm::dot(getFrontVector(), velocity)));
        }

        if (!drifting) {
            velocity -= std::min(4.0f * delta, 1.0f) * side * glm::dot(side, velocity);
        }
        else {
            velocity -= std::min(0.5f * delta, 1.0f) * side * glm::dot(side, velocity);
        }
    }
    else {
        if (boosting) {
            velocity += rotation * glm::vec3(24.0f, 0.0f, 0.0f) * delta;
        }
        if (directionalDoubleJumpTime <= 0.0f) {
            movementAngle += movementAcceleration * delta;
            movementAngle = glm::clamp(movementAngle, -4.5f, 4.5f);
            for (int i{}; i < 3; ++i) {
                if (movementAngle[i] != 0.0f && movementAcceleration[i] == 0.0f) {
                    movementAngle[i] *= 1.0f - std::min(1.0f, 4.0f * delta);
                }
            }
        }
        rotation = rotation * glm::quat(delta * movementAngle);
    }
    
    float maxVelocity;
    if (doWheelsCollide) {
        if (boosting || turboBoosting) {
            maxVelocity = 28.0f;

            if (velocityLength > 27.0f) {
                turboBoosting = true;
            }
            else {
                turboBoosting = false;
            }
        }
        else {
            maxVelocity = 16.0f;
        }
    }
    else {
        maxVelocity = 18.0f;
    }

    if (velocityLength > maxVelocity) {
        velocity = glm::mix(velocity, velocity * maxVelocity / velocityLength, std::min(2.0f * delta, 1.0f));
    }

    position += velocity * delta;    

    bool intersects{true};
    std::size_t i{};
    doWheelsCollide = false;

    //constexpr glm::vec3 carWheelsCenter{0.025f, 0.0f, 0.0f};
    while (intersects && i < 10) {
        std::optional<Intersection> intersectionOpt{collideCarArena(position, rotation)};
        intersects = intersectionOpt.has_value();
        if (intersects) {
            Intersection intersection{intersectionOpt.value()};
            position += intersection.normal * intersection.penetration;

            const float normalVelocity = glm::dot(velocity, intersection.normal);
            const glm::vec3 carUpVector{getUpVector()};
            float replDiff{glm::dot(carUpVector, intersection.normal)};

            bool wheelCollision{false};
            if (normalVelocity <= 0.0f) {

                bool frameCollision = replDiff < 0.2f;

                wheelCollision = replDiff > 0.95f;
                if (wheelCollision) {
                    directionalDoubleJumpTime = -1.0f;
                    doWheelsCollide = true;
                    doubleJumpTime = std::numeric_limits<float>::infinity();
                }

                const glm::vec3 newCollisionPoint{intersection.collisionPoint + intersection.normal * (intersection.penetration - 0.01f)};

                glm::quat newRotation;
                if (!frameCollision) {
                    newRotation = glm::slerp(rotation, glm::quatLookAt(glm::cross(intersection.normal, glm::cross(intersection.normal, glm::cross(getFrontVector(), intersection.normal))), intersection.normal), std::min(9.0f * delta, 1.0f));
                }
                else {
                    newRotation = glm::slerp(rotation, glm::quatLookAt(glm::cross(-intersection.normal, glm::cross(-intersection.normal, glm::cross(getFrontVector(), -intersection.normal))), -intersection.normal), std::min(2.5f * delta, 1.0f));
                }
                const glm::quat rotDiff = newRotation * glm::conjugate(rotation);
                rotation = newRotation;
                const glm::vec3 collDiff = newCollisionPoint - position;
                const glm::vec3 rotatedDiff = rotDiff * collDiff;
                position += 0.999f * (collDiff - rotatedDiff);

                const float jn = -(1.0f + (frameCollision ? 0.3f : 0.0f)) * normalVelocity;
                velocity += jn * intersection.normal;
                ++i;
            }
            rotation = glm::normalize(rotation);
        }
    }

    transformTree->transform
        .setTranslation(position)
        .setRotation(rotation);
        
}