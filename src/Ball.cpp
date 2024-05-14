#include "Ball.hpp"
#include "RocketLeague.hpp"

extern RocketLeague* globalRocketLeague;

Ball::Ball(TransformTree* transformTree) :
    transformTree{transformTree},
    mass{1.0f},
    position{glm::vec3(0.0f)},
    velocity{glm::vec3(0.0f)}
{}

Ball& Ball::setMass(float mass) {
    this->mass = mass;
    return *this;
}

Ball& Ball::setPosition(const glm::vec3& position) {
    this->position = position;
    return *this;
}

Ball& Ball::setVelocity(const glm::vec3& velocity) {
    this->velocity = velocity;
    return *this;
}

TransformTree* Ball::getTransformTree() const {
    return transformTree;
}

float Ball::getMass() const {
    return mass;
}

glm::vec3 Ball::getPosition() const {
    return position;
}

glm::vec3 Ball::getVelocity() const {
    return velocity;
}

void Ball::updatePhysics(float delta) {
    position += velocity * delta;
    velocity += globalRocketLeague->getGravity() * delta;

    transformTree->transform.setTranslation(position);
}