#include "Ball.hpp"
#include "RocketLeague.hpp"
#include "RocketLeaguePhysics.hpp"

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
    velocity += globalRocketLeague->getGravity() * delta;

    const float velocityLength{glm::length(velocity)};
    if (velocityLength > 15.0f) {
        velocity *= 15.0f / velocityLength;
    }
    
    position += velocity * delta;
    //glm::vec3 nextPosition{position + velocity * delta};

    bool intersects{true};
    std::optional<Intersection> intersectionOpt{collideBallArena(position, 0.75f)};
    intersects = intersectionOpt.has_value();
    if (intersects) {
        Intersection intersection{intersectionOpt.value()};
        position += intersection.normal * intersection.penetration;
        const float normalVelocity = glm::dot(velocity, intersection.normal);

        const float jn = -(1.0f + 0.4f) * normalVelocity;
        velocity += jn * intersection.normal;
    }

    /*bool intersects{true};
    std::size_t i{};
    while (intersects && position != nextPosition && i < 10) {
        std::optional<Intersection> intersectionOpt{collideBallArena(position, nextPosition, 0.45f)};
        intersects = intersectionOpt.has_value();
        if (intersects) {
            Intersection intersection{intersectionOpt.value()};
            position += velocity * intersection.t * delta;
            const float normalVelocity = glm::dot(velocity, intersection.normal);

            const float jn = -(1.0f + 0.4f) * normalVelocity;
            velocity += jn * intersection.normal;
            delta *= (1.0f - intersection.t);
            nextPosition = position + velocity * delta;
            ++i;
        }
    }
    position = nextPosition;*/

    transformTree->transform.setTranslation(position);
}