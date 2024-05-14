#pragma once

#include <glm/glm.hpp>
#include <Scene.hpp>

class Ball {
public:

    Ball(TransformTree* transformTree);

    Ball& setMass(float mass);

    Ball& setPosition(const glm::vec3& position);

    Ball& setVelocity(const glm::vec3& velocity);

    TransformTree* getTransformTree() const;

    float getMass() const;

    glm::vec3 getPosition() const;

    glm::vec3 getVelocity() const;

    void updatePhysics(float delta);

private:

    TransformTree* transformTree;

    float mass;
    glm::vec3 position;
    glm::vec3 velocity;

};