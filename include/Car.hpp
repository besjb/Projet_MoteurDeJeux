#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>

#include "Scene.hpp"
#include "Utility.hpp"

class Car {
public:

    Car(TransformTree* transformTree);

    Car& setMass(float mass);

    Car& setPosition(const glm::vec3& position);

    Car& setRotation(const glm::quat& rotation);

    Car& setVelocity(const glm::vec3& velocity);

    Car& setAngularVelocity(const glm::vec3 angularVelocity);

    Car& setForwardVelocity(float forwardVelocity);

    Car& setForwardAcceleration(float forwardAcceleration);

    Car& setForwardDeceleration(float forwardDeceleration);

    Car& setTurnSensitivity(float turnSensitivity);

    TransformTree* getTransformTree() const;

    float getMass() const;

    glm::vec3 getPosition() const;

    glm::quat getRotation() const;

    glm::vec3 getVelocity() const;

    glm::vec3 getAngularVelocity() const;

    glm::vec3 getUpVector() const;

    bool wheelsCollide() const;

    

    void startJump();

    void stopJump();

    bool isJumping() const;

    bool canDoubleJump() const;

    void doubleJump();
    
    float getForwardVelocity() const;

    float getForwardAcceleration() const;
    
    float getForwardDeceleration() const;

    float getTurnSensitivity() const;

    void updateAnimations(float delta);

    void updatePhysics(float delta);

private:

    TransformTree* transformTree;

    float mass;
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;

    float forwardVelocity;
    float forwardAcceleration;
    float forwardDeceleration;
    float turnSensitivity;

    bool doWheelsCollide;

    float jumpTime;
    float doubleJumpTime;

};