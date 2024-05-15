#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>

class Camera {
public:

    Camera(
        const glm::vec3& position = {},
        const glm::quat& rotation = glm::vec3{},
        float fov = 90,
        float aspectRatio = 16.0f / 9.0f,
        float nearClip = 0.1f,
        float farClip = 2000000.0f
    );

    void setAspectRatio(float aspectRatio);

    void setFov(float fov);

    void setPosition(const glm::vec3& position);

    void setPosition(float x, float y, float z);

    void move(const glm::vec3& offset);

    void move(float x, float y, float z);

    void moveRelative(const glm::vec3& offset);

    void moveRelative(float x, float y, float z);

    void setRotation(const glm::quat& rotation);

    void setRotation(float x, float y, float z);

    void rotate(const glm::quat& rotation);

    void rotate(float x, float y, float z);

    float getAspectRatio() const;

    float getFov() const;

    glm::vec3 getPosition() const;

    glm::quat getRotation() const;

    glm::mat4 getViewMatrix() const;

    glm::mat4 getProjectionMatrix() const;

    glm::mat4 getViewProjectionMatrix() const;

    void update(const glm::vec3& targetPosition, const glm::quat& carRotation);

    void updateCamera2(const glm::vec3& ballPosition, const glm::vec3& carPosition, const glm::quat& carRotation);

private:
    
    glm::vec3 position;
    glm::quat rotation;
    float fov;
    float aspectRatio;
    float nearClip;
    float farClip;

};