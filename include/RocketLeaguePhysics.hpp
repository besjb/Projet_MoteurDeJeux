#pragma once

#include <optional>
#include <array>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

struct Intersection {
    float t;
    glm::vec3 normal;
    glm::vec3 collisionPoint;
};

constexpr static std::array<glm::vec3, 9> carHitbox{
    glm::vec3{-0.0f, 0.5f, 0.0f},
    glm::vec3{-0.6f, 0.55f, 0.3f},
    glm::vec3{-0.6f, 0.55f, -0.3f},
    glm::vec3{0.6f, 0.2f, 0.3f},
    glm::vec3{0.6f, 0.2f, -0.3f},
    glm::vec3{0.6f, 0.0f, 0.3f},
    glm::vec3{0.6f, 0.0f, -0.3f},
    glm::vec3{-0.55f, 0.0f, 0.3f},
    glm::vec3{-0.55f, 0.0f, -0.3f}
};

constexpr static std::array<glm::vec3, 9> carHitboxTriangles{
    glm::vec3{-0.0f, 0.5f, 0.0f},
    glm::vec3{-0.6f, 0.55f, 0.3f},
    glm::vec3{-0.6f, 0.55f, -0.3f},
    glm::vec3{0.6f, 0.2f, 0.3f},
    glm::vec3{0.6f, 0.2f, -0.3f},
    glm::vec3{0.55f, 0.0f, 0.3f},
    glm::vec3{0.55f, 0.0f, -0.3f},
    glm::vec3{-0.5f, 0.0f, 0.3f},
    glm::vec3{-0.5f, 0.0f, -0.3f}
};

constexpr float arenaLength{};
constexpr float arenaWidth{};
constexpr float arenaHeight{10.0f};
constexpr float arenaWallSideSize{};
constexpr float arenaWallCornerRadius{};
constexpr float arenaGroundCornerRadius{};
constexpr float arenaCeilingCornerRadius{};
constexpr float arenaGoalLength{};
constexpr float arenaGoalHeight{};
constexpr float arenaGoalDepth{};
constexpr float arenaGoalBackCornerRadius{};

float sign(float value);

const float invSqrt5{1.0f / std::sqrt(5.0f)};

std::optional<Intersection> collideCarArena(const glm::vec3& carPosition1, const glm::vec3& carPosition2, const glm::quat& carRotation);

std::optional<Intersection> collideCarBall();

std::optional<Intersection> collideBallArena(const glm::vec3& ballPositionStart, const glm::vec3& ballPositionEnd, float radius);