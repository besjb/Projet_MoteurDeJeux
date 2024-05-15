#pragma once

#include <optional>
#include <array>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

struct Intersection {
    float penetration;
    glm::vec3 normal;
    glm::vec3 collisionPoint;
};

constexpr std::array<glm::vec3, 9> carHitbox{
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

constexpr std::array<glm::vec3, 9> carHitboxTriangles{
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

constexpr float arenaLength{80.0f};
constexpr float arenaWidth{40.0f};
constexpr float arenaHeight{26.0f};
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

std::optional<Intersection> collideCarArena(const glm::vec3& carPosition, const glm::quat& carRotation);

std::optional<Intersection> collideCarBall();

std::optional<Intersection> collideBallArena(const glm::vec3& ballPosition, float radius);