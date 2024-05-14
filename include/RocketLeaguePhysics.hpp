#pragma once

#include <optional>
#include <array>

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
constexpr float arenaHeight{};
constexpr float arenaWallSideSize{};
constexpr float arenaWallCornerRadius{};
constexpr float arenaGroundCornerRadius{};
constexpr float arenaCeilingCornerRadius{};
constexpr float arenaGoalLength{};
constexpr float arenaGoalHeight{};
constexpr float arenaGoalDepth{};
constexpr float arenaGoalBackCornerRadius{};

float sign(float value) {
    if (value > 0.0f) {
        return 1.0f;
    }
    else if (value < 0.0f) {
        return -1.0f;
    }
    else {
        return 0.0f;
    }
}

const float invSqrt5{1.0f / std::sqrt(5.0f)};

std::optional<Intersection> collideCarArena(const glm::vec3& carPosition1, const glm::vec3& carPosition2, const glm::quat& carRotation) {
    Intersection intersection;
    for (const glm::vec3& hitboxPoint : carHitbox) {
        glm::vec3 point1{carRotation * hitboxPoint + carPosition1};
        glm::vec3 point2{carRotation * hitboxPoint + carPosition2};

        std::optional<Intersection> intersection{collideBallArena(point1, point2, 0.0f)};

        /*const float x = std::abs(point.x - arenaLength * 0.5f);
        const float z = std::abs(point.z - arenaWidth * 0.5f);

        const glm::vec3 closestCornerWall{arenaLength * 0.5f * sign(x), 0.0f, arenaWidth * 0.5f * sign(z)};
        const glm::vec3 roundedWall1{closestCornerWall - glm::vec3{arenaWallSideSize * sign(x), 0.0f, 0.0f}};
        const glm::vec3 roundedWall2{closestCornerWall - glm::vec3{0.0f, 0.0f, arenaWallSideSize * sign(z)}};
        glm::vec3 closestRoundedWallNormal;
        glm::vec3 closestRoundedWall;
        if (glm::distance2(point, roundedWall1) < glm::distance2(point, roundedWall2)) {
            closestRoundedWallNormal = {-sign(x) * invSqrt5, 0.0f, -sign(z) * 2.0f * invSqrt5};
            closestRoundedWall = roundedWall1;
        } 
        else {
            closestRoundedWallNormal = {-sign(x) * 2.0f * invSqrt5, 0.0f, -sign(z) * invSqrt5};
            closestRoundedWall = roundedWall2;
        }

        if ()

        if (x < arenaWallSideSize && z < arenaWallSideSize) {
            closestHorizontalCornerPoint = glm::vec3(arenaLength * 0.5f * sign(point.x), 0.0f, point.z);
        }
        else if (x < z) {
            closestHorizontalCornerPoint = glm::vec3(arenaLength * 0.5f * sign(point.x), 0.0f, point.z);
        }
        else {
            closestHorizontalCornerPoint = glm::vec3(point.x, 0.0f, arenaWidth * 0.5f * sign(point.z));
        }

        if (point.y > arenaHeight * 0.5f) {
            closestHorizontalCornerPoint.y = arenaHeight;
        }
        sign(hitboxPoint.x)*/
    }
}

std::optional<Intersection> collideCarBall() {
    
}

std::optional<Intersection> collideBallArena(const glm::vec3& ballPositionStart, const glm::vec3& ballPositionEnd, float radius) {
    float t = 2.0f;

    /*const float a = std::abs(arenaLength - radius);
    if (std::abs(ballPositionStart.x) < std::abs(a) && std::abs(a) < std::abs(ballPositionEnd.x)) {

    }

    const float b = std::abs(arenaWidth - radius);
    if (std::abs(ballPositionStart.z) < std::abs(b) && std::abs(b) < std::abs(ballPositionEnd.z)) {

    }*/

    const float c = std::abs(arenaHeight - radius);
    if (std::abs(ballPositionStart.y - arenaHeight * 0.5f) < std::abs(c) && std::abs(c) < std::abs(ballPositionEnd.y - arenaHeight * 0.5f)) {
        const float tp{};
        
        t = std::min(t, tp);
    }
}