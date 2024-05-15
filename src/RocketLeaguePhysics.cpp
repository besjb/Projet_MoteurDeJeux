#include "RocketLeaguePhysics.hpp"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

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

std::optional<Intersection> collideCarArena(const glm::vec3& carPosition, const glm::quat& carRotation) {
    bool intersects{false};
    Intersection bestIntersection;
    bestIntersection.penetration = std::numeric_limits<float>::infinity();
    for (const glm::vec3& hitboxPoint : carHitbox) {
        glm::vec3 point{carRotation * hitboxPoint + carPosition};

        std::optional<Intersection> intersection{collideBallArena(point, 0.0f)};
        if (intersection.has_value() && intersection.value().penetration < bestIntersection.penetration) {
            intersects = true;
            bestIntersection = intersection.value();
        }
    }

    return intersects ? std::make_optional(bestIntersection) : std::nullopt;
}

std::optional<Intersection> collideCarBall(const glm::vec3& carPosition, const glm::quat& carRotation, const glm::vec3& ballPosition, float ballRadius) {
    bool intersects{false};
    Intersection bestIntersection;
    bestIntersection.penetration = std::numeric_limits<float>::infinity();
    for (const glm::vec3& hitboxPoint : carHitbox) {
        glm::vec3 point{carRotation * hitboxPoint + carPosition};

        if (glm::distance2(point, ballPosition) < ballRadius * ballRadius) {

            const float penetration{ballRadius - glm::distance(point, ballPosition)};
            if (penetration < bestIntersection.penetration) {
                intersects = true;
                bestIntersection.penetration = penetration;
                bestIntersection.normal = glm::normalize(point - ballRadius);
                bestIntersection.collisionPoint = point;
            }
        }
    }

    return intersects ? std::make_optional(bestIntersection) : std::nullopt;
}

std::optional<Intersection> collideBallArena(const glm::vec3& ballPosition, float radius) {

    // Bottom
    if (ballPosition.y < radius) {
        return std::make_optional(Intersection{
            radius - ballPosition.y,
            glm::vec3(0.0f, 1.0f, 0.0f),
            ballPosition
        });
    }

    // Top
    if (ballPosition.y > arenaHeight - radius) {
        return std::make_optional(Intersection{
            ballPosition.y - (arenaHeight - radius),
            glm::vec3(0.0f, -1.0f, 0.0f),
            ballPosition
        });
    }

    // Back
    if (ballPosition.x < -0.5f * arenaLength - radius) {
        return std::make_optional(Intersection{
            -ballPosition.x - (0.5f * arenaLength - radius),
            glm::vec3(1.0f, 0.0f, 0.0f),
            ballPosition
        });
    }

    // Front
    if (ballPosition.x > 0.5f * arenaLength - radius) {
        return std::make_optional(Intersection{
            ballPosition.x - (0.5f * arenaLength - radius),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            ballPosition
        });
    }

    // Left
    if (ballPosition.z < -0.5f * arenaWidth - radius) {
        return std::make_optional(Intersection{
            -ballPosition.z - (0.5f * arenaWidth - radius),
            glm::vec3(0.0f, 0.0f, 1.0f),
            ballPosition
        });
    }

    // Right
    if (ballPosition.z > 0.5f * arenaWidth - radius) {
        return std::make_optional(Intersection{
            ballPosition.z - (0.5f * arenaWidth - radius),
            glm::vec3(0.0f, 0.0f, -1.0f),
            ballPosition
        });
    }

    const float sqrt2{std::sqrt(2.0f)};
    const float invSqrt2{1.0f / sqrt2};

    float a;
    a = 0.5f * arenaLength - ballPosition.x + 0.5f * arenaWidth - ballPosition.z;
    if (a < arenaWallSideSize) {
        return std::make_optional(Intersection{
            arenaWallSideSize - a,
            glm::vec3(-invSqrt2, 0.0f, -invSqrt2),
            ballPosition
        });
    }

    a = 0.5f * arenaLength + ballPosition.x + 0.5f * arenaWidth - ballPosition.z;
    if (a < arenaWallSideSize) {
        return std::make_optional(Intersection{
            arenaWallSideSize - a,
            glm::vec3(invSqrt2, 0.0f, -invSqrt2),
            ballPosition
        });
    }

    a = 0.5f * arenaLength - ballPosition.x + 0.5f * arenaWidth + ballPosition.z;
    if (a < arenaWallSideSize) {
        return std::make_optional(Intersection{
            arenaWallSideSize - a,
            glm::vec3(-invSqrt2, 0.0f, invSqrt2),
            ballPosition
        });
    }

    a = 0.5f * arenaLength + ballPosition.x + 0.5f * arenaWidth + ballPosition.z;
    if (a < arenaWallSideSize) {
        return std::make_optional(Intersection{
            arenaWallSideSize - a,
            glm::vec3(invSqrt2, 0.0f, invSqrt2),
            ballPosition
        });
    }

    /*const float closestRoundedVerticalX{sign(ballPosition.x) * 0.5f * arenaLength - arenaWallSideSize * sign(ballPosition.x)};
    const float closestRoundedVerticalZ{sign(ballPosition.z) * 0.5f * arenaWidth - arenaWallSideSize * sign(ballPosition.z)};
    const glm::vec3 closestRoundedVertical1{closestRoundedVerticalX, ballPosition.y, sign(ballPosition.z) * 0.5f * arenaWidth};
    const glm::vec3 closestRoundedVertical2{sign(ballPosition.x) * 0.5f * arenaLength, ballPosition.y, closestRoundedVerticalZ};

    const float invSqrt3{1.0f / std::sqrt(3.0f)};

    glm::vec3 closestRoundedVertical;
    glm::vec3 verticalCornerNormal;
    if (glm::distance2(ballPosition, closestRoundedVertical1) < glm::distance2(ballPosition, closestRoundedVertical2)) {
        closestRoundedVertical = closestRoundedVertical1;
        verticalCornerNormal = glm::vec3{-invSqrt3 * sign(ballPosition.x), 0.0f, -2.0f * invSqrt3 * sign(ballPosition.z)};
    }
    else {
        closestRoundedVertical = closestRoundedVertical2;
        verticalCornerNormal = glm::vec3{-2.0f * invSqrt3 * sign(ballPosition.x), 0.0f, -invSqrt3 * sign(ballPosition.z)};
    }

    if (glm::distance2(ballPosition, closestRoundedVertical) < 0.25 * arenaWallCornerRadius * arenaWallCornerRadius) {
        const glm::vec3 cornerCenter{closestRoundedVertical + arenaWallCornerRadius * verticalCornerNormal};
        return std::make_optional(Intersection{
            std::abs(glm::distance(ballPosition, cornerCenter) - arenaWallCornerRadius),
            glm::normalize(cornerCenter - ballPosition),
            ballPosition
        });
    }*/

    return std::nullopt;
}