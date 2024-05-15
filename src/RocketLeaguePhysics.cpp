#include "RocketLeaguePhysics.hpp"
#include <iostream>

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

std::optional<Intersection> collideCarBall() {
    return std::nullopt;
}

std::optional<Intersection> collideBallArena(const glm::vec3& ballPosition, float radius) {
    if (ballPosition.y < radius) {
        return std::make_optional(Intersection{
            radius - ballPosition.y,
            glm::vec3(0.0f, 1.0f, 0.0f),
            ballPosition
        });
    }

    if (ballPosition.y > arenaHeight - radius) {
        return std::make_optional(Intersection{
            ballPosition.y - (arenaHeight - radius),
            glm::vec3(0.0f, -1.0f, 0.0f),
            ballPosition
        });
    }

    /*if (ballPosition.x < -(0.5f * arenaLength - radius)) {
        return std::make_optional(Intersection{
            (0.5f * arenaLength - radius) + ballPosition.x,
            glm::vec3(1.0f, 0.0f, 0.0f),
            ballPosition
        });
    }*/

    if (ballPosition.x < -0.5f * arenaLength - radius) {
        return std::make_optional(Intersection{
            -ballPosition.x - (0.5f * arenaLength - radius),
            glm::vec3(1.0f, 0.0f, 0.0f),
            ballPosition
        });
    }

    if (ballPosition.x > 0.5f * arenaLength - radius) {
        return std::make_optional(Intersection{
            ballPosition.x - (0.5f * arenaLength - radius),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            ballPosition
        });
    }

    if (ballPosition.z < -0.5f * arenaWidth - radius) {
        return std::make_optional(Intersection{
            -ballPosition.z - (0.5f * arenaWidth - radius),
            glm::vec3(0.0f, 0.0f, 1.0f),
            ballPosition
        });
    }

    if (ballPosition.z > 0.5f * arenaWidth - radius) {
        return std::make_optional(Intersection{
            ballPosition.z - (0.5f * arenaWidth - radius),
            glm::vec3(0.0f, 0.0f, -1.0f),
            ballPosition
        });
    }

    return std::nullopt;
}