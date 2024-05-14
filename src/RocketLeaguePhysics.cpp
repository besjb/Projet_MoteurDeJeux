#include "RocketLeaguePhysics.hpp"

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

std::optional<Intersection> collideCarArena(const glm::vec3& carPosition1, const glm::vec3& carPosition2, const glm::quat& carRotation) {
    Intersection bestIntersection;
    bestIntersection.t = 2.0f;
    for (const glm::vec3& hitboxPoint : carHitbox) {
        glm::vec3 point1{carRotation * hitboxPoint + carPosition1};
        glm::vec3 point2{carRotation * hitboxPoint + carPosition2};

        std::optional<Intersection> intersection{collideBallArena(point1, point2, 0.0f)};
        if (intersection.has_value() && intersection.value().t < bestIntersection.t) {
            bestIntersection = intersection.value();
        }

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

    if (bestIntersection.t < 1.0f) {
        return std::make_optional(bestIntersection);
    }
    
    return std::nullopt;
}

std::optional<Intersection> collideCarBall() {
    
}

std::optional<Intersection> collideBallArena(const glm::vec3& ballPositionStart, const glm::vec3& ballPositionEnd, float radius) {
    float t = 2.0f;
    glm::vec3 normal;
    glm::vec3 collisionPoint;

    /*const float a = std::abs(arenaLength - radius);
    if (std::abs(ballPositionStart.x) < std::abs(a) && std::abs(a) < std::abs(ballPositionEnd.x)) {

    }

    const float b = std::abs(arenaWidth - radius);
    if (std::abs(ballPositionStart.z) < std::abs(b) && std::abs(b) < std::abs(ballPositionEnd.z)) {

    }*/

    if (ballPositionStart.y != ballPositionEnd.y && ballPositionStart.y >= radius && radius >= ballPositionEnd.y) {
        const float tp{std::abs(ballPositionStart.y / (ballPositionEnd - ballPositionStart).y)};
        if (tp < t) {
            t = tp;
            collisionPoint = ballPositionStart + (ballPositionEnd - ballPositionStart) * t;
            normal = glm::vec3{0.0f, 1.0f, 0.0f};
        }
    }

    /*const float c = std::abs(arenaHeight - radius);
    if (ballPositionStart.y != ballPositionEnd.y && std::abs(ballPositionStart.y - arenaHeight * 0.5f) < std::abs(c) && std::abs(c) < std::abs(ballPositionEnd.y - arenaHeight * 0.5f)) {
        const float tp{std::abs((c - ballPositionStart + arenaHeight * 0.5f).y / (ballPositionEnd - ballPositionStart).y)};
        if (tp < t) {
            t = tp;
            collisionPoint = ballPositionStart + (ballPositionEnd - ballPositionStart) * t;
            normal = glm::vec3{0.0f, (collisionPoint.y > arenaHeight * 0.5f) ? -1.0f : 1.0f, 0.0f};
        }
    }*/

    if (t < 1.0f) {
        return std::make_optional(Intersection{t, normal, collisionPoint});
    }

    return std::nullopt;
}