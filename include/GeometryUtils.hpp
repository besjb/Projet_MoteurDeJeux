#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <optional>
#include <array>
#include <vector>

/*struct IntersectionInfo {
    glm::vec3 collisionPoint;
    glm::vec3 normal;
    union {
        float penetration;
        float t;
    };

    IntersectionInfo& invert();
};*/

namespace geometry {

    struct RaycastInfo {
    public:

        RaycastInfo(const glm::vec3& intersectionPoint, const glm::vec3& normal, float timePoint);

        bool intersects() const;

        glm::vec3 getIntersectionPoint() const;

        glm::vec3 getNormal() const;

        float getTimePoint() const;

        static RaycastInfo none();

    private:

        RaycastInfo();

        bool doesIntersect;
        glm::vec3 intersectionPoint;
        glm::vec3 normal;
        float timePoint;
    };

    struct DiscreteIntersectionInfo {
    public:

        DiscreteIntersectionInfo(const std::vector<glm::vec3>& intersectionPolygon, const glm::vec3& normal, float penetration);

        bool intersects() const;

        std::vector<glm::vec3> getIntersectionPolygon() const;

        glm::vec3 getNormal() const;

        float getPenetration() const;

        DiscreteIntersectionInfo getInverse() const;

        static DiscreteIntersectionInfo none();

    private:

        DiscreteIntersectionInfo();

        bool doesIntersect;
        std::vector<glm::vec3> intersectionPolygon;
        glm::vec3 normal;
        float penetration;

    };

    DiscreteIntersectionInfo sat(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, const std::vector<glm::vec3>& axes);

    DiscreteIntersectionInfo gjk(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2);

    RaycastInfo raycastSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& spherePosition, float sphereRadius);

    glm::vec3 convexSetSupportPoint(const std::vector<glm::vec3>& convexSet, const glm::vec3& direction);

    std::vector<glm::vec3> convexSetSupportPlane(const std::vector<glm::vec3>& convexSet, const glm::vec3& direction);

    std::vector<std::array<glm::vec3, 4>> convexSetTetrahedralization(const std::vector<glm::vec3>& convexSet);

    glm::vec3 uniformConvexSetBarycenter(const std::vector<glm::vec3>& convexSet);

    glm::vec3 weightedConvexSetBarycenter(const std::vector<glm::vec3>& convexSet, const std::vector<float>& pointWeights);

    float tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d);

    float convexSetVolume(const std::vector<glm::vec3>& convexSet);

    std::vector<glm::vec3> getBoxVertices(const glm::vec3& halfLengths);

    std::vector<glm::vec3> translatePoints(const std::vector<glm::vec3>& points, const glm::vec3& translation);

    std::vector<glm::vec3> transformPoints(const std::vector<glm::vec3>& points, const glm::vec3& translation, const glm::mat3& rotation);

    std::vector<glm::vec3> transformPoints(const std::vector<glm::vec3>& points, const glm::vec3& translation, const glm::quat& rotation);

    glm::vec3 closestPointOnTriangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    glm::vec3 closestPointOnPlane(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    std::vector<glm::vec3> intersectingConvexPolygon(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, const glm::vec3& direction);
    
    std::pair<glm::vec3, glm::vec3> capsuleCenters(const glm::vec3& position, const glm::quat& rotation, float halfLength);

    // Bounding box

    using BoundingBox = std::pair<glm::vec3, glm::vec3>;

    BoundingBox sphereBoundingBox(const glm::vec3& center, float radius);

    BoundingBox capsuleBoundingBox(const glm::vec3& center1, const glm::vec3& center2, float radius);

    BoundingBox convexSetBoundingBox(const std::vector<glm::vec3>& convexSet);

    // Intersection

    DiscreteIntersectionInfo AABBAABBIntersection(const glm::vec3& position1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::vec3 halfLength2);

    DiscreteIntersectionInfo boxBoxIntersection(const glm::vec3& position1, const glm::quat& rotation1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::quat& rotation2, const glm::vec3 halfLength2);

    DiscreteIntersectionInfo boxSphereIntersection(const glm::vec3& position1, const glm::quat& rotation, const glm::vec3& halfLength, const glm::vec3& position2, float radius);

    DiscreteIntersectionInfo boxCapsuleIntersection(const glm::vec3& position1, const glm::quat& rotation, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::quat& rotation2, float halfLength2, float radius);

    DiscreteIntersectionInfo pointSphereIntersection(const glm::vec3& pointPosition, const glm::vec3& center, float radius);

    DiscreteIntersectionInfo sphereSphereIntersection(const glm::vec3& center1, float radius1, const glm::vec3& center2, float radius2);

    DiscreteIntersectionInfo sphereCapsuleIntersection(const glm::vec3& position1, float radius1, const glm::vec3& position2, const glm::quat& rotation, float halfLength, float radius2);

    DiscreteIntersectionInfo convexConvexIntersection(const glm::vec3& position1, const glm::quat& rotation1, const std::vector<glm::vec3>& vertices1, const glm::vec3& position2, const glm::quat& rotation2, const std::vector<glm::vec3>& vertices2);
}