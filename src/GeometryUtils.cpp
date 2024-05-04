#include "GeometryUtils.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <deque>
#include <stdexcept>

/*IntersectionInfo& IntersectionInfo::invert() {
    normal = -normal;
    return *this;
}*/

namespace geometry {

    RaycastInfo::RaycastInfo(
        const glm::vec3& intersectionPoint,
        const glm::vec3& normal,
        float timePoint
    ) :
        doesIntersect(true),
        intersectionPoint(intersectionPoint),
        normal(normal),
        timePoint(timePoint)
    {}

    RaycastInfo::RaycastInfo() :
        doesIntersect(false)
    {}

    bool RaycastInfo::intersects() const {
        return doesIntersect;
    }

    glm::vec3 RaycastInfo::getIntersectionPoint() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the intersection point of a non-intersecting ray cast");
        }
        return intersectionPoint;
    }

    glm::vec3 RaycastInfo::getNormal() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the normal direction of a non-intersecting ray cast");
        }
        return normal;
    }

    float RaycastInfo::getTimePoint() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the time point of a non-intersecting ray cast");
        }
        return timePoint;
    }

    RaycastInfo RaycastInfo::none() {
        return RaycastInfo();
    }

    DiscreteIntersectionInfo::DiscreteIntersectionInfo(const std::vector<glm::vec3>& intersectionPolygon, const glm::vec3& normal, float penetration) :
        doesIntersect(true),
        intersectionPolygon(intersectionPolygon),
        normal(normal),
        penetration(penetration)
    {}

    DiscreteIntersectionInfo::DiscreteIntersectionInfo() :
        doesIntersect(false)
    {}

    bool DiscreteIntersectionInfo::intersects() const {
        return doesIntersect;
    }

    std::vector<glm::vec3> DiscreteIntersectionInfo::getIntersectionPolygon() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the intersection polygon of a non-intersecting test");
        }
        return intersectionPolygon;
    }

    glm::vec3 DiscreteIntersectionInfo::getNormal() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the normal direction of a non-intersecting test");
        }
        return normal;
    }

    float DiscreteIntersectionInfo::getPenetration() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the penetration distance of a non-intersecting test");
        }
        return penetration;
    }

    DiscreteIntersectionInfo DiscreteIntersectionInfo::getInverse() const {
        return doesIntersect ? DiscreteIntersectionInfo(intersectionPolygon, normal, penetration) : DiscreteIntersectionInfo();
    }

    DiscreteIntersectionInfo DiscreteIntersectionInfo::none() {
        return DiscreteIntersectionInfo();
    }
    
    DiscreteIntersectionInfo sat(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, const std::vector<glm::vec3>& axes) {
        glm::vec3 normal;
        float penetration = std::numeric_limits<float>::infinity();

        for (const glm::vec3& axis : axes) {
            float min1 = std::numeric_limits<float>::infinity();
            float max1 = -std::numeric_limits<float>::infinity();

            for (const glm::vec3& point : points1) {
                float projection = glm::dot(axis, point);
                if (projection < min1) {
                    min1 = projection;
                }
                else if (projection > max1) {
                    max1 = projection;
                }
            }

            float min2 = std::numeric_limits<float>::infinity();
            float max2 = -std::numeric_limits<float>::infinity();

            for (const glm::vec3& point : points2) {
                float projection = glm::dot(axis, point);
                if (projection < min2) {
                    min2 = projection;
                }
                else if (projection > max2) {
                    max2 = projection;
                }
            }

            float overlap = std::min(max1, max2) - std::max(min1, min2);
            if (overlap < 0.0f) {
                return DiscreteIntersectionInfo::none();
            } else if (overlap < penetration) {
                penetration = overlap;
                normal = min1 + max1 > min2 + max2 ? axis : -axis;
            }
        }

        return DiscreteIntersectionInfo(
            geometry::translatePoints(geometry::intersectingConvexPolygon(points1, points2, -normal), -normal * 0.5f * penetration),
            normal,
            penetration
        );
    }

    static bool gjkNearestSimplexLine(std::deque<glm::vec3>& simplex, glm::vec3& direction) {
        glm::vec3 a = simplex[0];
        glm::vec3 b = simplex[1];
        glm::vec3 ab = b - a;
        glm::vec3 ao = -a;

        if (glm::dot(ab, ao) > 0.0f) {
            direction = glm::cross(ab, glm::cross(ao, ab));
        }
        else {
            simplex = {a};
            direction = ao;
        }

        return false;
    }

    static bool gjkNearestSimplexTriangle(std::deque<glm::vec3>& simplex, glm::vec3& direction) {
        glm::vec3 a = simplex[0];
        glm::vec3 b = simplex[1];
        glm::vec3 c = simplex[2];
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ao = -a;

        glm::vec3 abc = glm::cross(ab, ac);
        
        if (glm::dot(glm::cross(abc, ac), ao) > 0.0f) {
            if (glm::dot(ac, ao) > 0.0f) {
                simplex = {a, c};
                direction = glm::cross(cross(ac, ao), ac);
            }

            else {
                simplex = {a, b};
                return gjkNearestSimplexLine(simplex, direction);
            }
        }
        else {
            if (glm::dot(glm::cross(ab, abc), ao) > 0.0f) {
                simplex = {a, b};
                return gjkNearestSimplexLine(simplex, direction);
            }

            else {
                if (glm::dot(abc, ao) > 0.0f) {
                    direction = abc;
                }

                else {
                    simplex = {a, c, b};
                    direction = -abc;
                }
            }
        }

        return false;
    }

    static bool gjkNearestSimplexTetrahedron(std::deque<glm::vec3>& simplex, glm::vec3& direction) {
        glm::vec3 a = simplex[0];
        glm::vec3 b = simplex[1];
        glm::vec3 c = simplex[2];
        glm::vec3 d = simplex[3];

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ad = d - a;
        glm::vec3 ao = -a;
    
        glm::vec3 abc = glm::cross(ab, ac);
        glm::vec3 acd = glm::cross(ac, ad);
        glm::vec3 adb = glm::cross(ad, ab);
    
        if (glm::dot(abc, ao) > 0.0f) {
            simplex = {a, b, c};
            return gjkNearestSimplexTriangle(simplex, direction);
        }
            
        if (glm::dot(acd, ao) > 0.0f) {
            simplex = {a, c, d};
            return gjkNearestSimplexTriangle(simplex, direction);
        }
    
        if (glm::dot(adb, ao) > 0.0f) {
            simplex = {a, d, b};
            return gjkNearestSimplexTriangle(simplex, direction);
        }

        return true;
    }

    static bool gjkNearestSimplex(std::deque<glm::vec3>& nearestSimplex, glm::vec3& direction) {
        switch (nearestSimplex.size()) {
            case 2 :
                return gjkNearestSimplexLine(nearestSimplex, direction);
            case 3 :
                return gjkNearestSimplexTriangle(nearestSimplex, direction);
            case 4 :
                return gjkNearestSimplexTetrahedron(nearestSimplex, direction);
            default:
                return false;
        }
    }

    static std::pair<std::vector<glm::vec4>, std::size_t> gjkGetFaceNormals(const std::deque<glm::vec3>& polytope, const std::vector<std::size_t>& faces) {
        std::vector<glm::vec4> normals;
        std::size_t minTriangle = 0;
        float minDistance = std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < faces.size(); i += 3) {
            glm::vec3 a = polytope[faces[i]];
            glm::vec3 b = polytope[faces[i + 1]];
            glm::vec3 c = polytope[faces[i + 2]];

            glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
            float distance = glm::dot(normal, a);

            if (distance < 0.0f) {
                normal = -normal;
                distance = -distance;
            }

            normals.emplace_back(normal, distance);

            if (distance < minDistance) {
                minTriangle = i / 3;
                minDistance = distance;
            }
        }

        return {normals, minTriangle};
    }

    static void gjkAddUnique(std::vector<std::pair<std::size_t, std::size_t>>& edges, const std::vector<size_t>& faces, std::size_t a, std::size_t b) {
        auto reverse = std::find(
            edges.begin(),
            edges.end(),
            std::make_pair(faces[b], faces[a])
        );
    
        if (reverse != edges.end()) {
            edges.erase(reverse);
        }
    
        else {
            edges.emplace_back(faces[a], faces[b]);
        }
    }

    static DiscreteIntersectionInfo gjkEpa(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, std::deque<glm::vec3>& simplex) {
        std::vector<std::size_t> faces = {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2
        };

        auto [normals, minFace] = gjkGetFaceNormals(simplex, faces);

        glm::vec3 minNormal;
        float minDistance = std::numeric_limits<float>::max();
        
        while (minDistance == std::numeric_limits<float>::max()) {
            minNormal = glm::vec3(normals[minFace]);
            minDistance = normals[minFace].w;
    
            glm::vec3 support = geometry::convexSetSupportPoint(points1, minNormal) - geometry::convexSetSupportPoint(points2, -minNormal);
    
            if (std::abs(glm::dot(minNormal, support) - minDistance) > 0.001f) {
                minDistance = std::numeric_limits<float>::max();

                std::vector<std::pair<std::size_t, std::size_t>> uniqueEdges;

                for (std::size_t i = 0; i < normals.size(); ++i) {
                    if (glm::dot(glm::vec3(normals[i]), support) > glm::dot(glm::vec3(normals[i]), simplex[faces[i*3]])) {
                        std::size_t f = i * 3;

                        gjkAddUnique(uniqueEdges, faces, f, f + 1);
                        gjkAddUnique(uniqueEdges, faces, f + 1, f + 2);
                        gjkAddUnique(uniqueEdges, faces, f + 2, f);

                        faces[f + 2] = faces.back();
                        faces.pop_back();
                        faces[f + 1] = faces.back();
                        faces.pop_back();
                        faces[f] = faces.back();
                        faces.pop_back();

                        normals[i] = normals.back();
                        normals.pop_back();

                        --i;
                    }
                }

                std::vector<std::size_t> newFaces;
                for (auto [edgeIndex1, edgeIndex2] : uniqueEdges) {
                    newFaces.push_back(edgeIndex1);
                    newFaces.push_back(edgeIndex2);
                    newFaces.push_back(simplex.size());
                }
                
                simplex.push_back(support);

                auto [newNormals, newMinFace] = gjkGetFaceNormals(simplex, newFaces);

                float oldMinDistance = std::numeric_limits<float>::max();
                for (std::size_t i = 0; i < normals.size(); i++) {
                    if (normals[i].w < oldMinDistance) {
                        oldMinDistance = normals[i].w;
                        minFace = i;
                    }
                }
    
                if (newNormals[newMinFace].w < oldMinDistance) {
                    minFace = newMinFace + normals.size();
                }
    
                faces.insert(faces.end(), newFaces.begin(), newFaces.end());
                normals.insert(normals.end(), newNormals.begin(), newNormals.end());
            }
        }

        return DiscreteIntersectionInfo(
            geometry::translatePoints(geometry::intersectingConvexPolygon(points1, points2, minNormal), minNormal * 0.5f * minDistance),
            -minNormal,
            minDistance
        );
    }

    DiscreteIntersectionInfo gjk(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2) {
        glm::vec3 supportPoint = geometry::convexSetSupportPoint(points1, glm::vec3(1.0f, 0.0f, 0.0f)) - geometry::convexSetSupportPoint(points2, glm::vec3(-1.0f, 0.0f, 0.0f));
        std::deque<glm::vec3> simplex = {supportPoint};
        glm::vec3 direction = -supportPoint;
        
        while (true) {
            supportPoint = geometry::convexSetSupportPoint(points1, direction) - geometry::convexSetSupportPoint(points2, -direction);
            if (glm::dot(supportPoint, direction) < 0.0f) {
                return DiscreteIntersectionInfo::none();
            }
            
            simplex.push_front(supportPoint);

            if (gjkNearestSimplex(simplex, direction)) {
                return gjkEpa(points1, points2, simplex);
            }
        }
    }

    glm::vec3 closestPointOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        const glm::vec3 ab = b - a;
        const glm::vec3 ac = c - a;
        const glm::vec3 ap = p - a;

        const float acac = glm::dot(ac, ac);
        const float acap = glm::dot(ac, ap);
        const float abab = glm::dot(ab, ab);
        const float abac = glm::dot(ab, ac);
        const float abap = glm::dot(ab, ap);

        const float d = 1.0f / (acac * abab - abac * abac);
        const float u = std::clamp((abab * acap - abac * abap) * d, 0.0f, 1.0f);
        const float v = std::clamp((acac * abap - abac * acap) * d, 0.0f, 1.0f);

        return a + u * ac + v * ab;
    }

    RaycastInfo raycastSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& spherePosition, float sphereRadius) {
        return RaycastInfo::none();
    }

    glm::vec3 convexSetSupportPoint(const std::vector<glm::vec3>& convexSet, const glm::vec3& direction) {
        float maxDot = -std::numeric_limits<float>::infinity();
        glm::vec3 supportPoint;
        for (const glm::vec3& point : convexSet) {
            float dot = glm::dot(point, direction);
            if (dot > maxDot) {
                maxDot = dot;
                supportPoint = point;
            }
        }
        return supportPoint;
    }

    std::vector<glm::vec3> convexSetSupportPlane(const std::vector<glm::vec3>& convexSet, const glm::vec3& direction) {
        float maxDot = -std::numeric_limits<float>::infinity();
        std::vector<glm::vec3> supportPlane;
        for (const glm::vec3& point : convexSet) {
            float dot = glm::dot(point, direction);
            if (dot > maxDot) {
                maxDot = dot;
            }
        }

        for (const glm::vec3& point : convexSet) {
            if (glm::dot(point, direction) == maxDot) {
                supportPlane.push_back(point);
            }
        }
        return supportPlane;
    }

    // todo tetrahedralization
    std::vector<std::array<glm::vec3, 4>> convexSetTetrahedralization(const std::vector<glm::vec3>& convexSet) {
        if (convexSet.size() < 4) {
            throw std::runtime_error("Convex set of size less than 4 (" + std::to_string(convexSet.size()) + ") cannot be broken down into tetrahedra");
        }

        std::vector<std::array<glm::vec3, 4>> tetrahedra;
        /*for (std::size_t i = 1; i < convexSet.size() - 2; ++i) {
            for (std::size_t j = i + 1; j < convexSet.size() - 1; ++j) {
                for (std::size_t k = j + 1; k < convexSet.size(); ++k) {
                    tetrahedra.push_back({convexSet[0], convexSet[i], convexSet[j], convexSet[k]});
                }
            }
        }*/

        return tetrahedra;
    }

    glm::vec3 uniformConvexSetBarycenter(const std::vector<glm::vec3>& convexSet) {
        glm::vec3 barycenter(0.0f);

        for (const glm::vec3& point : convexSet) {
            barycenter += point;
        }

        return barycenter / static_cast<float>(convexSet.size());
    }

    glm::vec3 weightedConvexSetBarycenter(const std::vector<glm::vec3>& convexSet, const std::vector<float>& pointWeights) {
        if (convexSet.size() != pointWeights.size()) {
            throw std::runtime_error("Cannot calculate the convex barycenter of unequal sizes of position and mass vectors");
        }

        float totalMass = 0.0f;
        glm::vec3 barycenter(0.0f);

        for (std::size_t i = 0; i < convexSet.size(); ++i) {
            totalMass += pointWeights[i];
            barycenter += convexSet[i] * pointWeights[i];
        }

        return barycenter / totalMass;
    }

    float tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
        return std::abs(glm::dot(a - d, glm::cross(b - d, c - d))) / 6.0f;
    }

    float convexSetVolume(const std::vector<glm::vec3>& convexSet) {
        float volume = 0.0f;
        for (auto [a, b, c, d] : convexSetTetrahedralization(convexSet)) {
            volume += tetrahedronVolume(a, b, c, d);
        }
        return volume;
    }

    std::vector<glm::vec3> getBoxVertices(const glm::vec3& halfLengths) {
        return {
            glm::vec3(-halfLengths.x, -halfLengths.y, -halfLengths.z),
            glm::vec3(-halfLengths.x, -halfLengths.y, halfLengths.z),
            glm::vec3(-halfLengths.x, halfLengths.y, -halfLengths.z),
            glm::vec3(-halfLengths.x, halfLengths.y, halfLengths.z),
            glm::vec3(halfLengths.x, -halfLengths.y, -halfLengths.z),
            glm::vec3(halfLengths.x, -halfLengths.y, halfLengths.z),
            glm::vec3(halfLengths.x, halfLengths.y, -halfLengths.z),
            glm::vec3(halfLengths.x, halfLengths.y, halfLengths.z)
        };
    }

    std::vector<glm::vec3> translatePoints(const std::vector<glm::vec3>& points, const glm::vec3& translation) {
        std::vector<glm::vec3> translatedPoints(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            translatedPoints[i] = points[i] + translation;
        }
        return translatedPoints;
    }

    std::vector<glm::vec3> transformPoints(const std::vector<glm::vec3>& points, const glm::vec3& translation, const glm::mat3& rotation) {
        if (rotation == glm::identity<glm::mat3>()) {
            return translatePoints(points, translation);
        }
        std::vector<glm::vec3> translatedPoints(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            translatedPoints[i] = rotation * points[i] + translation;
        }
        return translatedPoints;
    }

    std::vector<glm::vec3> transformPoints(const std::vector<glm::vec3>& points, const glm::vec3& translation, const glm::quat& rotation) {
        if (rotation == glm::identity<glm::quat>()) {
            return translatePoints(points, translation);
        }
        std::vector<glm::vec3> translatedPoints(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            translatedPoints[i] = rotation * points[i] + translation;
        }
        return translatedPoints;
    }

    std::pair<glm::vec3, glm::vec3> capsuleCenters(const glm::vec3& position, const glm::quat& rotation, float halfLength) {
        return std::make_pair(
            position + rotation * glm::vec3(0.0f, -halfLength, 0.0f),
            position + rotation * glm::vec3(0.0f, halfLength, 0.0f)
        );
    }

    BoundingBox sphereBoundingBox(const glm::vec3& center, float radius) {
        return std::make_pair(
            center - glm::vec3(radius),
            center + glm::vec3(radius)
        );
    }

    BoundingBox capsuleBoundingBox(const glm::vec3& center1, const glm::vec3& center2, float radius) {
        auto [minCenter1, maxCenter1] = sphereBoundingBox(center1, radius);
        auto [minCenter2, maxCenter2] = sphereBoundingBox(center2, radius);
        return std::make_pair(
            glm::vec3(std::min(minCenter1.x, minCenter2.x), std::min(minCenter1.y, minCenter2.y), std::min(minCenter1.z, minCenter2.z)),
            glm::vec3(std::max(maxCenter1.x, maxCenter2.x), std::max(maxCenter1.y, maxCenter2.y), std::max(maxCenter1.z, maxCenter2.z))
        );
    }

    BoundingBox convexSetBoundingBox(const std::vector<glm::vec3>& convexSet) {
        glm::vec3 min(std::numeric_limits<float>::infinity());
        glm::vec3 max(-std::numeric_limits<float>::infinity());
        for (const glm::vec3& point : convexSet) {
            for (int i{}; i < 3; ++i) {
                min[i] = std::min(min[i], point[i]);
                max[i] = std::max(max[i], point[i]);
            }
        }
        return std::make_pair(min, max);
    }

    glm::vec3 closestPointOnPlane(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        // todo possible without normalization
        glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
        glm::vec3 closestPoint = point - (glm::dot(normal, point) - glm::dot(normal, a)) * normal;
        return closestPoint;
    }

    std::vector<glm::vec3> intersectingConvexPolygon(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, const glm::vec3& direction) {
        std::vector<glm::vec3> supportPlane1 = convexSetSupportPlane(points1, direction);
        std::vector<glm::vec3> supportPlane2 = convexSetSupportPlane(points2, -direction);

        if (supportPlane1.size() == 1) {
            return {supportPlane1[0]};
        }
        else if (supportPlane2.size() == 1) {
            return {supportPlane2[0]};
        }

        // todo
        /*glm::vec3 sweepingDirection = glm::cross(direction, glm::vec3(1, 0, 0));
        if (sweepingDirection == glm::vec3()) {
            sweepingDirection = glm::cross(direction, glm::vec3(0, 1, 0));
        }

        const auto sweepSortingFunction = [&sweepingDirection](const glm::vec3& a, const glm::vec3& b) {
            return glm::dot(sweepingDirection, a) < glm::dot(sweepingDirection, b);
        };

        std::vector<glm::vec3> sortedPoints1 = points1;
        std::vector<glm::vec3> sortedPoints2 = points2;
        std::sort(points1.begin(), points1.end(), sweepSortingFunction);
        std::sort(points2.begin(), points2.end(), sweepSortingFunction);

        std::vector<glm::vec3> firstHalf1;
        std::vector<glm::vec3> firstHalf2;
        std::vector<glm::vec3> secondHalf1;
        std::vector<glm::vec3> secondHalf2;

        firstHalf1.push_back(points1[0]);
        secondHalf1.push_back(points1[0]);
        for (std::size_t i{1}; i < points1.size(); ++i) {
            const glm::vec3 difference{points1[i] - points1[0]};
            if (glm::dot(glm::cross(difference, sweepingDirection), difference) > 0) {
                firstHalf1.push_back(points1[i]);
            }
            else {
                secondHalf1.push_back(points1[i]);
            }
        }*/

        return {supportPlane2[0]};
    }

    DiscreteIntersectionInfo AABBAABBIntersection(const glm::vec3& position1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::vec3 halfLength2) {
        const glm::vec3 sumSize = halfLength1 + halfLength2;
        const glm::vec3 difference = glm::abs(position1 - position2);
        if (glm::all(glm::lessThanEqual(difference, sumSize))) {
            const glm::vec3 penetration{sumSize - difference};
            if (penetration.x < penetration.y && penetration.x < penetration.z) {
                return DiscreteIntersectionInfo(
                    {((halfLength1.x - 0.5f * penetration) / sumSize.x) * (position2 - position1)},
                    glm::vec3(position1.x >= position2.x ? 1.0f : -1.0f, 0.0f, 0.0f),
                    penetration.x
                );
            }
            else if (penetration.y < penetration.x && penetration.y < penetration.z) {
                return DiscreteIntersectionInfo(
                    {((halfLength1.y - 0.5f * penetration) / sumSize.y) * (position2 - position1)},
                    glm::vec3(0.0f, position1.y >= position2.y ? 1.0f : -1.0f, 0.0f),
                    penetration.y
                );
            }
            else {
                return DiscreteIntersectionInfo(
                    {((halfLength1.z - 0.5f * penetration) / sumSize.z) * (position2 - position1)},
                    glm::vec3(0.0f, 0.0f, position1.z >= position2.z ? 1.0f : -1.0f),
                    penetration.z
                );
            }
        }
        return DiscreteIntersectionInfo::none();
    }

    DiscreteIntersectionInfo boxBoxIntersection(const glm::vec3& position1, const glm::quat& rotation1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::quat& rotation2, const glm::vec3 halfLength2) {
        if (rotation1 == glm::identity<glm::quat>() && rotation2 == glm::identity<glm::quat>()) {
            return AABBAABBIntersection(position1, position2, halfLength1, halfLength2);
        }
        std::vector<glm::vec3> axes(6);
        axes[0] = rotation1 * glm::vec3(1.0f, 0.0f, 0.0f);
        axes[1] = rotation1 * glm::vec3(0.0f, 1.0f, 0.0f);
        axes[2] = rotation1 * glm::vec3(0.0f, 0.0f, 1.0f);

        axes[3] = rotation2 * glm::vec3(1.0f, 0.0f, 0.0f);
        axes[4] = rotation2 * glm::vec3(0.0f, 1.0f, 0.0f);
        axes[5] = rotation2 * glm::vec3(0.0f, 0.0f, 1.0f);

        for (int i = 0; i < 3; ++i) {
            for (int j = 3; j < 6; ++j) {
                const glm::vec3 crossAxes = glm::cross(axes[i], axes[j]);
                if (crossAxes != glm::vec3(0.0f, 0.0f, 0.0f)) {
                    axes.push_back(glm::normalize(crossAxes));
                }
            }
        }

        return sat(
            geometry::transformPoints(geometry::getBoxVertices(halfLength1), position1, rotation1),
            geometry::transformPoints(geometry::getBoxVertices(halfLength2), position2, rotation2),
            axes
        );
    }

    DiscreteIntersectionInfo boxSphereIntersection(const glm::vec3& position1, const glm::quat& rotation, const glm::vec3& halfLength, const glm::vec3& position2, float radius) {
        std::vector<glm::vec3> axes(3);
        axes[0] = rotation * glm::vec3(halfLength.x, 0.0f, 0.0f);
        axes[1] = rotation * glm::vec3(0.0f, halfLength.y, 0.0f);
        axes[2] = rotation * glm::vec3(0.0f, 0.0f, halfLength.z);

        const glm::vec3 relativePosition = position2 - position1;
        glm::vec3 normalizedPosition;
        for (int i = 0; i < 3; ++i) {
            normalizedPosition[i] = std::clamp(glm::dot(relativePosition, axes[i]) / glm::length2(axes[i]), -1.0f, 1.0f);
        }
        const glm::vec3 normalizedDifference = glm::abs(normalizedPosition);
        if (glm::all(glm::lessThan(normalizedDifference, glm::vec3(1.0f)))) {
            // Sphere inside
            float penetration;
            glm::vec3 normal;
            if (normalizedDifference[0] > normalizedDifference[1] && normalizedDifference[0] > normalizedDifference[2]) {
                penetration = (1.0f - normalizedDifference[0]) * halfLength.x;
                normal = axes[0] / halfLength.x;
            }
            else if (normalizedDifference[1] > normalizedDifference[0] && normalizedDifference[1] > normalizedDifference[2]) {
                penetration = (1.0f - normalizedDifference[1]) * halfLength.y;
                normal = axes[1] / halfLength.y;
            }
            else {
                penetration = (1.0f - normalizedDifference[2]) * halfLength.z;
                normal = axes[2] / halfLength.z;
            }
            return DiscreteIntersectionInfo(
                {position2 + 0.5f * penetration * normal},
                normal,
                penetration
            );
        }
        else {
            // Sphere outside
            glm::vec3 closestPointPosition(0.0f);
            for (int i = 0; i < 3; ++i) {
                closestPointPosition += axes[i] * normalizedPosition[i];
            }
            return geometry::pointSphereIntersection(closestPointPosition + position1, position2, radius);
        }
    }

    DiscreteIntersectionInfo boxCapsuleIntersection(const glm::vec3& position1, const glm::quat& rotation, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::quat& rotation2, float halfLength2, float radius) {
        //return boxSphereIntersection()
        return DiscreteIntersectionInfo::none();
    }

    DiscreteIntersectionInfo pointSphereIntersection(const glm::vec3& pointPosition, const glm::vec3& center, float radius) {
        return sphereSphereIntersection(pointPosition, 0.0f, center, radius);
    }

    DiscreteIntersectionInfo sphereSphereIntersection(const glm::vec3& center1, float radius1, const glm::vec3& center2, float radius2) {
        const float radiiSum{radius1 + radius2};
        const float distanceSquared = glm::distance2(center1, center2);
        if (distanceSquared <= radiiSum * radiiSum) {
            const glm::vec3 normal = glm::normalize(center1 - center2);
            const float penetration = radiiSum - std::sqrt(distanceSquared);
            return DiscreteIntersectionInfo(
                {center2 + normal * (radius2 - 0.5f * penetration)},
                normal,
                penetration
            );
        }
        return DiscreteIntersectionInfo::none();
    }
    
    DiscreteIntersectionInfo sphereCapsuleIntersection(const glm::vec3& position1, float radius1, const glm::vec3& position2, const glm::quat& rotation, float halfLength, float radius2) {
        auto [capsuleCenter1, capsuleCenter2] = capsuleCenters(position2, rotation, halfLength);
        const glm::vec3 lineDifference = capsuleCenter2 - capsuleCenter1;
        const glm::vec3 pointDifference = position2 - position1;
        const float linePosition = std::clamp(glm::dot(lineDifference, pointDifference) / glm::length2(lineDifference), 0.0f, 1.0f);
        const glm::vec3 pointPosition = linePosition * lineDifference + capsuleCenter1;
        return sphereSphereIntersection(pointPosition, radius2, position1, radius1);
    }

    DiscreteIntersectionInfo convexConvexIntersection(const glm::vec3& position1, const glm::quat& rotation1, const std::vector<glm::vec3>& vertices1, const glm::vec3& position2, const glm::quat& rotation2, const std::vector<glm::vec3>& vertices2) {
        return gjk(
            transformPoints(vertices1, position1, rotation1),
            transformPoints(vertices2, position2, rotation2)
        );
    }
}