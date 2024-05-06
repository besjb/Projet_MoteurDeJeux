#include "GeometryUtils.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/matrix_operation.hpp>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <deque>
#include <stdexcept>

#include <Mesh.hpp>
#include <Scene.hpp>
extern Mesh* globalEarthMesh;
extern Scene* scenePointer;

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

    DiscreteIntersectionInfo::DiscreteIntersectionInfo(const glm::vec3& intersectionPoint, const glm::vec3& normal, float penetration) :
        doesIntersect(true),
        intersectionPoint(intersectionPoint),
        normal(normal),
        penetration(penetration)
    {}

    DiscreteIntersectionInfo::DiscreteIntersectionInfo() :
        doesIntersect(false)
    {}

    bool DiscreteIntersectionInfo::intersects() const {
        return doesIntersect;
    }

    glm::vec3 DiscreteIntersectionInfo::getIntersectionPoint() const {
        if (!doesIntersect) {
            throw std::runtime_error("Cannot get the intersection point of a non-intersecting test");
        }
        return intersectionPoint;
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
        return doesIntersect ? DiscreteIntersectionInfo(intersectionPoint, normal, penetration) : DiscreteIntersectionInfo();
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
            glm::vec3(0.0f, 0.0f, 0.0f),//translatePoints(intersectingConvexPolygon(points1, points2, -normal), -normal * 0.5f * penetration),
            normal,
            penetration
        );
    }

    std::function<bool(const glm::vec3&, const glm::vec3&)> sweepSortingFunction(const glm::vec3& direction) {
        return [&direction](const glm::vec3& a, const glm::vec3& b) {
            return glm::dot(direction, a) < glm::dot(direction, b);
        };
    }

    static bool gjkNearestSimplexLine(std::deque<std::array<glm::vec3, 3>>& simplex, glm::vec3& direction) {
        std::array<glm::vec3, 3> sa = simplex[0];
        std::array<glm::vec3, 3> sb = simplex[1];
        glm::vec3 a = sa[0];
        glm::vec3 b = sb[0];
        glm::vec3 ab = b - a;
        glm::vec3 ao = -a;

        if (glm::dot(ab, ao) > 0.0f) {
            direction = glm::cross(glm::cross(ab, ao), ab);
        }
        else {
            simplex = {sa};
            direction = ao;
        }

        return false;
    }

    static bool gjkNearestSimplexTriangle(std::deque<std::array<glm::vec3, 3>>& simplex, glm::vec3& direction) {
        std::array<glm::vec3, 3> sa = simplex[0];
        std::array<glm::vec3, 3> sb = simplex[1];
        std::array<glm::vec3, 3> sc = simplex[2];
        glm::vec3 a = sa[0];
        glm::vec3 b = sb[0];
        glm::vec3 c = sc[0];
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ao = -a;

        glm::vec3 abc = glm::cross(ab, ac);
        
        if (glm::dot(glm::cross(abc, ac), ao) > 0.0f) {
            if (glm::dot(ac, ao) > 0.0f) {
                simplex = {sa, sc};
                direction = glm::cross(glm::cross(ac, ao), ac);
            }

            else {
                simplex = {sa, sb};
                return gjkNearestSimplexLine(simplex, direction);
            }
        }
        else {
            if (glm::dot(glm::cross(ab, abc), ao) > 0.0f) {
                simplex = {sa, sb};
                return gjkNearestSimplexLine(simplex, direction);
            }

            else {
                if (glm::dot(abc, ao) > 0.0f) {
                    direction = abc;
                }

                else {
                    simplex = {sa, sc, sb};
                    direction = -abc;
                }
            }
        }

        return false;
    }

    static bool gjkNearestSimplexTetrahedron(std::deque<std::array<glm::vec3, 3>>& simplex, glm::vec3& direction) {
        std::array<glm::vec3, 3> sa = simplex[0];
        std::array<glm::vec3, 3> sb = simplex[1];
        std::array<glm::vec3, 3> sc = simplex[2];
        std::array<glm::vec3, 3> sd = simplex[3];
        glm::vec3 a = sa[0];
        glm::vec3 b = sb[0];
        glm::vec3 c = sc[0];
        glm::vec3 d = sd[0];

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ad = d - a;
        glm::vec3 ao = -a;
    
        glm::vec3 abc = glm::cross(ab, ac);
        glm::vec3 acd = glm::cross(ac, ad);
        glm::vec3 adb = glm::cross(ad, ab);
    
        if (glm::dot(abc, ao) > 0.0f) {
            simplex = {sa, sb, sc};
            return gjkNearestSimplexTriangle(simplex, direction);
        }
            
        if (glm::dot(acd, ao) > 0.0f) {
            simplex = {sa, sc, sd};
            return gjkNearestSimplexTriangle(simplex, direction);
        }
    
        if (glm::dot(adb, ao) > 0.0f) {
            simplex = {sa, sd, sb};
            return gjkNearestSimplexTriangle(simplex, direction);
        }

        return true;
    }

    static bool gjkNearestSimplex(std::deque<std::array<glm::vec3, 3>>& nearestSimplex, glm::vec3& direction) {
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

    static std::pair<std::vector<glm::vec4>, std::size_t> gjkGetFaceNormals(std::deque<std::array<glm::vec3, 3>>& polytope, const std::vector<std::size_t>& faces) {
        std::vector<glm::vec4> normals;
        std::size_t minTriangle = 0;
        float minDistance = std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < faces.size(); i += 3) {
            glm::vec3 a = polytope[faces[i]][0];
            glm::vec3 b = polytope[faces[i + 1]][0];
            glm::vec3 c = polytope[faces[i + 2]][0];

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

    static DiscreteIntersectionInfo gjkEpa(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2, std::deque<std::array<glm::vec3, 3>>& simplex) {
        std::vector<std::size_t> faces = {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2
        };

        auto [normals, minFace] = gjkGetFaceNormals(simplex, faces);

        glm::vec3 minNormal;
        float minDistance = std::numeric_limits<float>::max();
        
        do {
            minNormal = glm::vec3(normals[minFace]);
            minDistance = normals[minFace].w;

            glm::vec3 supportPoint1 = convexSetSupportPoint(points1, minNormal);
            glm::vec3 supportPoint2 = convexSetSupportPoint(points2, -minNormal);
            glm::vec3 support = supportPoint1 - supportPoint2;

            if (std::abs(glm::dot(minNormal, support) - minDistance) > 0.001f) {
                minDistance = std::numeric_limits<float>::max();

                std::vector<std::pair<std::size_t, std::size_t>> uniqueEdges;

                for (std::size_t i = 0; i < normals.size(); ++i) {
                    if (glm::dot(glm::vec3(normals[i]), support) > glm::dot(glm::vec3(normals[i]), simplex[faces[i*3]][0])) {
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
                
                simplex.push_back({support, supportPoint1, supportPoint2});

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
        } while (minDistance == std::numeric_limits<float>::max());

        const auto [u, v, w] = barycentricCoordinates(glm::vec3(), simplex[faces[3 * minFace]][0], simplex[faces[3 * minFace + 1]][0], simplex[faces[3 * minFace + 2]][0]);
        //std::cout << "u " << u << " v " << v << " w " << w << '\n';
        //const glm::vec3 collisionPoint1 = u * simplex[faces[3 * minFace]][1] + v * simplex[faces[3 * minFace + 1]][1] + w * simplex[faces[3 * minFace + 2]][1];
        const glm::vec3 collisionPoint2 = u * simplex[faces[3 * minFace]][2] + v * simplex[faces[3 * minFace + 1]][2] + w * simplex[faces[3 * minFace + 2]][2];

        return DiscreteIntersectionInfo(
            collisionPoint2,
            //minNormal * glm::dot(simplex[faces[3 * minFace]], minNormal),
            -minNormal,
            minDistance
        );
    }

    DiscreteIntersectionInfo gjk(const std::vector<glm::vec3>& points1, const std::vector<glm::vec3>& points2) {
        // TODO heuristique (vélocité au lieu d'un vecteur quelconque)
        glm::vec3 supportPoint1 = convexSetSupportPoint(points1, glm::vec3(1.0f, 0.0f, 0.0f));
        glm::vec3 supportPoint2 = convexSetSupportPoint(points2, glm::vec3(-1.0f, 0.0f, 0.0f));
        glm::vec3 support = supportPoint1 - supportPoint2;
        std::deque<std::array<glm::vec3, 3>> simplex = {{support, supportPoint1, supportPoint2}};
        glm::vec3 direction = -support;
        
        // TODO I don't know why it crashes, but ignore faulty collisions for now... Set while(true) when fixed and remove i
        int i = 0;
        while (i++ < 20) {
            supportPoint1 = convexSetSupportPoint(points1, direction);
            supportPoint2 = convexSetSupportPoint(points2, -direction);
            support = supportPoint1 - supportPoint2;
            if (glm::dot(support, direction) < 0.0f) {
                return DiscreteIntersectionInfo::none();
            }
            
            simplex.push_front({support, supportPoint1, supportPoint2});

            if (gjkNearestSimplex(simplex, direction)) {
                return gjkEpa(points1, points2, simplex);
            }
        }

        return DiscreteIntersectionInfo::none(); 
    }

    glm::vec3 closestPointOnLine(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, bool closed) {
        glm::vec3 ab = b - a;
        const float t = glm::dot(point - a, ab) / glm::dot(ab, ab);
        return a + (closed ? std::clamp(t, 0.0f, 1.0f) : t);
    }

    std::pair<glm::vec3, glm::vec3> closestPointsOnLines(const glm::vec3& a1, const glm::vec3& a2, const glm::vec3& b1, const glm::vec3& b2, bool closed1, bool closed2) {
        const glm::vec3 d = b1 - a1;
        const glm::vec3 u = a2 - a1;
        const glm::vec3 v = b2 - b1;

        const float du = glm::dot(d, u);
        const float dv = glm::dot(d, v);
        const float uu = glm::dot(u, u);
        const float uv = glm::dot(u, v);
        const float vv = glm::dot(v, v);

        const float det = uu * vv - uv * uv;

        float s;
        float t;

        if (det < glm::epsilon<float>()) {
            s = du / uu;
            t = 0.0f;
        }
        else {
            const float invDet = 1.0f / det;
            s = (du * vv - dv * uv) * invDet;
            t = (du * uv - dv * uu) * invDet;
        }

        if (closed1) {
            s = std::clamp(s, 0.0f, 1.0f);
        }
        if (closed2) {
            t = std::clamp(t, 0.0f, 1.0f);
        }

        float sp = (t * uv + du) / uu;
        float tp = (s * uv - dv) / vv;

        if (closed1) {
            sp = std::clamp(sp, 0.0f, 1.0f);
        }
        if (closed2) {
            tp = std::clamp(tp, 0.0f, 1.0f);
        }

        return {a1 + sp * u, b1 + tp * v};
    }

    glm::vec3 closestPointOnTriangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        const auto [u, v, w] = barycentricCoordinates(point, a, b, c);
        return u * a + v * b + w * c;
    }

    RaycastInfo raycastSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& spherePosition, float sphereRadius) {
        return RaycastInfo::none();
    }

    glm::vec3 arbitraryOrthogonal(const glm::vec3& direction) {
        return glm::cross(direction, direction.x == 0.0f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f));
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

    std::array<glm::vec3, 3> convexSetSupportTriangle(const std::vector<glm::vec3>& convexSet, const glm::vec3& direction) {
        float maxDot[3] = {-std::numeric_limits<float>::infinity()};
        std::array<glm::vec3, 3> supportPoint;
        for (const glm::vec3& point : convexSet) {
            float dot = glm::dot(point, direction);
            for (int i = 0; i < 3; ++i) {
                if (dot > maxDot[i]) {
                    maxDot[i] = dot;
                    supportPoint[i] = point;
                    break;
                }
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

    std::vector<std::array<glm::vec3, 4>> convexSetTetrahedralization(const std::vector<glm::vec3>& convexSet) {
        if (convexSet.size() < 4) {
            throw std::runtime_error("Convex set of size less than 4 (" + std::to_string(convexSet.size()) + ") cannot be broken down into tetrahedra");
        }

        std::vector<std::array<glm::vec3, 4>> tetrahedra;
        std::vector<glm::vec3> sortedConvexSet = convexSet;
        std::sort(sortedConvexSet.begin(), sortedConvexSet.end(), sweepSortingFunction(glm::vec3(1.0f, 0.0f, 0.0f)));
        for (std::size_t i = 1; i < sortedConvexSet.size() - 2; ++i) {
            tetrahedra.push_back({sortedConvexSet[0], sortedConvexSet[i], sortedConvexSet[i + 1], sortedConvexSet[i + 2]});
        }
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
        std::vector<glm::vec3> transformedPoints(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            transformedPoints[i] = rotation * points[i] + translation;
        }
        return transformedPoints;
    }

    std::pair<glm::vec3, glm::vec3> capsuleHemispheres(const glm::vec3& position, const glm::quat& rotation, float halfHeight) {
        return std::make_pair(
            position + rotation * glm::vec3(0.0f, -halfHeight, 0.0f),
            position + rotation * glm::vec3(0.0f, halfHeight, 0.0f)
        );
    }

    std::array<float, 3> barycentricCoordinates(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        const glm::vec3 d = point - a;
        const glm::vec3 u = b - a;
        const glm::vec3 v = c - a;

        const float du = glm::dot(d, u);
        const float dv = glm::dot(d, v);
        const float uu = glm::dot(u, u);
        const float uv = glm::dot(u, v);
        const float vv = glm::dot(v, v);

        const float invDet = 1.0f / (vv * uu - uv * uv);
        const float cu = std::clamp((uu * dv - uv * du) * invDet, 0.0f, 1.0f);
        const float cv = std::clamp((vv * du - uv * dv) * invDet, 0.0f, 1.0f);
        const float cw = std::clamp(1.0f - cu - cv, 0.0f, 1.0f);

        return {cu, cv, cw};
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
        glm::vec3 normal = glm::cross(b - a, c - a);
        return point - (glm::dot(normal, point) - glm::dot(normal, a)) * normal;
    }

    DiscreteIntersectionInfo AABBAABBIntersection(const glm::vec3& position1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::vec3 halfLength2) {
        const glm::vec3 sumSize = halfLength1 + halfLength2;
        const glm::vec3 difference = glm::abs(position1 - position2);
        if (glm::all(glm::lessThanEqual(difference, sumSize))) {
            const glm::vec3 penetration{sumSize - difference};
            if (penetration.x < penetration.y && penetration.x < penetration.z) {
                const glm::vec3 normal = glm::vec3(position1.x >= position2.x ? 1.0f : -1.0f, 0.0f, 0.0f);

                return DiscreteIntersectionInfo(
                    glm::vec3(
                        position1.x - halfLength1.x * normal.x - 0.5f * penetration.x,
                        0.5f * (std::max(position1.y - halfLength1.y, position2.y - halfLength2.y) + std::min(position1.y + halfLength1.y, position2.y + halfLength2.y)),
                        0.5f * (std::max(position1.z - halfLength1.z, position2.z - halfLength2.z) + std::min(position1.z + halfLength1.z, position2.z + halfLength2.z))
                    ),
                    normal,
                    penetration.x
                );
            }
            else if (penetration.y < penetration.x && penetration.y < penetration.z) {
                const glm::vec3 normal = glm::vec3(0.0f, position1.y >= position2.y ? 1.0f : -1.0f, 0.0f);

                return DiscreteIntersectionInfo(
                    glm::vec3(
                        0.5f * (std::max(position1.x - halfLength1.x, position2.x - halfLength2.x) + std::min(position1.x + halfLength1.x, position2.x + halfLength2.x)),
                        position1.y - halfLength1.y * normal.y - 0.5f * penetration.y,
                        0.5f * (std::max(position1.z - halfLength1.z, position2.z - halfLength2.z) + std::min(position1.z + halfLength1.z, position2.z + halfLength2.z))
                    ),
                    normal,
                    penetration.y
                );
            }
            else {
                const glm::vec3 normal = glm::vec3(0.0f, 0.0f, position1.z >= position2.z ? 1.0f : -1.0f);

                return DiscreteIntersectionInfo(
                    glm::vec3(
                        0.5f * (std::max(position1.x - halfLength1.x, position2.x - halfLength2.x) + std::min(position1.x + halfLength1.x, position2.x + halfLength2.x)),
                        0.5f * (std::max(position1.y - halfLength1.z, position2.y - halfLength2.y) + std::min(position1.y + halfLength1.y, position2.y + halfLength2.y)),
                        position1.z - halfLength1.z * normal.z - 0.5f * penetration.z
                    ),
                    normal,
                    penetration.z
                );
            }
        }
        return DiscreteIntersectionInfo::none();
    }

    DiscreteIntersectionInfo boxBoxIntersection(const glm::vec3& position1, const glm::quat& rotation1, const glm::vec3& halfLength1, const glm::vec3& position2, const glm::quat& rotation2, const glm::vec3 halfLength2) {
        if (rotation1 == glm::identity<glm::quat>() && rotation2 == glm::identity<glm::quat>()) {
            return AABBAABBIntersection(position1, halfLength1, position2, halfLength2);
        }

        return gjk(
            transformPoints(getBoxVertices(halfLength1), position1, rotation1),
            transformPoints(getBoxVertices(halfLength2), position2, rotation2)
        );

        /*std::vector<glm::vec3> axes(6);
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
            transformPoints(getBoxVertices(halfLength1), position1, rotation1),
            transformPoints(getBoxVertices(halfLength2), position2, rotation2),
            axes
        );*/
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
                position2 + 0.5f * penetration * normal,
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
            return pointSphereIntersection(closestPointPosition + position1, position2, radius);
        }
    }

    DiscreteIntersectionInfo boxCapsuleIntersection(const glm::vec3& position1, const glm::quat& rotation1, const glm::vec3& halfLength, const glm::vec3& position2, const glm::quat& rotation2, float halfHeight, float radius) {
        // TODO Il existe peut être d'autres approches plus simples pour des boîtes...
        return capsuleConvexIntersection(position2, rotation2, halfHeight, radius, position1, rotation1, getBoxVertices(halfLength)).getInverse();
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
                center2 + normal * (radius2 - 0.5f * penetration),
                normal,
                penetration
            );
        }
        return DiscreteIntersectionInfo::none();
    }
    
    DiscreteIntersectionInfo sphereCapsuleIntersection(const glm::vec3& position1, float radius1, const glm::vec3& position2, const glm::quat& rotation, float halfLength, float radius2) {
        auto [capsuleCenter1, capsuleCenter2] = capsuleHemispheres(position2, rotation, halfLength);
        const glm::vec3 lineDifference = capsuleCenter2 - capsuleCenter1;
        const glm::vec3 pointDifference = position2 - position1;
        const float linePosition = std::clamp(glm::dot(lineDifference, pointDifference) / glm::length2(lineDifference), 0.0f, 1.0f);
        const glm::vec3 pointPosition = linePosition * lineDifference + capsuleCenter1;
        return sphereSphereIntersection(pointPosition, radius2, position1, radius1);
    }

    DiscreteIntersectionInfo sphereConvexIntersection(const glm::vec3& position1, float radius, const glm::vec3& position2, const glm::quat& rotation, const std::vector<glm::vec3> vertices) {
        const std::vector<glm::vec3> transformedVertices = transformPoints(vertices, position2, rotation);
        const glm::vec3 direction = position1 - uniformConvexSetBarycenter(transformedVertices);
        const auto [a, b, c] = convexSetSupportTriangle(transformedVertices, direction);
        const glm::vec3 closestPoint = closestPointOnTriangle(position1, a, b, c);
        return pointSphereIntersection(position1, closestPoint, radius);
    }

    DiscreteIntersectionInfo capsuleConvexIntersection(const glm::vec3& position1, const glm::quat& rotation1, float halfHeight, float radius, const glm::vec3& position2, const glm::quat& rotation2, const std::vector<glm::vec3> vertices) {
        const std::vector<glm::vec3> transformedVertices = transformPoints(vertices, position2, rotation2);
        const auto [hemisphere1, hemisphere2] = capsuleHemispheres(position1, rotation1, halfHeight);
        const glm::vec3 direction = closestPointOnLine(position1, hemisphere1, hemisphere2, true) - uniformConvexSetBarycenter(transformedVertices);
        const auto [a, b, c] = convexSetSupportTriangle(transformedVertices, direction);
        const glm::vec3 closestPoint = closestPointOnTriangle(position1, a, b, c);
        return pointSphereIntersection(position1, closestPoint, radius);
    }

    DiscreteIntersectionInfo capsuleCapsuleIntersection(const glm::vec3& position1, const glm::quat& rotation1, float halfHeight1, float radius1, const glm::vec3& position2, const glm::quat& rotation2, float halfHeight2, float radius2) {
        const auto [hemisphere11, hemisphere12] = capsuleHemispheres(position1, rotation1, halfHeight1);
        const auto [hemisphere21, hemisphere22] = capsuleHemispheres(position2, rotation2, halfHeight2);
        const auto [closestPoint1, closestPoint2] = closestPointsOnLines(hemisphere11, hemisphere12, hemisphere21, hemisphere22, true, true);
        return sphereSphereIntersection(closestPoint1, radius1, closestPoint2, radius2);
    }

    DiscreteIntersectionInfo convexConvexIntersection(const glm::vec3& position1, const glm::quat& rotation1, const std::vector<glm::vec3>& vertices1, const glm::vec3& position2, const glm::quat& rotation2, const std::vector<glm::vec3>& vertices2) {
        return gjk(
            transformPoints(vertices1, position1, rotation1),
            transformPoints(vertices2, position2, rotation2)
        );
    }

    // Volume

    float tetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
        return std::abs(glm::dot(a - d, glm::cross(b - d, c - d))) / 6.0f;
    }

    float sphereVolume(float radius) {
        return 4.0f / 3.0f * glm::pi<float>() * radius * radius * radius;
    }

    float boxVolume(const glm::vec3& halfLengths) {
        return 8.0f * (halfLengths.x * halfLengths.y * halfLengths.z);
    }

    float convexSetVolume(const std::vector<glm::vec3>& convexSet) {
        float volume = 0.0f;
        for (auto [a, b, c, d] : convexSetTetrahedralization(convexSet)) {
            volume += tetrahedronVolume(a, b, c, d);
        }
        return volume;
    }

    float cylinderVolume(float radius, float halfHeight) {
        return 2.0f * halfHeight * radius * radius * glm::pi<float>();
    }

    float capsuleVolume(float radius, float halfHeight) {
        return cylinderVolume(radius, halfHeight) + sphereVolume(radius);
    }

    // Inertia

    glm::mat3 shiftInertia(const glm::mat3& inertiaTensor, const glm::vec3& shift) {
        const glm::vec3 sq{shift * shift};
        const float xy{-shift.x * shift.y};
        const float xz{-shift.x * shift.z};
        const float yz{-shift.y * shift.z};
        return inertiaTensor + glm::mat3(
            sq.y + sq.z, xy, xz,
            xy, sq.x + sq.y, yz,
            xz, yz, sq.x + sq.y
        );
    }

    glm::mat3 rotateInertiaTensor(const glm::mat3& inertiaTensor, const glm::quat& rotation) {
        glm::mat3 rotationMatrix = glm::mat3_cast(rotation);
        return glm::transpose(rotationMatrix) * inertiaTensor * rotationMatrix;
    }

    glm::mat3 tetrahedronInertiaTensor(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
        const float v = 0.05f * tetrahedronVolume(a, b, c, d);
        const glm::vec3 s2 = c + d;
        const glm::vec3 s3 = s2 + c;
        const glm::vec3 s4 = s3 + d;
        const glm::vec3 s = 2.0f * (a * s4 + b * s3 + c * s2 + d);

        const float ap = -s4.y * s4.z;
        const float bp = -s4.x * s4.z;
        const float cp = -s4.x * s4.y;
        return v * glm::mat3(
            s.y + s.z, bp, cp,
            bp, s.x + s.z, ap,
            cp, ap, s.x + s.y
        );
    }

    glm::mat3 sphereInertiaTensor(float radius) {
        return glm::mat3(2.0f / 5.0f * radius * radius);
    }

    glm::mat3 boxInertiaTensor(const glm::vec3& halfLengths) {
        const glm::vec3 sqLength = 4.0f * halfLengths * halfLengths;
        return glm::diagonal3x3(glm::vec3(sqLength.y + sqLength.z, sqLength.x + sqLength.z, sqLength.x + sqLength.y) / 12.0f);
    }

    glm::mat3 cylinderInertiaTensor(float radius, float halfHeight) {
        const float radius2 = radius * radius;
        const float a = (3.0f * radius2 + 4.0f * halfHeight * halfHeight) / 6.0f;
        return glm::diagonal3x3(glm::vec3(a, radius2, a) * 0.5f);
    }

    glm::mat3 capsuleInertiaTensor(float radius, float halfHeight) {
        const float height = 2.0f * halfHeight;
        const float radius2 = radius * radius;

        const float cylinderVolume = glm::pi<float>() * height * radius2;
        const float hemispheresVolume = 2.0f * glm::pi<float>() * radius * radius2 / 3.0f;
        const float cylinderPart = cylinderVolume / (cylinderVolume + 2.0f * hemispheresVolume);
        const float hemispheresPart = 1.0f - cylinderPart;

        const float a = 0.5f * radius2 * cylinderPart;
        const float b = 0.5f * a + cylinderPart * height * height / 12.0f;
        const float c = 2.0f * hemispheresPart * radius2 / 5.0f;
        const float d = c + hemispheresPart * (halfHeight * halfHeight + 3.0f / 8.0f * height * radius);
        return glm::diagonal3x3(glm::vec3(b + 2.0f * d, a + 2.0f * c, b + 2.0f * d));
    }

    glm::mat3 convexSetInertiaTensor(const std::vector<glm::vec3>& convexSet) {
        glm::vec3 barycenter = uniformConvexSetBarycenter(convexSet);
        glm::mat3 inertiaTensor = glm::mat3(0.0f);
        for (const auto[a, b, c, d] : convexSetTetrahedralization(convexSet)) {
            inertiaTensor += shiftInertia(tetrahedronInertiaTensor(a, b, c, d), barycenter);
        }
        return inertiaTensor;
    }
}