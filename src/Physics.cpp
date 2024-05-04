#include "Physics.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <algorithm>
#include <iostream>

#include "Scene.hpp"
#include "Mesh.hpp"

extern Mesh* globalEarthMesh;
extern Scene* scenePointer;

void PhysicsEngine::tick(float delta) {
    for (RigidBody::Ref& rigidBody : rigidBodies) {
        rigidBody->tick(getEffectiveForceFields(rigidBody), delta);
    }

    // todo modifier
    constexpr bool continuous{false};

    for (const CollidingPair& pair : getPairs(continuous)) {
        const geometry::DiscreteIntersectionInfo intersectionInfo = pair.first->getCollider()->getCollisionInfo(pair.first->getPosition(), pair.second->getPosition(), pair.first->getRotation(), pair.second->getRotation(), pair.second->getCollider(), continuous);
        if (intersectionInfo.intersects()) {
            if (!continuous) {
                RigidBody::Ref body1 = pair.first;
                RigidBody::Ref body2 = pair.second;
                // Collision discrète
                float massRatio;
                if (body1->isStatic()) {
                    massRatio = 0.0f;
                }
                else if (body2->isStatic()) {
                    massRatio = 1.0f;
                }
                else {
                    massRatio = body1->getMass() / (body1->getMass() + body2->getMass());
                }
                body1->setPosition(body1->getPosition() + intersectionInfo.getPenetration() * intersectionInfo.getNormal() * massRatio);
                body2->setPosition(body2->getPosition() - intersectionInfo.getPenetration() * intersectionInfo.getNormal() * (1.0f - massRatio));

                const glm::vec3 collisionPoint = intersectionInfo.getIntersectionPolygon()[0] + intersectionInfo.getNormal() * intersectionInfo.getPenetration() * (massRatio - 0.5f);
                //scenePointer->getRootTransformTree()->addChild({collisionPoint, glm::identity<glm::quat>(), glm::vec3(0.04f)})->addObject(globalEarthMesh);

                const glm::vec3 velocity1 = body1->getVelocity() + glm::cross(body1->getAngularVelocity(), collisionPoint - body1->getPosition());
                const glm::vec3 velocity2 = body2->getVelocity() + glm::cross(body2->getAngularVelocity(), collisionPoint - body2->getPosition());

                const glm::vec3 relativeVelocity = velocity2 - velocity1;

                const float normalVelocity = glm::dot(relativeVelocity, intersectionInfo.getNormal());

                const float restitution = body1->getPhysicsMaterial().restitution + pair.second->getPhysicsMaterial().restitution;
                
                const glm::vec3 relativeCollisionPoint1 = collisionPoint - body1->getPosition();
                const glm::vec3 relativeCollisionPoint2 = collisionPoint - body2->getPosition();

                const float invMasses = 1.0f / body1->getMass() + 1.0f / body2->getMass();
                const glm::mat3 bodyRotation1 = glm::mat3_cast(body1->getRotation());
                const glm::mat3 bodyRotation2 = glm::mat3_cast(body2->getRotation());

                const glm::mat3 invI1 = body1->isStatic() ? glm::mat3(0.0f) : glm::inverse(glm::transpose(bodyRotation1) * body1->getCollider()->getInertiaTensor(body1->getMass()) * bodyRotation1);
                const glm::mat3 invI2 = body2->isStatic() ? glm::mat3(0.0f) : glm::inverse(glm::transpose(bodyRotation2) * body2->getCollider()->getInertiaTensor(body2->getMass()) * bodyRotation2);
                
                const glm::vec3 theta1 = body1->canRotate() ? glm::cross(invI1 * glm::cross(relativeCollisionPoint1, intersectionInfo.getNormal()), relativeCollisionPoint1) : glm::vec3(0.0f, 0.0f, 0.0f);
                const glm::vec3 theta2 = body2->canRotate() ? glm::cross(invI2 * glm::cross(relativeCollisionPoint2, intersectionInfo.getNormal()), relativeCollisionPoint2) : glm::vec3(0.0f, 0.0f, 0.0f);

                const float j = -(1.0f + restitution) * normalVelocity / (invMasses + glm::dot(theta1 + theta2, intersectionInfo.getNormal()));
                //const float j = -(1.0f + restitution) * normalVelocity / invMasses;

                const glm::vec3 impulse = j * intersectionInfo.getNormal();

                body1->applyImpulse(relativeCollisionPoint1, invI1, impulse);
                body2->applyImpulse(relativeCollisionPoint2, invI2, -impulse);
                /*body1->setVelocity(body1->getVelocity() - impulse / body1->getMass());
                body2->setVelocity(body2->getVelocity() + impulse / body2->getMass());

                if (body1->canRotate()) {
                    body1->setAngularVelocity(body1->getAngularVelocity() - invI1 * glm::cross(relativeCollisionPoint1, impulse));
                }
                if (body2->canRotate()) {
                    body2->setAngularVelocity(body2->getAngularVelocity() + invI2 * glm::cross(relativeCollisionPoint2, impulse));
                }*/

                /* Friction */
                /*const float friction = body1->getPhysicsMaterial().friction * pair.second->getPhysicsMaterial().friction;
                const glm::vec3 tangent = glm::normalize(relativeVelocity - glm::dot(relativeVelocity, intersectionInfo.normal) * intersectionInfo.normal);

                const float frictionMagnitude = friction * glm::length(impulse);

                // todo friction statique (qui bloque les objets pentus sur place)
                float staticFrictionForce = staticFrictionCoefficient * glm::length(impulse);
                if (frictionMagnitude > staticFrictionForce) {
                    frictionMagnitude = staticFrictionForce;
                }

                glm::vec3 frictionForce = -frictionMagnitude * tangent;

                body1->setVelocity(body1->getVelocity() + frictionForce / body1->getMass());
                body2->setVelocity(body2->getVelocity() - frictionForce / body2->getMass());

                if (body1->canRotate()) {
                    glm::vec3 leverArm = relativeCollisionPoint1 - glm::dot(relativeCollisionPoint1, intersectionInfo.normal) * intersectionInfo.normal;
                    glm::vec3 torque = glm::cross(leverArm, frictionForce);
                    body1->setAngularVelocity(body1->getAngularVelocity() + invI1 * torque);
                }

                if (body2->canRotate()) {
                    glm::vec3 leverArm = relativeCollisionPoint2 - glm::dot(relativeCollisionPoint2, intersectionInfo.normal) * intersectionInfo.normal;
                    glm::vec3 torque = glm::cross(leverArm, frictionForce);
                    body2->setAngularVelocity(body2->getAngularVelocity() - invI2 * torque);
                }*/
            }
        }
    }

    triggerEvents(delta);
}

void PhysicsEngine::addRigidBody(RigidBody::Ref& rigidBody) {
    rigidBodies.push_back(rigidBody);
}

void PhysicsEngine::addForceField(ForceField::Ref& forceField) {
    forceFields.push_back(forceField);
}

void PhysicsEngine::addTrigger(Trigger::Ref& trigger) {
    triggers.push_back(trigger);
}

PhysicsEngine& PhysicsEngine::setCollisionRule(const std::vector<int>& collisionLayers) {
    for (int collisionLayer1 : collisionLayers) {
        for (int collisionLayer2 : collisionLayers) {
            if (collisionLayer1 != collisionLayer2) {
                collisionRules[collisionLayer1].insert(collisionLayer2);
            }
        }
    }
    return *this;
}

PhysicsEngine& PhysicsEngine::setNoCollisionRule(const std::vector<int>& collisionLayers) {
    for (int collisionLayer1 : collisionLayers) {
        for (int collisionLayer2 : collisionLayers) {
            if (collisionLayer1 != collisionLayer2) {
                collisionRules[collisionLayer1].erase(collisionLayer2);
            }
        }
    }
    return *this;
}

bool PhysicsEngine::doLayersCollide(int collisionLayer1, int collisionLayer2) const {
    return collisionLayer1 == collisionLayer2 || (collisionRules.find(collisionLayer1) != collisionRules.end() && collisionRules.at(collisionLayer1).contains(collisionLayer2));
}

std::vector<PhysicsEngine::CollidingPair> PhysicsEngine::sweepAndPrune() const {
    std::vector<glm::vec3> minExtents(rigidBodies.size());
    std::vector<glm::vec3> maxExtents(rigidBodies.size());
    std::vector<std::size_t> xSortedPositions(rigidBodies.size());
    for (std::size_t i = 0; i < rigidBodies.size(); ++i) {
        std::tie(minExtents[i], maxExtents[i]) = rigidBodies[i]->getCollider()->getBoundingBox(rigidBodies[i]->getPosition(), rigidBodies[i]->getRotation());
        xSortedPositions[i] = i;
    }
    std::sort(xSortedPositions.begin(), xSortedPositions.end(), [&](std::size_t bodyIndex1, std::size_t bodyIndex2) {
        return minExtents[bodyIndex1].x > minExtents[bodyIndex2].x;
    });
    std::vector<PhysicsEngine::CollidingPair> potentialCollidingPairs;
    for (std::size_t i = 0; i < rigidBodies.size() - 1; ++i) {
        std::size_t j = i + 1;
        bool checkNextBoxes{true};
        do {
            if (maxExtents[xSortedPositions[i]].x >= minExtents[xSortedPositions[j]].x) {
                if ((maxExtents[i].y >= minExtents[j].y && maxExtents[j].y >= minExtents[i].y) && (maxExtents[i].z >= minExtents[j].z && maxExtents[j].z >= minExtents[i].z)) {
                    potentialCollidingPairs.push_back(std::make_pair(rigidBodies[i], rigidBodies[j]));
                }
                ++j;
            }
            else {
                checkNextBoxes = false;
            }
        } while (checkNextBoxes && j < rigidBodies.size());
    }
    return potentialCollidingPairs;
}

std::vector<PhysicsEngine::CollidingPair> PhysicsEngine::getPairs(bool continuous) const {
    std::vector<PhysicsEngine::CollidingPair> collidingPairs;
    for (std::size_t i = 0; i < rigidBodies.size(); ++i) {
        for (std::size_t j = i + 1; j < rigidBodies.size(); ++j) {
            if (!(rigidBodies[i]->isStatic() && rigidBodies[j]->isStatic()) && doLayersCollide(rigidBodies[i]->getLayer(), rigidBodies[j]->getLayer())) {
                collidingPairs.push_back(std::make_pair(rigidBodies[i], rigidBodies[j]));
            }
        }
    }
    /*for (PhysicsEngine::CollidingPair& collidingPair : sweepAndPrune()) {
        auto [body1, body2] = collidingPair;
        if (!(body1->isStatic() && body2->isStatic()) && doLayersCollide(body1->getLayer(), body2->getLayer())) {
            collidingPairs.push_back(collidingPair);
        }
    }*/
    return collidingPairs;
}

std::vector<ForceField::Ref> PhysicsEngine::getEffectiveForceFields(const RigidBody::Ref& rigidBody) const {
    std::vector<ForceField::Ref> effectiveForceFields;
    for (const ForceField::Ref& forceField : forceFields) {
        if (forceField->isGlobal() || rigidBody->getCollider()->getCollisionInfo(rigidBody->getPosition(), forceField->getPosition(), rigidBody->getRotation(), forceField->getRotation(), forceField->getCollider(), false).intersects()) {
            effectiveForceFields.push_back(forceField);
        }
    }
    return effectiveForceFields;
}

void PhysicsEngine::triggerEvents(float delta) {
    for (std::size_t i = 0; i < rigidBodies.size(); ++i) {
        for (std::size_t j = 0; j < triggers.size(); ++j) {
            RigidBody::Ref rigidBody = rigidBodies[i];
            Trigger::Ref trigger = triggers[j];
            if (trigger->getCollider()->getCollisionInfo(trigger->getPosition(), rigidBody->getPosition(), trigger->getRotation(), rigidBody->getRotation(), rigidBody->getCollider(), false).intersects()) {
                if (functionalTriggers[j].contains(i)) {
                    trigger->triggerOnTickEvent(rigidBody, delta);
                }
                else {
                    functionalTriggers[j].insert(i);
                    trigger->triggerOnEnterEvent(rigidBody, delta);
                }
            }
            else if (functionalTriggers[j].contains(i)) {
                functionalTriggers[j].erase(i);
                trigger->triggerOnExitEvent(rigidBody, delta);
            }
        }
    }
}

PhysicsMaterial::PhysicsMaterial(
    float friction,
    float restitution,
    float linearDamping,
    float angularDamping
) :
    friction(friction),
    restitution(restitution),
    linearDamping(linearDamping),
    angularDamping(angularDamping)
{
}

PhysicsMaterial::~PhysicsMaterial() {
}

/*************/
/* RigidBody */
/*************/

RigidBody::RigidBody(
    TransformTree* transformTree,
    Collider* collider,
    float mass,
    PhysicsMaterial material,
    int layer,
    bool staticBody,
    const glm::vec3& position,
    const glm::vec3& velocity,
    bool fixedRotation,
    const glm::quat& rotation,
    const glm::vec3& angularVelocity
    
) :
    transformTree(transformTree),
    collider(collider),
    mass(mass),
    material(material),
    layer(layer),
    staticBody(staticBody),
    position(position),
    velocity(velocity),
    fixedRotation(fixedRotation),
    rotation(rotation),
    angularVelocity(angularVelocity)
{
}

RigidBody::~RigidBody() {
}

void RigidBody::tick(const std::vector<ForceField::Ref>& forceFields, float delta) {
    if (!isStatic()) {
        glm::vec3 additionalAcceleration = glm::vec3(0.0f);
        for (const ForceField::Ref& forceField : forceFields) {
            additionalAcceleration += forceField->getAcceleration(position);
        }
        //velocity += delta * (acceleration + additionalAcceleration);
        //position += delta * velocity;
        integrateMotion(forceFields, delta);

        if (canRotate()) {
            rotation = glm::quat{delta * angularVelocity} * rotation;
        }
    }    

    if (transformTree != nullptr) {
        transformTree->transform.setTranslation(position);
        transformTree->transform.setRotation(rotation);
    }
}

RigidBody& RigidBody::applyImpulse(const glm::vec3& impulsePosition, const glm::mat3& inverseInertia, const glm::vec3& impulse) {
    if (!isStatic()) {
        velocity -= impulse / mass;
        if (canRotate()) {
            angularVelocity -= inverseInertia * glm::cross(impulsePosition, impulse);
        }
    }
    return *this;
}

glm::vec3 RigidBody::getAccelerationAtPosition(const std::vector<ForceField::Ref>& forceFields) const {
    glm::vec3 acceleration = glm::vec3(0.0f);
    for (const ForceField::Ref& forceField : forceFields) {
        acceleration += forceField->getAcceleration(position);
    }
    return acceleration;
}

void RigidBody::integrateMotion(const std::vector<ForceField::Ref>& forceFields, float delta) {
    glm::vec3 acceleration{getAccelerationAtPosition(forceFields)};

    position += velocity * delta + 0.5f * acceleration * delta * delta;
    
    glm::vec3 newAcceleration{getAccelerationAtPosition(forceFields)};
        
    velocity += 0.5f * (acceleration + newAcceleration) * delta;
}

geometry::DiscreteIntersectionInfo Collider::getCollisionInfo(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const Collider* collider, bool continuous) {
    switch (collider->getColliderType()) {
        case ColliderType::OBB :
            return getCollisionInfoOBB(position, positionOther, rotation, rotationOther, static_cast<const OBBCollider*>(collider), continuous);
        case ColliderType::SPHERE :
            return getCollisionInfoSphere(position, positionOther, rotation, rotationOther, static_cast<const SphereCollider*>(collider), continuous);
        case ColliderType::CAPSULE :
            return getCollisionInfoCapsule(position, positionOther, rotation, rotationOther, static_cast<const CapsuleCollider*>(collider), continuous);
        case ColliderType::CONVEX :
            return getCollisionInfoConvex(position, positionOther, rotation, rotationOther, static_cast<const ConvexCollider*>(collider), continuous);
        default:
            return geometry::DiscreteIntersectionInfo::none();
    }
}

/**************/
/* ForceField */
/**************/

ForceField::ForceField(
    Collider* collider,
    glm::vec3 position,
    glm::quat rotation,
    const ForceFieldFunction& forceFieldFunction
) :
    collider(collider),
    position(position),
    rotation(rotation),
    forceFieldFunction(forceFieldFunction)
{}

ForceField::~ForceField() {
}

/***********/
/* Trigger */
/***********/

Trigger::Trigger (
    Collider* collider,
    glm::vec3 position,
    glm::quat rotation,
    const TriggerEvent& onEnterEvent,
    const TriggerEvent& onExitEvent,
    const TriggerEvent& onTickEvent
) :
    collider(collider),
    position(position),
    rotation(rotation),
    onEnterEvent(onEnterEvent),
    onExitEvent(onExitEvent),
    onTickEvent(onTickEvent)
{}

Trigger::~Trigger() {
}

/********************/
/* Helper functions */
/********************/

static geometry::DiscreteIntersectionInfo collisionInfoSphereConvex(const glm::vec3& position1, const glm::vec3& position2, const glm::quat rotation, const float radius, const std::vector<glm::vec3>& vertices) {
    glm::vec3 eulerRotation = glm::eulerAngles(rotation);
    glm::mat4 rotationMatrix = glm::eulerAngleXYZ(eulerRotation.x, eulerRotation.y, eulerRotation.z);
    std::vector<glm::vec3> verticesPositions(vertices.size());
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        verticesPositions[i] = position2 + glm::vec3(rotationMatrix * glm::vec4(vertices[i], 1.0f));
    }

    float minDistance = std::numeric_limits<float>::infinity();
    glm::vec3 closestPoint = glm::vec3(0.0f);
    /*for (std::size_t i = 0; i < vertices.size(); ++i) {
        for (std::size_t j = 0; j < vertices.size(); ++j) {
            for (std::size_t k = 0; k < vertices.size(); ++k) {
                if (i == j || j == k || k == i) {
                    glm::vec3 point = closestPointOnTriangle(position1, vertices[i], vertices[j], vertices[k]);
                    float distance = glm::distance2(point, position1);
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestPoint = point;
                    }
                }
            }
        }
    }*/
    for (std::size_t i = 0; i < verticesPositions.size(); ++i) {
        glm::vec3 point = verticesPositions[i];
        float distance = glm::distance2(point, position1);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = point;
        }
    }
    return geometry::pointSphereIntersection(position1, closestPoint, radius);
}

/*******/
/* OBB */
/*******/

OBBCollider::OBBCollider(const glm::vec3& halfDimensions) :
    size(halfDimensions)
{}

glm::mat3 OBBCollider::getInertiaTensor(float mass) const {
    const float x2{size.x * size.x};
    const float y2{size.y * size.y};
    const float z2{size.z * size.z};
    return glm::diagonal3x3(mass / 3.0f * glm::vec3(y2 + z2, x2 + z2, x2 + y2));
}

geometry::DiscreteIntersectionInfo OBBCollider::getCollisionInfoOBB(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const OBBCollider* collider, bool continuous) const {
    return geometry::boxBoxIntersection(position, rotation, size, positionOther, rotationOther, collider->size);
}

geometry::DiscreteIntersectionInfo OBBCollider::getCollisionInfoSphere(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const SphereCollider* collider, bool continuous) const {
    return geometry::boxSphereIntersection(position, rotation, size, positionOther, collider->radius);
}

geometry::DiscreteIntersectionInfo OBBCollider::getCollisionInfoConvex(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const ConvexCollider* collider, bool continuous) const {
    return geometry::convexConvexIntersection(
        position, rotation, geometry::getBoxVertices(size),
        positionOther, rotationOther, collider->vertices
    );
}

geometry::BoundingBox OBBCollider::getBoundingBox(const glm::vec3& position, const glm::quat& rotation) const {
    return geometry::convexSetBoundingBox(geometry::transformPoints(geometry::getBoxVertices(size), position, rotation));
}

/**********/
/* Sphere */
/**********/

SphereCollider::SphereCollider(float radius) :
    radius(radius)
{}

glm::mat3 SphereCollider::getInertiaTensor(float mass) const {
    return glm::mat3(2.0f / 5.0f * mass * radius * radius);
}

geometry::DiscreteIntersectionInfo SphereCollider::getCollisionInfoOBB(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const OBBCollider* collider, bool continuous) const {
    return geometry::boxSphereIntersection(positionOther, rotationOther, collider->size, position, radius).getInverse();
}

geometry::DiscreteIntersectionInfo SphereCollider::getCollisionInfoSphere(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const SphereCollider* collider, bool continuous) const {
    return geometry::sphereSphereIntersection(position, radius, positionOther, collider->radius);
}

geometry::DiscreteIntersectionInfo SphereCollider::getCollisionInfoCapsule(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const CapsuleCollider* collider, bool continuous) const {
    return geometry::sphereCapsuleIntersection(position, radius, positionOther, rotationOther, collider->halfLength, collider->radius);
}

geometry::DiscreteIntersectionInfo SphereCollider::getCollisionInfoConvex(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const ConvexCollider* collider, bool continuous) const {
    return collisionInfoSphereConvex(position, positionOther, rotationOther, radius, collider->vertices);
}

geometry::BoundingBox SphereCollider::getBoundingBox(const glm::vec3& position, const glm::quat& rotation) const {
    const glm::vec3 radiusVector = glm::vec3(radius);
    return std::make_pair(
        position - radiusVector,
        position + radiusVector
    );
}

/***********/
/* Capsule */
/***********/

CapsuleCollider::CapsuleCollider(float halfLength, float radius) :
    halfLength(halfLength),
    radius(radius)
{}

/*
const float oneDiv3 = (float)(1.0 / 3.0);
const float oneDiv8 = (float)(1.0 / 8.0);
const float oneDiv12 = (float)(1.0 / 12.0);
void ComputeRigidBodyProperties_Capsule(float capsuleHeight, float capsuleRadius, float density, float & mass, float3 & centerOfMass, float3x3 & inertia) {
  float cM; // cylinder mass
  float hsM; // mass of hemispheres
  float rSq = capsuleRadius * capsuleRadius;
  cM = PI * capsuleHeight * rSq * density;
  hsM = PI_TIMES2 * oneDiv3 * rSq * capsuleRadius * density; // from cylinder
  inertia._22 = rSq * cM * 0.5 f;
  inertia._11 = inertia._33 = inertia._22 * 0.5 f + cM * capsuleHeight * capsuleHeight * oneDiv12; // from hemispheres
  float temp0 = hsM * 2.0 f * rSq / 5.0 f;
  inertia._22 += temp0 * 2.0 f;
  float temp1 = capsuleHeight * 0.5 f;
  float temp2 = temp0 + hsM * (temp1 * temp1 + 3.0 f * oneDiv8 * capsuleHeight * capsuleRadius);
  inertia._11 += temp2 * 2.0 f;
  inertia._33 += temp2 * 2.0 f;
  inertia._12 = inertia._13 = inertia._21 = inertia._23 = inertia._31 = inertia._32 = 0.0 f;
  mass = cM + hsM * 2.0 f;
}*/

glm::mat3 CapsuleCollider::getInertiaTensor(float mass) const {
    const float length = 2.0f * halfLength;
    const float radius2 = radius * radius;

    const float cylinderVolume = glm::pi<float>() * length * radius2;
    const float hemispheresVolume = 2.0f * glm::pi<float>() * radius * radius2 / 3.0f;
    const float cylinderMass = mass * cylinderVolume / (cylinderVolume + 2.0f * hemispheresVolume);
    const float hemispheresMass = mass - cylinderMass;

    const float a = 0.5f * radius2 * cylinderMass;
    const float b = 0.5f * a + cylinderMass * length * length / 12.0f;
    const float c = 2.0f * hemispheresMass * radius2 / 5.0f;
    const float d = c + hemispheresMass * (halfLength * halfLength + 3.0f / 8.0f * length * radius);
    return glm::diagonal3x3(glm::vec3(b + 2.0f * d, a + 2.0f * c, b + 2.0f * d));
}

geometry::DiscreteIntersectionInfo CapsuleCollider::getCollisionInfoSphere(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const SphereCollider* collider, bool continuous) const {
    return geometry::sphereCapsuleIntersection(positionOther, collider->radius, position, rotation, halfLength, radius).getInverse();
}

geometry::BoundingBox CapsuleCollider::getBoundingBox(const glm::vec3& position, const glm::quat& rotation) const {
    auto [capsuleCenter1, capsuleCenter2] = geometry::capsuleCenters(position, rotation, halfLength);
    return geometry::capsuleBoundingBox(capsuleCenter1, capsuleCenter2, radius);
}

/**********/
/* Convex */
/**********/

ConvexCollider::ConvexCollider(
    const std::vector<glm::vec3>& vertices
) :
    vertices(vertices)
{}

glm::mat3 ConvexCollider::getInertiaTensor(float mass) const {
    return glm::mat3(1.0f);
}

geometry::DiscreteIntersectionInfo ConvexCollider::getCollisionInfoOBB(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const OBBCollider* collider, bool continuous) const {
    return geometry::convexConvexIntersection(
        position, rotation, vertices,
        positionOther, rotationOther, geometry::getBoxVertices(collider->size)
    );
}

geometry::DiscreteIntersectionInfo ConvexCollider::getCollisionInfoConvex(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const ConvexCollider* collider, bool continuous) const {
    return geometry::convexConvexIntersection(
        position, rotation, vertices,
        positionOther, rotationOther, collider->vertices
    );
}

geometry::DiscreteIntersectionInfo ConvexCollider::getCollisionInfoSphere(const glm::vec3& position, const glm::vec3& positionOther, const glm::quat& rotation, const glm::quat& rotationOther, const SphereCollider* collider, bool continuous) const {
    return collisionInfoSphereConvex(positionOther, position, rotation, collider->radius, vertices).getInverse();
}

geometry::BoundingBox ConvexCollider::getBoundingBox(const glm::vec3& position, const glm::quat& rotation) const {
    return geometry::convexSetBoundingBox(geometry::transformPoints(vertices, position, rotation));
}