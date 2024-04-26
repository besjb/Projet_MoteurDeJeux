#pragma once

#include "Transform.hpp"
#include "Object.hpp"
#include "Camera.hpp"
#include "LightSource.hpp"

#include <vector>

class TransformTree {
public:

    TransformTree(const Transform& transform = {}, const std::vector<Object*>& objects = {});

    ~TransformTree();

    Transform transform;

    bool hasParent();

    TransformTree* getParent() const;

    TransformTree* addChild(const Transform& transform = {});

    TransformTree* addObject(Object* object);

    TransformTree* ref(TransformTree*& reference);

    const std::vector<Object*> getObjects() const;

    const std::vector<TransformTree*> getChildren() const;

private:

    TransformTree* parent;

    std::vector<TransformTree*> childrenTransforms;

    std::vector<Object*> objects;

};

class Scene {
public:

    Scene(const LightSource& lightSource = {}, const Camera& camera = {});

    void render();

    TransformTree* getRootTransformTree();

    LightSource& getLightSource();

    const LightSource& getLightSource() const;

    Camera& getCamera();

    const Camera& getCamera() const;

private:

    LightSource lightSource;

    Camera camera;

    TransformTree root;

};