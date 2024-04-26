#include "Scene.hpp"
#include "Material.hpp"

#include <stack>
#include <algorithm>

TransformTree::TransformTree(const Transform& transform, const std::vector<Object*>& objects) :
    parent{nullptr},
    transform{transform},
    objects{objects}
{}

TransformTree::~TransformTree() {
    for (TransformTree* child : childrenTransforms) {
        delete child;
    }
}

bool TransformTree::hasParent() {
    return parent != nullptr;
}

TransformTree* TransformTree::getParent() const {
    return parent;
}

TransformTree* TransformTree::addChild(const Transform& transform) {
    TransformTree* subTree{new TransformTree{transform}};
    childrenTransforms.push_back(subTree);
    subTree->parent = this;
    return subTree;
}

TransformTree* TransformTree::addObject(Object* object) {
    objects.emplace_back(object);
    return this;
}

TransformTree* TransformTree::ref(TransformTree*& reference) {
    reference = this;
    return this;
}

const std::vector<Object*> TransformTree::getObjects() const {
    return objects;
}

const std::vector<TransformTree*> TransformTree::getChildren() const {
    return childrenTransforms;
}

Scene::Scene(const LightSource& lightSource, const Camera& camera) :
    lightSource{lightSource},
    camera{camera},
    root{}
{}

void Scene::render() {
    std::stack<std::pair<TransformTree*, Transform>> transformStack;
    transformStack.push(std::make_pair(&root, root.transform));

    while (!transformStack.empty()) {
        TransformTree* transformNode{transformStack.top().first};
        Transform transform{transformStack.top().second};
        transformStack.pop();

        for (Object* object : transformNode->getObjects()) {
            object->getMaterial()->setUniforms(*this, transform.toMat4());
            object->render();
        }

        for (TransformTree* transformTree : transformNode->getChildren()) {
            transformStack.push(std::make_pair(transformTree, transform.multiply(transformTree->transform)));
        }
    }
}

TransformTree* Scene::getRootTransformTree() {
    return &root;
}

LightSource& Scene::getLightSource() {
    return lightSource;
}

const LightSource& Scene::getLightSource() const {
    return lightSource;
}

Camera& Scene::getCamera() {
    return camera;
}

const Camera& Scene::getCamera() const {
    return camera;
}