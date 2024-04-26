#pragma once

#include <glm/glm.hpp>

class Material;

class Object {
public:

    virtual void render() = 0;
    virtual Material* getMaterial() = 0;

protected : 

    virtual void init() = 0;
    virtual void destroy() = 0;

};