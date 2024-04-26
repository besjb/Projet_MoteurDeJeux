#pragma once

#include "Shader.hpp"

#include <list>
#include <GL/glew.h>

// todo delete program?

class ShaderProgram {
public:

    ShaderProgram();

    bool attachShader(Shader& shader);

    bool detachShader(Shader::Type shaderType);

    bool usesShader(Shader::Type shaderType) const;

    void link();

    void use();

    GLuint getId() const;

    void destroy();

protected:

    using shaders_iterator_t = std::list<Shader*>::const_iterator;

    shaders_iterator_t getShaderIterator(Shader::Type shaderType) const;

private:

    std::list<Shader*> shaders;

    GLuint programIdentifier;
};