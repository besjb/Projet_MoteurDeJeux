#pragma once

#include <string_view>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <GL/glew.h>

class Shader {
public:

    enum class Type : GLenum {
        VERTEX = GL_VERTEX_SHADER,
        FRAGMENT = GL_FRAGMENT_SHADER,
        GEOMETRY = GL_GEOMETRY_SHADER,
        COMPUTE = GL_COMPUTE_SHADER,
        EVALUATION = GL_TESS_EVALUATION_SHADER,
        CONTROL = GL_TESS_CONTROL_SHADER
    };

    Shader(std::string_view programPath, Type shaderType);

    void reload();

    GLuint getId() const;

    Type getType() const;

    friend std::ostream& operator<<(std::ostream& out, const Shader& shader);

private:

    std::string programPath;
    Type shaderType;
    GLuint shaderIdentifier;

    void compile();
};