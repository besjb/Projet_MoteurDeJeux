#include "ShaderProgram.hpp"

ShaderProgram::ShaderProgram() :
    programIdentifier{glCreateProgram()}
{}

bool ShaderProgram::attachShader(Shader& shader) {
    if (usesShader(shader.getType())) {
        return false;
    }
    shaders.push_front(&shader);
    return true;
}

bool ShaderProgram::detachShader(Shader::Type shaderType) {
    shaders_iterator_t shaderPosition{getShaderIterator(shaderType)};
    if (shaderPosition == shaders.end()) {
        return false;
    }
    shaders.erase(shaderPosition);
    return true;
}

bool ShaderProgram::usesShader(Shader::Type shaderType) const {
    return getShaderIterator(shaderType) != shaders.end();
}

ShaderProgram::shaders_iterator_t ShaderProgram::getShaderIterator(Shader::Type shaderType) const {
    for (shaders_iterator_t it{shaders.begin()}; it != shaders.end(); ++it) {
        if ((*it)->getType() == shaderType) {
            return it;
        }
    }
    return shaders.end();
}

void ShaderProgram::link() {
    for (const Shader* shader : shaders) {
        glAttachShader(programIdentifier, shader->getId());
    }

    glLinkProgram(programIdentifier);

    // Check for linking errors
    GLint linkStatus;
    glGetProgramiv(programIdentifier, GL_LINK_STATUS, &linkStatus);
    if (linkStatus == GL_FALSE) {
        GLint maxLength = 0;
        glGetProgramiv(programIdentifier, GL_INFO_LOG_LENGTH, &maxLength);
        std::vector<GLchar> errorLog(maxLength);
        glGetProgramInfoLog(programIdentifier, maxLength, &maxLength, &errorLog[0]);
        std::cerr << "Shader program linking failed:\n" << &errorLog[0] << '\n';

        glDeleteProgram(programIdentifier);
        throw std::runtime_error("Shader program linking failed");
    }
}

GLuint ShaderProgram::getId() const {
    return programIdentifier;
}

void ShaderProgram::use() {
    glUseProgram(programIdentifier);
}

void ShaderProgram::destroy() {
    glDeleteProgram(programIdentifier);
}