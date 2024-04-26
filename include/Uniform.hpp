#pragma once

#include "ShaderProgram.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Uniform {
public:

    Uniform(ShaderProgram& shaderProgram, std::string_view locationName);

    void set(int variable);

    void set(const glm::ivec1& vector);
    
    void set(const glm::ivec2& vector);

    void set(const glm::ivec3& vector);

    void set(const glm::ivec4& vector);

    void set(float variable);

    void set(const glm::fvec1& vector);
    
    void set(const glm::fvec2& vector);

    void set(const glm::fvec3& vector);

    void set(const glm::fvec4& vector);

    void set(unsigned variable);

    void set(const glm::uvec1& vector);
    
    void set(const glm::uvec2& vector);

    void set(const glm::uvec3& vector);

    void set(const glm::uvec4& vector);

    void set(const glm::mat2& matrix, bool transpose = false);

    void set(const glm::mat3& matrix, bool transpose = false);

    void set(const glm::mat4& matrix, bool transpose = false);

    void set(const glm::mat2x3& matrix, bool transpose = false);

    void set(const glm::mat3x2& matrix, bool transpose = false);

    void set(const glm::mat2x4& matrix, bool transpose = false);

    void set(const glm::mat4x2& matrix, bool transpose = false);

    void set(const glm::mat3x4& matrix, bool transpose = false);

    void set(const glm::mat4x3& matrix, bool transpose = false);

private:

    GLint location;

};