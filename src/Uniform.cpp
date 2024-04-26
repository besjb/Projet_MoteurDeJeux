#include "Uniform.hpp"

Uniform::Uniform(ShaderProgram& shaderProgram, std::string_view locationName) :
    location{glGetUniformLocation(shaderProgram.getId(), locationName.data())}
{}

void Uniform::set(int variable) {
    glUniform1i(location, variable);
}

void Uniform::set(const glm::ivec1& vector) {
    glUniform1i(location, vector.x);
}

void Uniform::set(const glm::ivec2& vector) {
    glUniform2i(location, vector.x, vector.y);
}

void Uniform::set(const glm::ivec3& vector) {
    glUniform3i(location, vector.x, vector.y, vector.z);
}

void Uniform::set(const glm::ivec4& vector) {
    glUniform4i(location, vector.x, vector.y, vector.z, vector.w);
}



void Uniform::set(float variable) {
    glUniform1f(location, variable);
}

void Uniform::set(const glm::fvec1& vector) {
    glUniform1f(location, vector.x);
}

void Uniform::set(const glm::fvec2& vector) {
    glUniform2f(location, vector.x, vector.y);
}

void Uniform::set(const glm::fvec3& vector) {
    glUniform3f(location, vector.x, vector.y, vector.z);
}

void Uniform::set(const glm::fvec4& vector) {
    glUniform4f(location, vector.x, vector.y, vector.z, vector.w);
}



void Uniform::set(unsigned variable) {
    glUniform1ui(location, variable);
}

void Uniform::set(const glm::uvec1& vector) {
    glUniform1ui(location, vector.x);
}

void Uniform::set(const glm::uvec2& vector) {
    glUniform2ui(location, vector.x, vector.y);
}

void Uniform::set(const glm::uvec3& vector) {
    glUniform3ui(location, vector.x, vector.y, vector.z);
}

void Uniform::set(const glm::uvec4& vector) {
    glUniform4ui(location, vector.x, vector.y, vector.z, vector.w);
}



void Uniform::set(const glm::mat2& matrix, bool transpose) {
    glUniformMatrix2fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat3& matrix, bool transpose) {
    glUniformMatrix3fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat4& matrix, bool transpose) {
    glUniformMatrix4fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat2x3& matrix, bool transpose) {
    glUniformMatrix2x3fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat3x2& matrix, bool transpose) {
    glUniformMatrix3x2fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat2x4& matrix, bool transpose) {
    glUniformMatrix2x4fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat4x2& matrix, bool transpose) {
    glUniformMatrix4x2fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat3x4& matrix, bool transpose) {
    glUniformMatrix3x4fv(location, 1, transpose, glm::value_ptr(matrix));
}

void Uniform::set(const glm::mat4x3& matrix, bool transpose) {
    glUniformMatrix4x3fv(location, 1, transpose, glm::value_ptr(matrix));
}