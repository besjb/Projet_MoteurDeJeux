#include "Shader.hpp"

Shader::Shader(std::string_view programPath, Shader::Type shaderType) :
    programPath{programPath}, shaderType{shaderType}
{
    std::ifstream inputStream{programPath.data()};

    if (!inputStream.is_open()) {
        std::cerr << "Unable to open shader file at path \"" << programPath << "\"\n";
        throw std::runtime_error("Failed to open file");
    }

    std::string shaderSource{std::string{
        std::istreambuf_iterator<char>(inputStream),
        std::istreambuf_iterator<char>()
    }};

    shaderIdentifier = glCreateShader(static_cast<GLenum>(shaderType));
    const char* cShaderSource{shaderSource.c_str()};
    glShaderSource(shaderIdentifier, 1, &cShaderSource, NULL);
    compile();
}

void Shader::reload() {
    compile();
}

void Shader::compile() {
    glCompileShader(shaderIdentifier);

    GLint compileStatus;
    glGetShaderiv(shaderIdentifier, GL_COMPILE_STATUS, &compileStatus);
    if (compileStatus == GL_FALSE) {
        GLint maxLength = 0;
        glGetShaderiv(shaderIdentifier, GL_INFO_LOG_LENGTH, &maxLength);
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(shaderIdentifier, maxLength, &maxLength, &errorLog[0]);
        std::cerr << "Shader compilation failed for " << *this << ":\n" <<  &errorLog[0] << '\n';
        glDeleteShader(shaderIdentifier);
        throw std::runtime_error("Shader compilation failed");
    }
}

GLuint Shader::getId() const {
    return shaderIdentifier;
}

Shader::Type Shader::getType() const {
    return shaderType;
}

std::ostream& operator<<(std::ostream& out, const Shader& shader) {
    switch (shader.getType()) {
        case Shader::Type::VERTEX :
            out << "Vertex";
            break;
        case Shader::Type::FRAGMENT :
            out << "Fragment";
            break;
        case Shader::Type::GEOMETRY :
            out << "Geometry";
            break;
        case Shader::Type::COMPUTE :
            out << "Compute";
            break;
        case Shader::Type::EVALUATION :
            out << "Evaluation";
            break;
        case Shader::Type::CONTROL :
            out << "Control";
            break;
    }
    return out << " shader(\"" << shader.programPath << "\")";
}