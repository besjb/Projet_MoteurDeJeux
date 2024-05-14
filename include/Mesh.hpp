#pragma once

#include "Object.hpp"
#include "MeshMaterial.hpp"
#include "Uniform.hpp"

#include <glm/glm.hpp>
#include <GL/glew.h>

#include <vector>
#include <string_view>
#include <cstdint>

class Mesh : public Object {
public:

    void render() override;

    Material* getMaterial() override;

    static Mesh loadFromFile(std::string_view modelPath, MeshMaterial* material);

    Mesh(MeshMaterial* material);

    ~Mesh();

protected:

    void init() override;

    void destroy() override;

private:

    void updateBuffers();

    std::vector<std::uint32_t> indices;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> texCoords;

    GLuint VAO;
    GLuint vertexPositionBuffer;
    GLuint normalBuffer;
    GLuint texCoordsBuffer;
    GLuint EBO;

    MeshMaterial* material;

};