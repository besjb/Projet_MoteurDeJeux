#include "Mesh.hpp"
#include "Uniform.hpp"
#include "Scene.hpp"

#include <tiny_obj_loader.h>

#include <stdexcept>
#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/hash.hpp>
#include <functional>
#include <map>

Mesh::Mesh(MeshMaterial* material) :
    material{material}
{
    init();
}

Mesh::~Mesh() {
    destroy();
}

void Mesh::init() {
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &vertexPositionBuffer);

    glGenBuffers(1, &normalBuffer);
    
    glGenBuffers(1, &texCoordsBuffer);

    glGenBuffers(1, &EBO);
}

void Mesh::destroy() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &vertexPositionBuffer);
    glDeleteBuffers(1, &normalBuffer);
    glDeleteBuffers(1, &texCoordsBuffer);
    glDeleteBuffers(1, &EBO);
}

void Mesh::render() {
    material->getShaderProgram()->use();
    
    glBindVertexArray(VAO);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, material->getTextureLocation());

    glBindBuffer(GL_ARRAY_BUFFER, vertexPositionBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);
    
    glBindBuffer(GL_ARRAY_BUFFER, texCoordsBuffer);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, nullptr);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

Material* Mesh::getMaterial() {
    return material;
}

struct UniqueVertexData {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoord;

    bool operator==(const UniqueVertexData& other) const {
        return position == other.position && normal == other.normal && texCoord == other.texCoord;
    }
};

namespace std {
    template<> struct hash<UniqueVertexData> {
        std::size_t operator()(const UniqueVertexData& vertex) const {
            return ((hash<glm::vec3>()(vertex.position) ^
                   (hash<glm::vec3>()(vertex.normal) << 1)) >> 1) ^
                   (hash<glm::vec2>()(vertex.texCoord) << 1);
        }
    };
}

Mesh Mesh::loadFromFile(std::string_view modelPath, MeshMaterial* material) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, modelPath.data())) {
        throw std::runtime_error(warn + err);
    }

    Mesh mesh{material};

    std::unordered_map<UniqueVertexData, std::uint32_t> uniqueVertices;

    for (const tinyobj::shape_t& shape : shapes) {
        for (const tinyobj::index_t& index : shape.mesh.indices) {
            UniqueVertexData uniqueVertexData;

            uniqueVertexData.position = {
                attrib.vertices[3 * index.vertex_index],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };

            uniqueVertexData.texCoord = {
                attrib.texcoords[2 * index.texcoord_index],
                1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
            };

            uniqueVertexData.normal = {
                attrib.normals[3 * index.normal_index],
                attrib.normals[3 * index.normal_index + 1],
                attrib.normals[3 * index.normal_index + 2]
            };

            if (uniqueVertices.count(uniqueVertexData) == 0) {
                uniqueVertices[uniqueVertexData] = static_cast<std::uint32_t>(mesh.vertices.size());
                mesh.vertices.push_back(uniqueVertexData.position);
                mesh.texCoords.push_back(uniqueVertexData.texCoord);
                mesh.normals.push_back(uniqueVertexData.normal);
            }

            mesh.indices.push_back(uniqueVertices[uniqueVertexData]/*mesh.indices.size()*/);
        }
    }

    mesh.updateBuffers();
    return mesh;
}

void Mesh::updateBuffers() {
    glBindBuffer(GL_ARRAY_BUFFER, vertexPositionBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * normals.size(), normals.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, texCoordsBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(), indices.data(), GL_STATIC_DRAW);
}