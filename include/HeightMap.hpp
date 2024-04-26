#pragma once

#include "HeightMapMaterial.hpp"
#include "Object.hpp"
#include "Uniform.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

#include <GL/glew.h>

class HeightMap : public Object {
public:

    HeightMap(HeightMapMaterial* material);

    HeightMap(HeightMapMaterial* material, unsigned xSize, unsigned zSize, float height);

    ~HeightMap();

    Material* getMaterial() override;

    void render() override;

    static HeightMap loadFromFile(std::string_view fileName, HeightMapMaterial* material, unsigned xSize, unsigned zSize, float height);

    std::vector<float>& operator[](std::size_t position);

    const std::vector<float>& operator[](std::size_t position) const;

    void updateBuffers();

    glm::vec3 getHeightAt(float x, float z) const;

protected:

    void init() override;

    void destroy() override;

private:

    unsigned xSize;
    unsigned zSize;
    float height;
    std::vector<std::vector<float>> altitudes;

    std::vector<std::uint16_t> indices;

    GLuint VAO;
    GLuint vertexPositionBuffer;
    GLuint normalBuffer;
    GLuint texCoordsBuffer;
    GLuint altitudeBuffer;
    GLuint EBO;

    HeightMapMaterial* material;

};