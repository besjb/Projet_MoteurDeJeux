#include "HeightMap.hpp"
#include "Scene.hpp"

#include <stb_image.h>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

HeightMap::HeightMap(HeightMapMaterial* material) :
    xSize{1},
    zSize{1},
    height{1.0f},
    altitudes{{0.0f}},
    material{material}
{
    init();
}

HeightMap::HeightMap(HeightMapMaterial* material, unsigned xSize, unsigned zSize, float height) :
    xSize{xSize},
    zSize{zSize},
    height{height},
    altitudes{std::vector<std::vector<float>>(xSize, std::vector<float>(zSize, 0.0f))},
    material{material}
{
    init();
}

HeightMap::~HeightMap() {
    destroy();
}

std::vector<float>& HeightMap::operator[](std::size_t position) {
    return altitudes[position];
}

const std::vector<float>& HeightMap::operator[](std::size_t position) const {
    return altitudes[position];
}

HeightMap HeightMap::loadFromFile(std::string_view fileName, HeightMapMaterial* material, unsigned xSize, unsigned zSize, float height) {
    int imageWidth;
    int imageHeight;
    int comp;
    std::uint8_t* image{stbi_load(fileName.data(), &imageWidth, &imageHeight, &comp, STBI_grey)};

    HeightMap heightMap{material, xSize, zSize, height};
    for (unsigned i{}; i < xSize; ++i) {
        for (unsigned j{}; j < zSize; ++j) {
            unsigned x{i * imageWidth / xSize + imageWidth / xSize / 2};
            unsigned z{j * imageHeight / zSize + imageHeight / zSize / 2};

            heightMap[i][j] = image[x + imageWidth * z] / 255.0f;
        }
    }

    stbi_image_free(image);
    heightMap.updateBuffers();
    return heightMap;
}

void HeightMap::init() {
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &vertexPositionBuffer);

    glGenBuffers(1, &normalBuffer);
    
    glGenBuffers(1, &texCoordsBuffer);

    glGenBuffers(1, &altitudeBuffer);

    glGenBuffers(1, &EBO);
}

void HeightMap::destroy() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &vertexPositionBuffer);
    glDeleteBuffers(1, &normalBuffer);
    glDeleteBuffers(1, &texCoordsBuffer);
    glDeleteBuffers(1, &altitudeBuffer);
    glDeleteBuffers(1, &EBO);
}

Material* HeightMap::getMaterial() {
    return material;
}

void HeightMap::render() {
    material->getShaderProgram()->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, material->getGrassTextureLocation());

    glActiveTexture(GL_TEXTURE0 + 1);
    glBindTexture(GL_TEXTURE_2D, material->getRockTextureLocation());

    glActiveTexture(GL_TEXTURE0 + 2);
    glBindTexture(GL_TEXTURE_2D, material->getSnowRocksTextureLocation());
    
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, vertexPositionBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(1);
    
    glBindBuffer(GL_ARRAY_BUFFER, texCoordsBuffer);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(2);
    
    glBindBuffer(GL_ARRAY_BUFFER, altitudeBuffer);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(float), nullptr);
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, nullptr);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void HeightMap::updateBuffers() {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> texCoords;
    std::vector<float> altitude;
    
    glm::vec3 startPosition{-0.5f * xSize, 0.0f, -0.5f * zSize};

    float scaleX{xSize / (xSize - 1.0f)};
    float scaleZ{zSize / (zSize - 1.0f)};
    std::size_t previousSize(vertices.size());
    for (unsigned i{}; i < xSize; ++i) {
        for (unsigned j{}; j < zSize; ++j) {
            vertices.push_back(startPosition + glm::vec3{i * scaleX, altitudes[i][j] * height, j * scaleZ});

            altitude.push_back(altitudes[i][j]);

            texCoords.push_back(glm::vec2{static_cast<float>(i) / (xSize - 1), static_cast<float>(j) / (zSize - 1)});

            float x{altitudes[std::min<int>(i + 1, xSize - 1)][j] - altitudes[std::max<int>(i - 1, 0)][j]};

            float z{altitudes[i][std::min<int>(j + 1, zSize - 1)] - altitudes[i][std::max<int>(j - 1, 0)]};

            normals.push_back(glm::normalize(glm::cross(glm::vec3{0.0f, z * height, 2.0f}, glm::vec3{2.0f, x * height, 0.0f})));
        }
    }

    for (int i{}; i < xSize - 1; ++i) {
        for (int j{}; j < zSize - 1; ++j) {
            std::size_t index{previousSize + j + i * xSize};
            indices.push_back(index);
            indices.push_back(index + 1);
            indices.push_back(index + xSize);

            indices.push_back(index + 1);
            indices.push_back(index + xSize + 1);
            indices.push_back(index + xSize);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertexPositionBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * normals.size(), normals.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, texCoordsBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * texCoords.size(), texCoords.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, altitudeBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * altitude.size(), altitude.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * indices.size(), indices.data(), GL_STATIC_DRAW);
}

/*glm::vec3 getHeightAt(float x, float z) const {
    int xIndex{static_cast<int>(std::round(x))};
    int zIndex{static_cast<int>(std::round(z))};
    if (xIndex == x && zIndex == z) {
        return altitudes[xIndex][zIndex];
    }
    int xIndex2{x > xIndex ? xIndex + 1 : xIndex - 1};
    int zIndex2{z > zIndex ? zIndex + 1 : zIndex - 1};
    int altitude1{std::lerp(altitudes[xIndex][zIndex], std::lerp(altitudes[xIndex2][zIndex])};
    return {x, , z};

}*/