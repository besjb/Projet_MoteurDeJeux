#pragma once

#include <vector>
#include <memory>

#include "Scene.hpp"
#include "Ball.hpp"
#include "Car.hpp"
#include "ShaderProgram.hpp"
#include "Mesh.hpp"

class RocketLeague {
public:

    RocketLeague(float screenRatio);

    ~RocketLeague();
    
    glm::vec3 getGravity() const;

    void update(float delta);

    Ball& addBall();

    void render();

    Scene& getScene();

private:

    ShaderProgram meshShaderProgram;

    MeshMaterial* carMaterial;
    MeshMaterial* arenaMaterial;
    MeshMaterial* ballMaterial;

    Mesh* carModel;
    Mesh* arenaModel;
    Mesh* ballModel;

    void initModels();

    void init(float screenRatio);

    void destroy();

    Scene scene;

    void updatePhysics(float delta);

    Car car;

    std::vector<Ball> balls;

    static constexpr glm::vec3 gravity{0.0f, -0.0f, 0.0f};

};