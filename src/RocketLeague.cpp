#include "RocketLeague.hpp"
#include "Shader.hpp"
#include "MeshMaterial.hpp"
#include "Mesh.hpp"

extern bool switchCamera;

RocketLeague::RocketLeague(float screenRatio) :
    car{scene.getRootTransformTree()->addChild(Transform().setScale(glm::vec3(0.01f)))}
{
    init(screenRatio);
}

RocketLeague::~RocketLeague() {
    destroy();
}

glm::vec3 RocketLeague::getGravity() const {
    return gravity;
}

void RocketLeague::update(float delta) {
    car.updateAnimations(delta);
    if(!switchCamera)
        scene.getCamera().update(car.getPosition(), car.getRotation());
    else
        scene.getCamera().updateCamera2(balls[0].getPosition(), car.getPosition(), car.getRotation());
    scene.getCamera().fovEffectBoosting(car.isTurboBoosting());
    updatePhysics(delta);
}

Ball& RocketLeague::addBall() {
    return balls.emplace_back(scene.getRootTransformTree()
        ->addChild(Transform().setScale(glm::vec3(0.008F)))
        ->addObject(ballModel)
    );
}

std::vector<Ball>& RocketLeague::getBalls() {
    return balls;
}

void RocketLeague::render() {
    scene.render();
}

Scene& RocketLeague::getScene() {
    return scene;
}

Car& RocketLeague::getCar() {
    return car;
}

void RocketLeague::updatePhysics(float delta) {
    car.updatePhysics(delta);
    for (Ball& ball : balls) {
        ball.updatePhysics(delta);
    }
}

void RocketLeague::initModels() {
    Shader vertexShader{"./shaders/mesh/phong.vert", Shader::Type::VERTEX};
    Shader fragmentShader{"./shaders/mesh/phong.frag", Shader::Type::FRAGMENT};

    meshShaderProgram.attachShader(vertexShader);
    meshShaderProgram.attachShader(fragmentShader);
    meshShaderProgram.link();

    skyboxMaterial = new MeshMaterial{
        &meshShaderProgram,
        loadTexture("textures/savanna_digital.jpg"),
        {1.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
    };

    skyboxModel = new Mesh{Mesh::loadFromFile("./models/basic_skybox_3d.obj", skyboxMaterial)};

    carMaterial = new MeshMaterial{
        &meshShaderProgram,
        loadTexture("models/octane-rocket-league-car/textures/Octane_Chasis_Map.png"),
        {0.15f, 0.15f, 0.15f},
        {1.0f, 1.0f, 1.0f},
        {0.3f, 0.3f, 0.3f},
        16.0f
    };

    carModel = new Mesh{Mesh::loadFromFile("./models/Octane.obj", carMaterial)};

    arenaMaterial = new MeshMaterial{
        &meshShaderProgram,
        loadTexture("models/arena.png"),
        {0.15f, 0.15f, 0.15f},
        {1.0f, 1.0f, 1.0f},
        {0.3f, 0.3f, 0.3f},
        16.0f
    };

    arenaModel = new Mesh{Mesh::loadFromFile("./models/arena0.obj", arenaMaterial)};

    ballMaterial = new MeshMaterial{
        &meshShaderProgram,
        loadTexture("textures/balltexture.png"),
        {0.15f, 0.15f, 0.15f},
        {1.0f, 1.0f, 1.0f},
        {0.3f, 0.3f, 0.3f},
        16.0f
    };

    ballModel = new Mesh{Mesh::loadFromFile("./models/ball.obj", ballMaterial)};
}

void RocketLeague::init(float screenRatio) {
    scene.getLightSource() = LightSource{
        {0.0f, 6.0f, 0.0f},
        {1.0f, 1.0f, 1.0f}
    };
    
    scene.getCamera() = Camera{
        {-1.0f, 5.0f, 0.0f},
        glm::vec3{0.0f, 0.0f, 0.0f},
        60.0f,
        screenRatio
    };

    initModels();

    scene.getRootTransformTree()
        ->addChild(Transform().setScale(glm::vec3(2.0f)))
            ->addObject(arenaModel)
            ->getParent()
        ->addChild(Transform().setTranslation(glm::vec3(0.0f, 1000.0f, 0.0f)).setScale(glm::vec3(0.001f)))
            ->addObject(skyboxModel);

    car.getTransformTree()
        ->addObject(carModel);

    car
        .setMass(10.0f)
        .setPosition({0.0f, 5.0f, 0.0f});

    addBall()
        .setPosition({0.0f, 10.0f, 0.0f});
}

void RocketLeague::destroy() {
    delete skyboxMaterial;
    delete carMaterial;
    delete arenaMaterial;
    delete ballMaterial;

    delete skyboxMaterial;
    delete carModel;
    delete arenaModel;
    delete ballModel;
    meshShaderProgram.destroy();
}