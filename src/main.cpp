#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "Shader.hpp"
#include "ShaderProgram.hpp"
#include "Uniform.hpp"
#include "Camera.hpp"
#include "HeightMap.hpp"
#include "Scene.hpp"
#include "HeightMapMaterial.hpp"
#include "MeshMaterial.hpp"
#include "Mesh.hpp"
#include "Utility.hpp"
#include "LightSource.hpp"
#include "Physics.hpp"
#include "Controls.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <stb_image.h>
#include <string_view>
#include <chrono>
#include <limits>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

glm::vec3 cameraMovement;
std::vector<RigidBody> rigidBodies;
Mesh* globalEarthMesh;
Mesh* globalCarMesh;
float deltaT{};
bool impulseModel{false};
Scene* scenePointer;

bool captureMouse{false};
PhysicsEngine* physicsEnginePointer;

void cursorCallback(GLFWwindow* window, double x, double y) {
    if (!captureMouse) {
        Scene* scene{static_cast<Scene*>(glfwGetWindowUserPointer(window))};

        constexpr static float sensitivity{5.0f};

        glm::quat leftRightRotation{glm::angleAxis(glm::radians<float>(deltaT * x * sensitivity), glm::vec3{0.0f, 1.0f, 0.0f})};
        glm::quat upDownRotation{glm::angleAxis(glm::radians<float>(deltaT * y * sensitivity), glm::vec3{1.0f, 0.0f, 0.0f})};
        
        scene->getCamera().setRotation(upDownRotation * scene->getCamera().getRotation() * leftRightRotation);
        glfwSetCursorPos(window, 0, 0);
    }
}

void initImgui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

int main(int argc, char** argv) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    
    GLFWwindow* window = glfwCreateWindow(1440, 900, "Graphics Engine", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW\n";
        return -1;
    }

    const GLFWvidmode* mode{glfwGetVideoMode(glfwGetPrimaryMonitor())};
    int windowWidth = mode->width;
    int windowHeight = mode->height;

    glfwSetKeyCallback(window, keyCallback);

    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPos(window, 0, 0);
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    glfwFocusWindow(window);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    Shader vertexShader{"./shaders/mesh/phong.vert", Shader::Type::VERTEX};
    Shader fragmentShader{"./shaders/mesh/phong.frag", Shader::Type::FRAGMENT};

    ShaderProgram meshShaderProgram;
    meshShaderProgram.attachShader(vertexShader);
    meshShaderProgram.attachShader(fragmentShader);
    meshShaderProgram.link();

    MeshMaterial earthMaterial{
        &meshShaderProgram,
        loadTexture("textures/solar/earth.jpg"),
        {0.15f, 0.15f, 0.15f},
        {1.0f, 1.0f, 1.0f},
        {0.3f, 0.3f, 0.3f},
        16.0f
    };

    Mesh earthMesh{Mesh::loadFromFile("./models/cube.obj", &earthMaterial)};
    globalEarthMesh = &earthMesh;

    MeshMaterial carMaterial{
        &meshShaderProgram,
        loadTexture("models/octane-rocket-league-car/textures/Octane_Chasis_Map.png"),
        {0.15f, 0.15f, 0.15f},
        {1.0f, 1.0f, 1.0f},
        {0.3f, 0.3f, 0.3f},
        16.0f
    };

    Mesh carMesh{Mesh::loadFromFile("./models/cube.obj", &carMaterial)};
    globalCarMesh = &carMesh;

    Scene scene{
        // Source de lumière
        {
            {0.0f, 6.0f, 0.0f},
            {1.0f, 1.0f, 1.0f}
        },
        // Caméra
        {
            {-9.0f, 6.0f, -9.0f},
            glm::vec3{0.0f, -4.0f, 0.0f},
            60.0f,
            static_cast<float>(windowWidth) / windowHeight
        }
    };

    scenePointer = &scene;

    glfwSetWindowUserPointer(window, &scene);

    initImgui(window);

    PhysicsEngine physicsEngine;
    physicsEnginePointer = &physicsEngine;
    ForceField::Ref gravity{ForceField::global(ForceField::uniformForceField({0.0f, -9.81f, 0.0f}))};

    physicsEngine.addForceField(gravity);

    OBBCollider planeCollider{{16.0f, 1.0f, 16.0f}};

    TransformTree* arenaTree{scene.getRootTransformTree()};
    RigidBody::Ref plane{RigidBody::make(
        arenaTree->addChild({{}, glm::identity<glm::quat>(), {16.0f, 1.0f, 16.0f}})->addObject(&earthMesh),
        &planeCollider,
        -1.0f,
        PhysicsMaterial{0.5f, 0.0f, 0.0f, 0.0f},
        0,
        true,
        glm::vec3{2.0f, 0.0f, 0.0f},
        glm::vec3{},
        true
    )};
    physicsEngine.addRigidBody(plane);

    /*RigidBody::Ref wall1{RigidBody::make(
        arenaTree->addChild({{}, glm::identity<glm::quat>(), {16.0f, 1.0f, 16.0f}})->addObject(&earthMesh),
        &planeCollider,
        -1.0f,
        PhysicsMaterial{0.0f, 0.0f, 0.0f, 0.0f},
        0,
        true,
        glm::vec3{16.0f, 0.0f, 0.0f},
        glm::vec3{0.0f, 0.0f, 0.0f},
        true,
        glm::angleAxis(glm::radians(90.0f), glm::vec3{0.0f, 0.0f, 1.0f})
    )};
    physicsEngine.addRigidBody(wall1);

    RigidBody::Ref wall2{RigidBody::make(
        arenaTree->addChild({{}, glm::identity<glm::quat>(), {16.0f, 1.0f, 16.0f}})->addObject(&earthMesh),
        &planeCollider,
        -1.0f,
        PhysicsMaterial{0.0f, 0.0f, 0.0f, 0.0f},
        0,
        true,
        glm::vec3{-16.0f, 0.0f, 0.0f},
        glm::vec3{0.0f, 0.0f, 0.0f},
        true,
        glm::angleAxis(glm::radians(90.0f), glm::vec3{0.0f, 0.0f, 1.0f})
    )};
    physicsEngine.addRigidBody(wall2);

    RigidBody::Ref wall3{RigidBody::make(
        arenaTree->addChild({{}, glm::identity<glm::quat>(), {16.0f, 1.0f, 16.0f}})->addObject(&earthMesh),
        &planeCollider,
        -1.0f,
        PhysicsMaterial{0.0f, 0.0f, 0.0f, 0.0f},
        0,
        true,
        glm::vec3{0.0f, 0.0f, 16.0f},
        glm::vec3{0.0f, 0.0f, 0.0f},
        true,
        glm::angleAxis(glm::radians(90.0f), glm::vec3{1.0f, 0.0f, 0.0f})
    )};
    physicsEngine.addRigidBody(wall3);

    RigidBody::Ref wall4{RigidBody::make(
        arenaTree->addChild({{}, glm::identity<glm::quat>(), {16.0f, 1.0f, 16.0f}})->addObject(&earthMesh),
        &planeCollider,
        -1.0f,
        PhysicsMaterial{0.0f, 0.0f, 0.0f, 0.0f},
        0,
        true,
        glm::vec3{0.0f, 0.0f, -16.0f},
        glm::vec3{0.0f, 0.0f, 0.0f},
        true,
        glm::angleAxis(glm::radians(90.0f), glm::vec3{1.0f, 0.0f, 0.0f})
    )};
    physicsEngine.addRigidBody(wall4);*/

    std::vector<glm::vec3> verts{
        glm::vec3(-1.0f, 0.05f, 0.05f),
        glm::vec3(-1.0f, 0.05f, -0.05f),
        glm::vec3(-1.0f, -0.05f, 0.05f),
        glm::vec3(-1.0f, -0.05f, -0.05f),
        glm::vec3(1.0f, 0.0f, 0.0f)
        //glm::vec3(-1.87f, 0.11f, 2.3f),
        //glm::vec3(-0.96f, -2.15f, 1.21f),
        //glm::vec3(1.47f, -1.94f, 2.55f),
        //glm::vec3(-0.64f, 1.64f, 1.0f)
    };
    std::cout << glm::to_string(geometry::convexSetAverage(verts));

    double lastTime{std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count()};
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        double currentTime{std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count()};
        deltaT = static_cast<float>(currentTime - lastTime);
        if (cameraMovement != glm::vec3{}) {
            scene.getCamera().moveRelative(deltaT * cameraMovement);
        }
        
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        physicsEngine.tick(deltaT);
        scene.render();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Interface");
        //ImGui::ShowDemoWindow();
        ImGui::Separator();
        ImGui::Text("Welcome to this TP about Cameras! Press escape to close the exe");
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        lastTime = currentTime;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    meshShaderProgram.destroy();

    glfwTerminate();
    
    return 0;
}