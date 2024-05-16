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

#include "RocketLeague.hpp"

/*#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>*/

glm::vec3 cameraMovement;
Mesh* globalEarthMesh;
Mesh* globalCarMesh;
float deltaT{};
bool impulseModel{false};

RocketLeague* globalRocketLeague;

bool captureMouse{false};
bool switchCamera{false}; // false = car camera / true = ball camera

void cursorCallback(GLFWwindow* window, double x, double y) {
    if (!captureMouse) {
        Scene* scene{static_cast<Scene*>(glfwGetWindowUserPointer(window))};

        constexpr static float sensitivity{5.0f};

        glm::quat leftRightRotation{glm::angleAxis(glm::radians<float>(deltaT * x * sensitivity), glm::vec3{0.0f, 1.0f, 0.0f})};
        glm::quat upDownRotation{glm::angleAxis(glm::radians<float>(deltaT * y * sensitivity), glm::vec3{1.0f, 0.0f, 0.0f})};
        
        //scene->getCamera().setRotation(upDownRotation * scene->getCamera().getRotation() * leftRightRotation);
        glfwSetCursorPos(window, 0, 0);
    }
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
    glfwSetMouseButtonCallback(window, mouseCallback);

    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPos(window, 0, 0);
    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    glfwFocusWindow(window);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    RocketLeague rocketLeague(static_cast<float>(windowWidth) / windowHeight);
    globalRocketLeague = &rocketLeague;

    glfwSetWindowUserPointer(window, &rocketLeague.getScene());

    double lastTime{std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count()};
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        double currentTime{std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count()};
        deltaT = static_cast<float>(currentTime - lastTime);
        
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        rocketLeague.update(deltaT);
        rocketLeague.render();

        /*ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Interface");
        //ImGui::ShowDemoWindow();
        ImGui::Separator();
        ImGui::Text("Welcome to this TP about Cameras! Press escape to close the exe");
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());*/

        glfwSwapBuffers(window);

        lastTime = currentTime;
    }

    /*ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();*/

    glfwTerminate();
    //meshShaderProgram.destroy();
    
    return 0;
}