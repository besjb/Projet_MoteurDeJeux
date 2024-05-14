#include "Controls.hpp"
#include "Scene.hpp"
#include "Mesh.hpp"
#include "RocketLeague.hpp"

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

extern bool captureMouse;
extern Mesh* globalCarMesh;
extern glm::vec3 cameraMovement;

extern RocketLeague* globalRocketLeague;

glm::vec3 mouv = glm::vec3(0.,0.,0.);

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    Scene* scene{static_cast<Scene*>(glfwGetWindowUserPointer(window))};
    static TransformTree* tree;
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                return;
            case GLFW_KEY_W:
                moveBackwardCamera();
                return;
            case GLFW_KEY_S:
                moveForwardCamera();
                return;
            case GLFW_KEY_A:
                moveRightCamera();
                return;
            case GLFW_KEY_D:
                moveLeftCamera();
                return;
            case GLFW_KEY_E:
                moveUpCamera();
                return;
            case GLFW_KEY_Q:
                moveDownCamera();
                return;
            case GLFW_KEY_UP:
                globalRocketLeague->getCar().setForwardAcceleration(globalRocketLeague->getCar().getForwardAcceleration() + 8.0f);
                return;
            case GLFW_KEY_DOWN:
                globalRocketLeague->getCar().setForwardAcceleration(globalRocketLeague->getCar().getForwardAcceleration() - 8.0f);
                return;
            case GLFW_KEY_LEFT:
                moveLeftCar(tree);
                return;
            case GLFW_KEY_RIGHT:
                moveRightCar(tree);
                return;
            /*case GLFW_KEY_SPACE: {
                //tree = scene->getRootTransformTree()->addChild({{}, glm::identity<glm::quat>(), glm::vec3{0.2f, 0.2f, 0.2f}})->addObject(globalCarMesh);
                //glm::vec3 forward{glm::vec3{0.0f, 0.0f, -1.0f} * scene->getCamera().getRotation()};
                /*tree = scene->getRootTransformTree()->addChild({{}, glm::identity<glm::quat>(), glm::vec3{0.4f, 0.2f, 0.8f}})->addObject(globalCarMesh);
                glm::vec3 forward{glm::vec3{0.0f, 0.0f, -1.0} * scene->getCamera().getRotation()};

                RigidBody::Ref cube{RigidBody::make(
                    tree,
                    &cubeCollider,
                    1.0f,
                    PhysicsMaterial{0.5f, 0.5f, 0.0f, 0.0f},
                    0,
                    false,
                    scene->getCamera().getPosition(),
                    8.0f * forward,
                    false
                    //glm::conjugate(scene->getCamera().getRotation()) * glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3{0.5f, 0.0f, 1.0f}))
                    //glm::vec3{2.0f, 2.0f, 0.0f}
                )};

                physicsEnginePointer->addRigidBody(cube);
                return;
            }*/
            case GLFW_KEY_LEFT_CONTROL:
                if (captureMouse) {
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                    //glfwSetCursorPosCallback(window, cursorCallback);
                    captureMouse = false;
                }
                else {
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                    //glfwSetCursorPosCallback(window, nullptr);
                    captureMouse = true;
                }
            /*case GLFW_KEY_TAB:
                impulseModel = !impulseModel;
                return;*/
            default:
                return;
        }
    }
    else if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W:
                moveForwardCamera();
                return;
            case GLFW_KEY_S:
                moveBackwardCamera();
                return;
            case GLFW_KEY_A:
                moveLeftCamera();
                return;
            case GLFW_KEY_D:
                moveRightCamera();
                return;
            case GLFW_KEY_E:
                moveDownCamera();
                return;
            case GLFW_KEY_Q:
                moveUpCamera();
                return;
            case GLFW_KEY_UP:
                globalRocketLeague->getCar().setForwardAcceleration(globalRocketLeague->getCar().getForwardAcceleration() - 8.0f);
                return;
            case GLFW_KEY_DOWN:
                globalRocketLeague->getCar().setForwardAcceleration(globalRocketLeague->getCar().getForwardAcceleration() + 8.0f);
                return;
            case GLFW_KEY_LEFT:
                mouv = glm::vec3(0.,0.,0.);
                return;
            case GLFW_KEY_RIGHT:
                mouv = glm::vec3(0.,0.,0.);
                return;
            default:
                return;
        }
    }
}

/* ===== CAR ===== */

void moveForwardCar(TransformTree* t){
    mouv.z += 3.0; 
    t->transform.translate(mouv);
}

void moveBackwardCar(TransformTree* t){
    mouv.z -= 3.0; 
    t->transform.translate(mouv);
}
void moveLeftCar(TransformTree* t){
    mouv.x -= 3.0; 
    t->transform.translate(mouv);
}

void moveRightCar(TransformTree* t){
    mouv.x += 3.0; 
    t->transform.translate(mouv);
}

/* ===== CAMERA ===== */

void moveForwardCamera(){
    cameraMovement.z += 3.0;
}

void moveBackwardCamera(){
    cameraMovement.z -= 3.0;
}

void moveLeftCamera(){
    cameraMovement.x += 3.0;
}

void moveRightCamera(){
    cameraMovement.x -= 3.0;
}

void moveUpCamera(){
    cameraMovement.y += 3.0;
}

void moveDownCamera(){
    cameraMovement.y -= 3.0;
}