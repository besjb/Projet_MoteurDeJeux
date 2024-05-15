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
extern bool switchCamera;

extern RocketLeague* globalRocketLeague;

glm::vec3 mouv = glm::vec3(0.,0.,0.);

void mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    Car& car = globalRocketLeague->getCar();
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            if (car.wheelsCollide()) {
                car.startJump();
            }
            else if (car.canDoubleJump()) {
                car.doubleJump();
            }
        }
        else if (action == GLFW_RELEASE && car.isJumping()) {
            car.stopJump();
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            car.startBoost();
        }
        else if (action == GLFW_RELEASE) {
            car.stopBoost();
        }
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    Car& car = globalRocketLeague->getCar();
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                return;
            case GLFW_KEY_W:
                car.setForwardAcceleration(car.getForwardAcceleration() + 14.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(0.0f, 0.0f, 15.0f));
                return;
            case GLFW_KEY_S:
                car.setForwardAcceleration(car.getForwardAcceleration() - 14.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(0.0f, 0.0f, 15.0f));
                return;
            case GLFW_KEY_A:
                car.setTurnSensitivity(car.getTurnSensitivity() + 1.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(0.0f, 15.0f, 0.0f));
                return;
            case GLFW_KEY_D:
                car.setTurnSensitivity(car.getTurnSensitivity() - 1.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(0.0f, 15.0f, 0.0f));
                return;
            case GLFW_KEY_E:
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(15.0f, 0.0f, 0.0f));
                return;
            case GLFW_KEY_Q:
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(15.0f, 0.0f, 0.0f));
                return;
            case GLFW_KEY_UP:
                //moveBackwardCamera();
                return;
            case GLFW_KEY_DOWN:
                //moveForwardCamera();
                return;
            case GLFW_KEY_LEFT:
                //moveRightCamera();
                return;
            case GLFW_KEY_RIGHT:
                //moveLeftCamera();
                return;
            case GLFW_KEY_SPACE:
                switchCamera = !switchCamera;
                return;
            case GLFW_KEY_LEFT_SHIFT:
                car.startDrifting();
                return;
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
            default:
                return;
        }
    }
    else if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W:
                car.setForwardAcceleration(car.getForwardAcceleration() - 14.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(0.0f, 0.0f, 15.0f));
                return;
            case GLFW_KEY_S:
                car.setForwardAcceleration(car.getForwardAcceleration() + 14.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(0.0f, 0.0f, 15.0f));
                return;
            case GLFW_KEY_A:
                car.setTurnSensitivity(car.getTurnSensitivity() - 1.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(0.0f, 15.0f, 0.0f));
                return;
            case GLFW_KEY_D:
                car.setTurnSensitivity(car.getTurnSensitivity() + 1.0f);
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(0.0f, 15.0f, 0.0f));
                return;
            case GLFW_KEY_E:
                car.setMovementAcceleration(car.getMovementAcceleration() - glm::vec3(15.0f, 0.0f, 0.0f));
                return;
            case GLFW_KEY_Q:
                car.setMovementAcceleration(car.getMovementAcceleration() + glm::vec3(15.0f, 0.0f, 0.0f));
                return;
            case GLFW_KEY_UP:
                //moveForwardCamera();
                return;
            case GLFW_KEY_DOWN:
                //moveBackwardCamera();
                return;
            case GLFW_KEY_LEFT:
                //moveLeftCamera();
                return;
            case GLFW_KEY_RIGHT:
                //moveRightCamera();
                return;
            case GLFW_KEY_LEFT_SHIFT:
                car.stopDrifting();
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
/*
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
*/