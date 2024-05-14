#pragma once

#include "Scene.hpp"
#include "Mesh.hpp"

#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

void mouseCallback(GLFWwindow* window, int button, int action, int mods);
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

// CAR
void moveForwardCar(TransformTree* t);
void moveBackwardCar(TransformTree* t);
void moveLeftCar(TransformTree* t);
void moveRightCar(TransformTree* t);

// CAMERA
/*
void moveForwardCamera();
void moveBackwardCamera();
void moveLeftCamera();
void moveRightCamera();
void moveUpCamera();
void moveDownCamera();
*/