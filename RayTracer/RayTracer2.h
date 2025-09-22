#include <iostream>
#include <cmath>
//#include "stl_reader.h"
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>

#pragma once
class RayTracer2
{

    
    //// Define the window dimensions and aspect ratio
    //const int WIDTH = 800;
    //const int HEIGHT = 600;
    //const float ASPECT_RATIO = (float)WIDTH / (float)HEIGHT;

    //// Define the camera position, view angle, and perspective
    //const glm::vec3 CAMERA_POSITION(0.0f, 0.0f, 5.0f);
    //const float VIEW_ANGLE = 60.0f;
    //const float PERSPECTIVE = 1.0f;

    //// Define the scene dimensions and objects
    //const glm::vec3 SCENE_DIMENSIONS(10.0f, 10.0f, 10.0f);
    //const glm::vec3 OBJECT_POSITIONS[3] = { glm::vec3(-2.0f, -2.0f, 2.0f), glm::vec3(2.0f, -2.0f, 2.0f), glm::vec3(0.0f, 4.0f, 2.0f) };
    //const float OBJECT_COLORS[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };

    //// Define the shaders and textures
    //const std::string VERTEX_SHADER = "shaders/vertex.glsl";
    //const std::string FRAGMENT_SHADER = "shaders/fragment.glsl";
    //const GLuint TEXTURE_ID = 0;

    //// Define the rendering pipeline
    //GLFWwindow* window;
    //GLuint vertexBufferObject;
    //GLuint shaderProgram;
    //GLuint vertexArrayObject;
    //GLuint textureId;

    //void initOpenGL() {
    //    // Initialize GLFW
    //    glfwInit();

    //    // Create a window and its OpenGL context
    //    window = glfwCreateWindow(WIDTH, HEIGHT, "Ray Tracer", NULL, NULL);
    //    if (!window) {
    //        std::cerr << "Failed to create GLFW window" << std::endl;
    //        glfwTerminate();
    //        return;
    //    }

    //    // Make the context current
    //    glfwMakeContextCurrent(window);

    //    // Load the OpenGL functions
    //    if (!gladLoadGL()) {
    //        std::cerr << "Failed to initialize GLAD" << std::endl;
    //        return;
    //    }
    //}

    //void createScene() {
    //    // Create a scene graph for the objects in the scene
    //    SceneGraph* scene = new SceneGraph();
    //    scene->addObject(OBJECT_POSITIONS[0], OBJECT_COLORS[0]);
    //    scene->addObject(OBJECT_POSITIONS[1], OBJECT_COLORS[1]);
    //    scene->addObject(OBJECT_POSITIONS[2], OBJECT_COLORS[2]);

    //    // Create a vertex buffer object for the scene
    //    glGenVertexArrays(1, &vertexArrayObject);
    //    glBindVertexArray(vertexArrayObject);
    //    glGenBuffers(1, &vertexBufferObject);
    //    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);

    //    // Load the scene geometry into the vertex buffer object
    //    GLfloat* vertices = new GLfloat[3 * 3];
    //    for (int i = 0; i < 3; i++) {
    //        for (int j = 0; j < 3; j++) {
    //            vertices[i * 3 + j] = OBJECT_POSITIONS[i][j];
    //        }
    //    }

    //    // Upload the data to the GPU
    //    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * 3, vertices, GL_STATIC_DRAW);
    //    delete[] vertices;
    //}

    //void createShaders() {
    //    // Create a vertex shader and fragment shader for the scene
    //    Shader* vertexShader = new Shader(GL_VERTEX_SHADER, VERTEX_SHADER);
    //    Shader* fragmentShader = new Shader(GL_FRAGMENT_SHADER, FRAGMENT_SHADER);

    //    // Create a program for the shaders
    //    shaderProgram = glCreateProgram();
    //    glAttachShader(shaderProgram, vertexShader->getId());
    //    glAttachShader(shaderProgram, fragmentShader->getId());
    //    glLinkProgram(shaderProgram);
    //}

    //void createTextures() {
    //    // Create a texture for the scene
    //    textureId = 0;

    //    // Load the texture data into the GPU
    //    GLubyte* imageData = new GLubyte[3 * WIDTH * HEIGHT];
    //    for (int i = 0; i < WIDTH * HEIGHT; i++) {
    //        int x = i % WIDTH;
    //        int y = i / WIDTH;

    //        // Calculate the color of each pixel in the image, based on its position and the objects in the scene
    //        float red = (float)x / (float)(WIDTH - 1);
    //        float green = (float)y / (float)(HEIGHT - 1);
    //        float blue = sin(red * 0.5f + green * 0.5f);

    //        // Store the color of each pixel in the texture data
    //        imageData[3 * i] = red * 255;
    //        imageData[3 * i + 1] = green * 255;
    //        imageData[3 * i + 2] = blue * 255;
    //    }

    //    // Upload the texture data to the GPU
    //    glGenTextures(1, &textureId);
    //    glBindTexture(GL_TEXTURE_2D, textureId);
    //    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, WIDTH, HEIGHT, 0, GL_BGR, GL_UNSIGNED_BYTE, imageData);
    //    delete[] imageData;
    //}

    //void renderScene() {
    //    // Clear the color and depth buffers
    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //    // Set up the viewport and perspective matrices
    //    glViewport(0, 0, WIDTH, HEIGHT);
    //    glMatrixMode(GL_PROJECTION);
    //    glLoadIdentity();
    //    gluPerspective(VIEW_ANGLE, ASPECT_RATIO, 0.1f, 100.0f);

    //    // Set up the modelview matrix for the camera
    //    glMatrixMode(GL_MODELVIEW);
    //    glLoadIdentity();
    //    gluLookAt(CAMERA_POSITION.x, CAMERA_POSITION.y, CAMERA_POSITION.z, OBJECT_POSITIONS[0].x, OBJECT_POSITIONS[0].y,
    //        OBJECT_POSITIONS[0].z, 0.0f, 1.0f, 0.0f);

    //    // Render the scene using ray tracing
    //    for (int i = 0; i < WIDTH * HEIGHT; i++) {
    //        float x = (float)(i % WIDTH) / (float)(WIDTH - 1);
    //        float y = (float)(i / WIDTH) / (float)(HEIGHT - 1);

    //        // Calculate the position of the pixel in world space
    //        glm::vec3 pixelPosition(x * SCENE_DIMENSIONS.x, y * SCENE_DIMENSIONS.y, 0.0f);

    //        // Calculate the color of the pixel based on the objects in the scene
    //        glm::vec3 pixelColor = rayTrace(pixelPosition, OBJECT_POSITIONS, OBJECT_COLORS);

    //        // Set the color of the pixel in the output image
    //        glTexSubImage2D(GL_TEXTURE_2D, 0, i % WIDTH, i / WIDTH, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pixelColor);
    //    }

    //    // Render the output image to the screen
    //    glBindTexture(GL_TEXTURE_2D, textureId);
    //    glBegin(GL_QUADS);
    //    glTexCoord2f(0.0f, 0.0f);
    //    glVertex3f(-1.0f, -1.0f, 0.0f);
    //    glTexCoord2f(1.0f, 0.0f);
    //    glVertex3f(1.0f, -1.0f, 0.0f);
    //    glTexCoord2f(1.0f, 1.0f);
    //    glVertex3f(1.0f, 1.0f, 0.0f);
    //    glTexCoord2f(0.0f, 1.0f);
    //    glVertex3f(-1.0f, 1.0f, 0.0f);
    //    glEnd();
    //}

    //void cleanup() {
    //    // Clean up any resources that were allocated during initialization
    //    glDeleteBuffers(1, &vertexBufferObject);
    //    glDeleteVertexArrays(1, &vertexArrayObject);
    //    glDeleteProgram(shaderProgram);

    //    // Close the window and terminate GLFW
    //    glfwDestroyWindow(window);
    //    glfwTerminate();
    //}

    //int main() {
    //    initOpenGL();
    //    createScene();
    //    createShaders();
    //    createTextures();
    //    renderScene();
    //    cleanup();
    //    return 0;
    //}
};

