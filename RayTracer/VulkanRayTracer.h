#pragma once

// Ultra-simple Vulkan ray tracer
// Just the basics - no frills!

#include <vulkan/vulkan.h>
#include <string>

class SimpleVulkanRT {
public:
    SimpleVulkanRT(int width, int height);
    ~SimpleVulkanRT();

    bool init();                                    // Setup Vulkan
    void loadTriangles(float* triangles, int count); // Load mesh
    void render();                                   // Ray trace
    bool saveImage(const char* filename);           // Save PPM

private:
    int w, h;
    VkInstance instance;
    VkDevice device;
    VkQueue queue;
    VkCommandPool cmdPool;
    VkBuffer triangleBuf, outputBuf;
    VkDeviceMemory triangleMem, outputMem;
    int triCount;
};
