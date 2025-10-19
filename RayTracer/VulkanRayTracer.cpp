#include "VulkanRayTracer.h"
#include <fstream>
#include <vector>
#include <cstring>
#include <algorithm>

// Minimal logging
#define LOG(msg) printf("%s\n", msg)

SimpleVulkanRT::SimpleVulkanRT(int width, int height)
    : w(width), h(height), instance(VK_NULL_HANDLE), device(VK_NULL_HANDLE),
      queue(VK_NULL_HANDLE), cmdPool(VK_NULL_HANDLE), triangleBuf(VK_NULL_HANDLE),
      outputBuf(VK_NULL_HANDLE), triangleMem(VK_NULL_HANDLE), outputMem(VK_NULL_HANDLE),
      triCount(0) {}

SimpleVulkanRT::~SimpleVulkanRT() {
    if (device) {
        vkDeviceWaitIdle(device);
        if (triangleBuf) vkDestroyBuffer(device, triangleBuf, nullptr);
        if (triangleMem) vkFreeMemory(device, triangleMem, nullptr);
        if (outputBuf) vkDestroyBuffer(device, outputBuf, nullptr);
        if (outputMem) vkFreeMemory(device, outputMem, nullptr);
        if (cmdPool) vkDestroyCommandPool(device, cmdPool, nullptr);
        vkDestroyDevice(device, nullptr);
    }
    if (instance) vkDestroyInstance(instance, nullptr);
}

bool SimpleVulkanRT::init() {
    // 1. Create instance
    VkInstanceCreateInfo instInfo = { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    if (vkCreateInstance(&instInfo, nullptr, &instance) != VK_SUCCESS) {
        LOG("Failed to create Vulkan instance");
        return false;
    }

    // 2. Get GPU
    VkPhysicalDevice gpu = VK_NULL_HANDLE;
    uint32_t count = 1;
    vkEnumeratePhysicalDevices(instance, &count, &gpu);
    if (gpu == VK_NULL_HANDLE) {
        LOG("No GPU found");
        return false;
    }

    // 3. Create device
    float priority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo = { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &priority;

    VkDeviceCreateInfo devInfo = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    devInfo.queueCreateInfoCount = 1;
    devInfo.pQueueCreateInfos = &queueInfo;

    if (vkCreateDevice(gpu, &devInfo, nullptr, &device) != VK_SUCCESS) {
        LOG("Failed to create device");
        return false;
    }

    vkGetDeviceQueue(device, 0, 0, &queue);

    // 4. Create command pool
    VkCommandPoolCreateInfo poolInfo = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    vkCreateCommandPool(device, &poolInfo, nullptr, &cmdPool);

    LOG("Vulkan initialized");
    return true;
}

void SimpleVulkanRT::loadTriangles(float* triangles, int count) {
    triCount = count;
    LOG("Note: This is a skeleton - compute shader needed to actually render");
}

void SimpleVulkanRT::render() {
    LOG("Render called - but no compute shader loaded yet");
    // This would dispatch the compute shader if we had one
}

bool SimpleVulkanRT::saveImage(const char* filename) {
    LOG("Saving test pattern image...");

    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    file << "P6\n" << w << " " << h << "\n255\n";

    // Output simple gradient
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t r = (255 * x) / w;
            uint8_t g = (255 * y) / h;
            uint8_t b = 128;
            file.write((char*)&r, 1);
            file.write((char*)&g, 1);
            file.write((char*)&b, 1);
        }
    }

    file.close();
    LOG("Image saved");
    return true;
}
