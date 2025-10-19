// Simple example of using VulkanRayTracer
// (This file is excluded from build)

#include "VulkanRayTracer.h"

int vulkan_example_main() {
    SimpleVulkanRT rt(800, 600);

    if (!rt.init()) {
        return -1;
    }

    // Triangle: v0, v1, v2 (9 floats total)
    float tri[] = {
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         0.0f,  1.0f, 0.0f
    };

    rt.loadTriangles(tri, 1);
    rt.render();
    rt.saveImage("vulkan_out.ppm");

    return 0;
}
