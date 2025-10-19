# Vulkan Ray Tracer (Optional)

This is an **optional** Vulkan-based GPU ray tracer. The CPU ray tracer already works!

## Files
- `VulkanRayTracer.h/cpp` - Simple Vulkan ray tracer class (excluded from build)
- `VulkanExample.cpp` - Usage example (excluded from build)
- `shaders/raytracer.comp` - GLSL compute shader

## Requirements
To use the Vulkan ray tracer, you need:
1. [Vulkan SDK](https://vulkan.lunarg.com/) installed
2. GPU with Vulkan support

## How to Enable

### Step 1: Install Vulkan SDK
Download from: https://vulkan.lunarg.com/

### Step 2: Compile the Shader
Open a command prompt in the `shaders` folder and run:
```bash
glslangValidator -V raytracer.comp -o raytracer.spv
```

Or using glslc:
```bash
glslc raytracer.comp -o raytracer.spv
```

### Step 3: Include Files in Build
1. Right-click `VulkanRayTracer.cpp` in Visual Studio
2. Properties → Excluded From Build → Set to "No"
3. Do the same for `VulkanRayTracer.h`

### Step 4: Link Vulkan
Add to your linker dependencies:
```
vulkan-1.lib
```

And add Vulkan SDK include/lib paths if needed.

## Usage
```cpp
#include "VulkanRayTracer.h"

SimpleVulkanRT rt(800, 600);
rt.init();

float triangles[] = {
    -1.0f, -1.0f, 0.0f,  // v0
     1.0f, -1.0f, 0.0f,  // v1
     0.0f,  1.0f, 0.0f   // v2
};

rt.loadTriangles(triangles, 1);
rt.render();
rt.saveImage("output.ppm");
```

## Note
**Your CPU ray tracer (RayTracerEngine) already works perfectly!**

This Vulkan version is just an optional alternative if you want GPU acceleration. The CPU version in `RayTracerEngine.cpp` is fully functional and doesn't require Vulkan SDK.
