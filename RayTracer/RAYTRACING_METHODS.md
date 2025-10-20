# Ray Tracing Methods Architecture

This ray tracer supports multiple ray tracing algorithms through a pluggable architecture.

## Architecture Overview

```
IRayTracer (interface)
    ↑
    ├── VoxelDDARayTracer (implemented)
    ├── BVHRayTracer (future)
    ├── OctreeRayTracer (future)
    └── BruteForceRayTracer (future)
```

### Core Components

- **`RayTracerCommon.h`**: Common data structures (Ray, RayHit, Camera)
- **`IRayTracer.h`**: Abstract interface all ray tracers must implement
- **`VoxelDDARayTracer`**: Grid-based DDA traversal (current default)

## Available Methods

### 1. Voxel Grid DDA (Default)
- **Class**: `VoxelDDARayTracer`
- **Enum**: `RayTracingMethod::VOXEL_DDA`
- **Description**: Pre-voxelizes scene into a 3D grid and uses DDA algorithm to traverse
- **Pros**: Very fast for dense scenes, predictable performance
- **Cons**: Memory intensive, not great for sparse scenes
- **Best for**: Solid objects, dense meshes

## Using Different Methods

### In Code

```cpp
TraceImages tracer;

// Use default method (Voxel Grid DDA)
tracer.TraceImage("model.stl", "output.png");

// Explicitly specify method
tracer.TraceImage("model.stl", "output.png", RayTracingMethod::VOXEL_DDA);

// Future: Use BVH
// tracer.TraceImage("model.stl", "output.png", RayTracingMethod::BVH);
```

## Adding a New Ray Tracing Method

### Step 1: Add Enum Value

Edit `TraceImages.h`:
```cpp
enum class RayTracingMethod {
    VOXEL_DDA,
    BVH,           // Add this
    OCTREE,        // Or this
    BRUTE_FORCE    // Or this
};
```

### Step 2: Create Implementation Files

**BVHRayTracer.h**:
```cpp
#pragma once
#include "IRayTracer.h"
#include "SceneCreator.h"

class BVHRayTracer : public IRayTracer {
public:
    BVHRayTracer(const VoxelGrid& grid);
    ~BVHRayTracer() override;

    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) override;

    cg_datastructures::RayHit traceRay(
        const cg_datastructures::Ray& ray) override;

    std::string getMethodName() const override { return "BVH"; }

private:
    // Your BVH-specific data structures
    // ...
};
```

**BVHRayTracer.cpp**:
```cpp
#include "BVHRayTracer.h"
#include "Logger.h"

BVHRayTracer::BVHRayTracer(const VoxelGrid& grid) {
    LOG_INFO("[{}] Building BVH...", getMethodName());
    // Build BVH here
}

BVHRayTracer::~BVHRayTracer() = default;

cg_datastructures::RayHit BVHRayTracer::traceRay(const cg_datastructures::Ray& ray) {
    // Implement BVH traversal
    // ...
}

std::vector<cg_datastructures::Vec3> BVHRayTracer::render(
    const cg_datastructures::Camera& camera,
    int width, int height) {
    // Can reuse common rendering code or implement custom
    // ...
}
```

### Step 3: Update Factory Method

Edit `TraceImages.cpp`:
```cpp
std::unique_ptr<IRayTracer> TraceImages::createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method) {
    switch (method) {
        case RayTracingMethod::VOXEL_DDA:
            return std::make_unique<VoxelDDARayTracer>(voxelGrid);

        case RayTracingMethod::BVH:  // Add this
            return std::make_unique<BVHRayTracer>(voxelGrid);

        default:
            LOG_WARN("Unknown ray tracing method, defaulting to Voxel Grid DDA");
            return std::make_unique<VoxelDDARayTracer>(voxelGrid);
    }
}
```

### Step 4: Update Project File

Add to `RayTracer.vcxproj`:
- `<ClCompile Include="BVHRayTracer.cpp" />`
- `<ClInclude Include="BVHRayTracer.h" />`

### Step 5: Test

```cpp
tracer.TraceImage("model.stl", "output_bvh.png", RayTracingMethod::BVH);
```

## Performance Comparison

You can easily compare different methods by rendering the same scene:

```cpp
tracer.TraceImage("bunny.stl", "bunny_voxel.png", RayTracingMethod::VOXEL_DDA);
tracer.TraceImage("bunny.stl", "bunny_bvh.png", RayTracingMethod::BVH);
// Compare render times in logs
```

## Future Method Ideas

1. **BVH (Bounding Volume Hierarchy)**
   - Build tree structure of bounding boxes
   - Fast for complex scenes, good memory efficiency
   - Industry standard

2. **Octree**
   - Adaptive spatial subdivision
   - Good for sparse scenes
   - Balanced memory/speed

3. **Brute Force**
   - Test every triangle
   - Simple, good for debugging
   - Slow but guaranteed correct

4. **Uniform Grid (different from current)**
   - Simpler than DDA
   - Educational purposes

5. **Hybrid Methods**
   - Top-level BVH, bottom-level grids
   - Best of both worlds
