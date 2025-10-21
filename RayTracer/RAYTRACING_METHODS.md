# Ray Tracing Methods Architecture

This ray tracer supports multiple ray tracing algorithms through a pluggable architecture.

## Architecture Overview

```
IRayTracer (interface)
    ↑
    ├── VoxelDDARayTracer (implemented)
    ├── ARTRayTracer (implemented)
    ├── OctreeRayTracer (implemented)
    ├── BVHRayTracer (future)
    └── BruteForceRayTracer (future)
```

### Core Components

- **`RayTracerCommon.h`**: Common data structures (Ray, RayHit, Camera)
- **`IRayTracer.h`**: Abstract interface all ray tracers must implement
- **`VoxelDDARayTracer`**: Grid-based DDA traversal (current default)
- **`ARTRayTracer`**: ARTS (Accelerated Ray-Tracing System) - Fujimoto's 3DDDA method
- **`OctreeRayTracer`**: Adaptive octree space subdivision - Glassner's method

## Available Methods

### 1. Voxel Grid DDA (Default)
- **Class**: `VoxelDDARayTracer`
- **Enum**: `RayTracingMethod::VOXEL_DDA`
- **Description**: Pre-voxelizes scene into a 3D grid and uses DDA algorithm to traverse
- **Pros**: Very fast for dense scenes, predictable performance
- **Cons**: Memory intensive, not great for sparse scenes
- **Best for**: Solid objects, dense meshes

### 2. ARTS - Accelerated Ray-Tracing System (Implemented)
- **Class**: `ARTRayTracer`
- **Enum**: `RayTracingMethod::ART`
- **Description**: Implementation of Fujimoto's classic 3DDDA (3D Digital Differential Analyzer) from the seminal 1986 paper
- **Algorithm**: Uses uniform spatial subdivision with 3DDDA traversal
- **Pros**:
  - Performance virtually independent of object count
  - Excellent for scenes with many objects (1000+)
  - Spatial coherence optimization
  - Includes detailed performance statistics
- **Cons**: Memory intensive like other grid methods
- **Best for**: Scenes with very high object counts, complex meshes
- **Historical Significance**: One of the first practical ray tracing acceleration structures
- **Paper**: "ARTS: Accelerated Ray-Tracing System" by Akira Fujimoto, Takayuki Tanaka, and K. Iwata (IEEE CG&A, 1986)

### 3. Octree Space Subdivision (Implemented)
- **Class**: `OctreeRayTracer`
- **Enum**: `RayTracingMethod::OCTREE`
- **Description**: Adaptive octree-based space subdivision from Andrew S. Glassner's 1984 paper
- **Algorithm**: Non-uniform hierarchical subdivision based on triangle density
- **Pros**:
  - Adaptive to scene complexity
  - Efficient for non-uniformly distributed geometry
  - Automatically optimizes subdivision depth
  - Lower memory usage than uniform grids for sparse scenes
  - Includes detailed statistics (nodes visited, triangle tests)
- **Cons**:
  - Slower build time compared to uniform grids
  - Can be inefficient for uniformly distributed objects
- **Best for**:
  - Scenes with non-uniform object distribution
  - Models with detail concentrated in specific regions
  - Large scenes with sparse geometry
- **Historical Significance**: Revolutionary for 1984 - enabled ray tracing hundreds/thousands of objects
- **Paper**: "Space subdivision for fast ray tracing" by Andrew S. Glassner (IEEE CG&A, Vol. 4, no. 10, 1984, pp 15-22)

## Using Different Methods

### In Code

```cpp
TraceImages tracer;

// Use default method (Voxel Grid DDA)
tracer.TraceImage("model.stl", "output.png");

// Explicitly specify method
tracer.TraceImage("model.stl", "output_voxel.png", RayTracingMethod::VOXEL_DDA);

// Use ARTS method (Fujimoto's 3DDDA)
tracer.TraceImage("model.stl", "output_art.png", RayTracingMethod::ART);

// Use Octree method (Glassner's adaptive subdivision)
tracer.TraceImage("model.stl", "output_octree.png", RayTracingMethod::OCTREE);

// Future: Use BVH
// tracer.TraceImage("model.stl", "output_bvh.png", RayTracingMethod::BVH);
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
tracer.TraceImage("bunny.stl", "bunny_art.png", RayTracingMethod::ART);
tracer.TraceImage("bunny.stl", "bunny_octree.png", RayTracingMethod::OCTREE);
// Compare render times and statistics in logs

// Future:
// tracer.TraceImage("bunny.stl", "bunny_bvh.png", RayTracingMethod::BVH);
```

All implementations include detailed statistics logging:
- **Voxel DDA & ARTS**: Voxels/nodes traversed, triangle tests, averages per ray
- **Octree**: Octree nodes visited, triangle tests, octree structure info (total nodes, leaf nodes)

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
