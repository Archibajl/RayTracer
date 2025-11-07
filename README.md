# RayTracer

A high-performance CUDA-accelerated ray tracer with pluggable acceleration structures for rendering STL meshes.

## Overview

This project implements a modular ray tracer with a pluggable architecture that supports multiple ray tracing algorithms. It's designed to work with STL mesh files and leverages CUDA for GPU-accelerated voxelization.

## Features

- **Pluggable Architecture**: Easily switch between different ray tracing algorithms via the `IRayTracer` interface
- **STL Mesh Loading**: Read and process STL mesh files
- **Multiple Acceleration Structures**:
  - Octree-Like Ray Tracer (implemented)
  - BVH, Brute Force (planned)
- **CUDA Support**: GPU-accelerated voxel grid generation
- **Camera System**: Configurable camera with FOV and aspect ratio settings
- **Image Output**: Render to PNG files
- **Performance Statistics**: Detailed traversal and intersection statistics

## Project Structure

```
RayTracer/
├── RayTracer.cpp              # Main entry point
├── TraceImages.h/.cpp         # Main API for rendering STL to images
├── IRayTracer.h               # Abstract ray tracer interface
├── OctreeLikeRayTracer.h/.cpp # Octree-like ray tracer implementation
├── RayTracerCommon.h          # Shared data structures (Ray, RayHit, Camera)
├── SceneCreator.h/.cpp/.cu    # Scene creation and voxel grid generation
├── STLReader.h/.cpp           # STL file parsing
├── Voxels.h                   # Voxel data structures
├── VoxelGrid.h                # Voxel grid structure
├── GeometryUtils.h/.cpp       # Geometry intersection utilities
├── Logger.h                   # Logging utilities
└── ImageSaver.h               # Image output utilities
```

## Key Components

### TraceImages
The main API for rendering STL meshes to images:
- Loads STL files
- Generates voxel grids using CUDA
- Creates appropriate ray tracer based on selected method
- Saves rendered output to PNG files

### IRayTracer Interface
Abstract interface that all ray tracing implementations must implement:
- `render()`: Renders a full frame
- `traceRay()`: Traces a single ray through the scene
- `getMethodName()`: Returns method name for logging

### OctreeLikeRayTracer
Current ray tracing implementation:
- Uses voxel grid acceleration structure
- Adaptive traversal algorithm
- Efficient for various scene types
- Includes detailed performance statistics logging

### VoxelGrid
CUDA-accelerated acceleration structure:
- Divides 3D space into uniform voxels
- Stores triangle indices per voxel
- GPU-accelerated construction
- Enables fast ray traversal

### Common Data Structures (RayTracerCommon.h)

#### Ray
Represents a ray with origin and direction for intersection testing.

#### RayHit
Stores ray intersection results:
- Hit status (hit/miss)
- Intersection point and distance
- Surface normal
- Triangle index

#### Camera
Configurable camera with:
- Position and look-at target
- Up vector for orientation
- Field of view (FOV)
- Aspect ratio

#### Vec3
Basic 3D vector structure for positions, directions, and normals.

## Build Requirements

- Visual Studio 2019 or later
- CUDA Toolkit (compatible with your GPU)
- C++17 or later

## Building the Project

1. Open `RayTracer.sln` in Visual Studio
2. Ensure CUDA Toolkit is properly installed and configured
3. Select your build configuration (Debug/Release)
4. Build the solution (Ctrl+Shift+B)

## Usage

### Basic Usage

```cpp
#include "TraceImages.h"

int main() {
    TraceImages tracer;

    // Render STL to image using default method (Voxel Grid DDA)
    tracer.TraceImage("model.stl", "output.png");

    return 0;
}
```

### Specifying Ray Tracing Method

```cpp
#include "TraceImages.h"

int main() {
    TraceImages tracer;

    // Use Octree-like method
    tracer.TraceImage("bunny.stl", "bunny_octree.png", RayTracingMethod::OCTREE);

    // Future: Use BVH method (when implemented)
    // tracer.TraceImage("bunny.stl", "bunny_bvh.png", RayTracingMethod::BVH);

    return 0;
}
```


## Architecture

### Pluggable Ray Tracing System

The project uses an interface-based design that allows easy addition of new ray tracing algorithms:

```
IRayTracer (interface)
    ↑
    ├── OctreeLikeRayTracer (implemented)
    ├── BVHRayTracer (future)
    └── BruteForceRayTracer (future)
```

### Current Implementation: OctreeLike Ray Tracer

#### Voxelization
The mesh is subdivided into a regular 3D grid. Each voxel stores indices of triangles that intersect it, enabling efficient spatial queries.

#### Ray Traversal
Uses an adaptive traversal algorithm to navigate the voxel grid:
1. Ray enters the grid at a calculated entry point
2. Step through voxels along the ray direction
3. Test only triangles in occupied voxels
4. Return first intersection found

#### Ray-Triangle Intersection
Implements efficient ray-triangle intersection testing using geometric algorithms.

## Recent Development

Recent updates include:
- **Octree-Like Ray Tracer**: Efficient ray tracing implementation
- **Pluggable Architecture**: `IRayTracer` interface enables easy method comparison
- **Performance Statistics**: Detailed metrics for ray tracing performance
- **Geometry Utilities**: Efficient intersection algorithms

## Performance Considerations

### OctreeLike Ray Tracer
- **Voxel Resolution**: Affects both memory usage and performance
  - Too coarse: Many triangles per voxel, slower intersection tests
  - Too fine: More memory usage, longer traversal time
  - Optimal resolution depends on mesh complexity and density
- **Adaptive Traversal**: Efficient ray traversal through voxel grid
- **Best For**: Various scene types with good balance between speed and memory usage

### Future Methods (Planned)
- **BVH**: Better for complex scenes with varying triangle density
- **Brute Force**: Simple baseline for performance comparison

## License

[Add your license information here]

## Contributing

[Add contribution guidelines if applicable]

## Contact

[Add contact information if desired]
