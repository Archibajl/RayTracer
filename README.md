# RayTracer

A high-performance CUDA-accelerated ray tracer with pluggable acceleration structures for rendering STL meshes.

## Overview

This project implements a modular ray tracer with a pluggable architecture that supports multiple ray tracing algorithms. It's designed to work with STL mesh files and leverages CUDA for GPU-accelerated voxelization.

## Features

- **Pluggable Architecture**: Easily switch between different ray tracing algorithms via the `IRayTracer` interface
- **STL Mesh Loading**: Read and process STL mesh files
- **Multiple Acceleration Structures**:
  - Voxel Grid DDA (implemented)
  - BVH, Octree, Brute Force (planned)
- **CUDA Support**: GPU-accelerated voxel grid generation
- **Camera System**: Configurable camera with FOV and aspect ratio settings
- **Image Output**: Render to PNG files

## Project Structure

```
RayTracer/
├── RayTracer.cpp              # Main entry point
├── TraceImages.h/.cpp         # Main API for rendering STL to images
├── IRayTracer.h               # Abstract ray tracer interface
├── VoxelDDARayTracer.h/.cpp   # Voxel grid DDA implementation
├── RayTracerCommon.h          # Shared data structures (Ray, RayHit, Camera)
├── SceneCreator.h/.cpp/.cu    # Scene creation and voxel grid generation
├── StlMesh.h/.cpp             # STL mesh data structures
├── STLReader.h/.cpp           # STL file parsing
├── Voxels.h/.cpp              # Voxel data structures
├── MeshGenerator.h/.cpp       # Mesh generation utilities
├── Logger.h/.cpp              # Logging utilities
├── RAYTRACING_METHODS.md      # Detailed documentation on ray tracing methods
└── kernel.cu                  # CUDA kernels
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

### VoxelDDARayTracer
Current default ray tracing implementation:
- Uses voxel grid acceleration structure
- DDA (Digital Differential Analyzer) traversal algorithm
- Fast for dense scenes and solid objects
- Memory intensive but predictable performance

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

    // Explicitly specify Voxel Grid DDA method
    tracer.TraceImage("bunny.stl", "bunny_voxel.png", RayTracingMethod::VOXEL_DDA);

    // Future: Use BVH method (when implemented)
    // tracer.TraceImage("bunny.stl", "bunny_bvh.png", RayTracingMethod::BVH);

    return 0;
}
```

### Advanced Usage (Custom Ray Tracer)

For more advanced usage and implementing custom ray tracing methods, see [RAYTRACING_METHODS.md](RayTracer/RAYTRACING_METHODS.md).

## Architecture

### Pluggable Ray Tracing System

The project uses an interface-based design that allows easy addition of new ray tracing algorithms:

```
IRayTracer (interface)
    ↑
    ├── VoxelDDARayTracer (implemented)
    ├── BVHRayTracer (future)
    ├── OctreeRayTracer (future)
    └── BruteForceRayTracer (future)
```

### Current Implementation: Voxel Grid DDA

#### Voxelization
The mesh is subdivided into a regular 3D grid using CUDA acceleration. Each voxel stores indices of triangles that intersect it, enabling efficient spatial queries.

#### Ray Traversal
Uses a DDA (Digital Differential Analyzer) algorithm to traverse the voxel grid:
1. Ray enters the grid at a calculated entry point
2. Step through voxels along the ray direction
3. Test only triangles in occupied voxels
4. Return first intersection found

#### Ray-Triangle Intersection
Implements the Möller–Trumbore intersection algorithm for efficient ray-triangle intersection testing.

## Recent Development

Recent updates include:
- **Pluggable Architecture**: Refactored to use `IRayTracer` interface for multiple ray tracing methods
- **Ray Tracer Implementation**: Traces rays and saves to image files
- **Memory Management**: Refactored StlMesh class to use pointers for better memory management
- **CUDA Voxelization**: GPU-accelerated voxel grid generation
- **Logging System**: Added comprehensive logging throughout the pipeline
- **Code Organization**: Cleaned up and reorganized project structure

## Performance Considerations

### Voxel Grid DDA Method
- **Voxel Resolution**: Affects both memory usage and performance
  - Too coarse: Many triangles per voxel, slower intersection tests
  - Too fine: More memory usage, longer traversal time
  - Optimal resolution depends on mesh complexity and density
- **CUDA Acceleration**: GPU-accelerated voxel grid construction provides significant speedup for large meshes
- **DDA Traversal**: Cache-friendly and predictable performance
- **Best For**: Dense meshes, solid objects, scenes with high triangle density

### Future Methods (Planned)
- **BVH**: Better for complex scenes with varying triangle density
- **Octree**: Good for sparse scenes with adaptive subdivision
- **Brute Force**: Simple baseline for performance comparison

See [RAYTRACING_METHODS.md](RayTracer/RAYTRACING_METHODS.md) for detailed comparison and implementation guide.

## License

[Add your license information here]

## Contributing

[Add contribution guidelines if applicable]

## Contact

[Add contact information if desired]
