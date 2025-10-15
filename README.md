# RayTracer

A high-performance CUDA-accelerated ray tracer with voxel grid acceleration structure for rendering STL meshes.

## Overview

This project implements a ray tracer that uses voxelization to accelerate ray-triangle intersection tests. It's designed to work with STL mesh files and leverages CUDA for GPU acceleration where possible.

## Features

- **STL Mesh Loading**: Read and process STL mesh files
- **Voxel Grid Acceleration**: Spatial subdivision using voxel grids to accelerate ray tracing
- **Ray-Triangle Intersection**: Efficient intersection testing with DDA-like voxel traversal
- **CUDA Support**: GPU-accelerated computation for performance
- **Camera System**: Configurable camera with FOV and aspect ratio settings

## Project Structure

```
RayTracer/
├── RayTracer.cpp           # Main entry point
├── RayTracerEngine.h/.cpp  # Core ray tracing engine
├── SceneCreator.h/.cpp/.cu # Scene creation and management
├── StlMesh.h/.cpp          # STL mesh data structures
├── STLReader.h/.cpp        # STL file parsing
├── Voxels.h/.cpp           # Voxel data structures
├── MeshGenerator.h/.cpp    # Mesh generation utilities
└── kernel.cu               # CUDA kernels
```

## Key Components

### RayTracerEngine
The main rendering engine that:
- Generates rays from a camera
- Traces rays through the voxel grid
- Performs ray-triangle intersection tests
- Returns hit information including intersection point, normal, and triangle index

### VoxelGrid
Acceleration structure that:
- Divides 3D space into uniform voxels
- Stores triangle indices per voxel
- Enables fast ray traversal using DDA algorithm
- CUDA-compatible for GPU processing

### Data Structures

#### Vec3
Basic 3D vector structure for positions, directions, and normals.

#### Triangle
Stores triangle vertices (v0, v1, v2) and optional normal.

#### Voxel
CUDA-compatible structure storing:
- Occupancy status
- Triangle index range
- Density and color information

#### Camera
Configurable camera with:
- Position and look-at target
- Field of view (FOV)
- Aspect ratio

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

```cpp
// Load STL mesh
StlMeshCuda mesh = LoadSTLFile("model.stl");

// Build voxel grid
VoxelGrid grid = BuildVoxelGridFromStlMesh(mesh, nx, ny, nz);

// Create ray tracer
RayTracerEngine engine(grid);

// Setup camera
cg_datastructures::Camera camera(
    Vec3(0, 0, 10),      // position
    Vec3(0, 0, 0),       // look at
    Vec3(0, 1, 0),       // up vector
    60.0f,               // FOV in degrees
    16.0f/9.0f           // aspect ratio
);

// Render frame
auto pixels = engine.render(camera, width, height);
```

## Algorithm Details

### Voxelization
The mesh is subdivided into a regular 3D grid. Each voxel stores indices of triangles that intersect it, enabling efficient spatial queries.

### Ray Traversal
Uses a DDA (Digital Differential Analyzer) algorithm to traverse the voxel grid:
1. Ray enters the grid
2. Step through voxels along the ray direction
3. Test only triangles in occupied voxels
4. Return first intersection found

### Ray-Triangle Intersection
Implements the Möller–Trumbore intersection algorithm for efficient ray-triangle intersection testing.

## Recent Development

Recent commits have focused on:
- Refactoring StlMesh class to use pointers for better memory management
- Implementing voxel grid construction and mapping
- Updating data structures for CUDA compatibility
- Removing unused files and cleaning up the codebase

## Performance Considerations

- Voxel grid resolution affects both memory usage and performance
  - Too coarse: many triangles per voxel, slower intersection tests
  - Too fine: more memory, longer traversal
- CUDA acceleration provides significant speedup for large meshes
- DDA traversal is cache-friendly and efficient

## License

[Add your license information here]

## Contributing

[Add contribution guidelines if applicable]

## Contact

[Add contact information if desired]
