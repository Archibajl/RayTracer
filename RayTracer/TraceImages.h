#pragma once

#include <string>
#include <vector>
#include <memory>
#include "SceneCreator.h"
#include "IRayTracer.h"

namespace cg_datastructures {
    struct Vec3;
}

/**
 * Ray tracing methods available
 */
enum class RayTracingMethod {
    VOXEL_DDA,      // Voxel grid with DDA traversal (current default)
    // Future methods:
    // BVH,          // Bounding Volume Hierarchy
    // OCTREE,       // Octree spatial subdivision
    // BRUTE_FORCE   // Direct triangle intersection (for testing)
};

class TraceImages {
public:
    /**
     * Generate an image from an STL file
     * @param gridFileLocation Path to STL file
     * @param outputFileName Output filename (e.g., "image.png")
     * @param method Ray tracing method to use
     */
    void TraceImage(std::string gridFileLocation,
                    std::string outputFileName,
                    RayTracingMethod method = RayTracingMethod::VOXEL_DDA);

private:
    void genateImageFromGrid(VoxelGrid voxelGrid,
                            std::string filename,
                            RayTracingMethod method);

    VoxelGrid generateVoxelGridFromFile(const std::string filepath, int nx, int ny, int nz);
    VoxelGrid generateVoxelGridFromMesh(StlMeshCuda mesh, int nx, int ny, int nz);
    void SaveImage(const std::string& filename,
                   const std::vector<cg_datastructures::Vec3>& pixels,
                   int width, int height);

    // Factory method to create appropriate ray tracer
    std::unique_ptr<IRayTracer> createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method);
};
