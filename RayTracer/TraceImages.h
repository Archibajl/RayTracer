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
    ART,            // ARTS (Accelerated Ray-Tracing System) - Fujimoto's 3DDDA method
    OCTREE,         // Octree space subdivision (Glassner 1984)
    // Future methods:
    // BVH,          // Bounding Volume Hierarchy
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

    stl_reader::StlMesh<float, unsigned int> loadStlMesh(const std::string& filepath);
    VoxelGrid generateVoxelGridFromStlMesh(const stl_reader::StlMesh<float, unsigned int>& stlMesh, int nx, int ny, int nz);
    VoxelGrid loadOrGenerateVoxelGrid(const std::string& filepath, int nx, int ny, int nz);
    void SaveImage(const std::string& filename,
                   const std::vector<cg_datastructures::Vec3>& pixels,
                   int width, int height);

    // Factory method to create appropriate ray tracer
    std::unique_ptr<IRayTracer> createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method);

    // Logging helpers
    std::string getRayTracingMethodName(RayTracingMethod method);
    void logStartInfo(const std::string& gridFileLocation, const std::string& outputFileName, RayTracingMethod method);
    void logCompletionInfo(double totalTimeSeconds, const std::string& outputFileName);
    void logErrorInfo(const std::string& gridFileLocation, const std::string& errorMessage, double timeSeconds);

    // Pipeline helpers with timing
    void renderImageWithTiming(VoxelGrid& voxelGrid, const std::string& outputFileName, RayTracingMethod method);
    std::vector<cg_datastructures::Vec3> renderWithTiming(IRayTracer* raytracer, const cg_datastructures::Camera& camera, int width, int height);

    // Camera setup
    cg_datastructures::Camera setupCamera(const VoxelGrid& voxelGrid);
};
