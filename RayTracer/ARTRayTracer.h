#pragma once

#include "IRayTracer.h"
#include "SceneCreator.h"
#include "VectorMath.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>

/**
 * Voxel Grid Ray Tracer (Simplified Array-Based Traversal)
 *
 * Originally based on Fujimoto's ARTS 3DDDA algorithm, now simplified to use
 * straightforward nested for-loops for easier understanding and debugging.
 *
 * Current implementation:
 * - Simple triple-nested for-loop traversal (z, y, x order)
 * - Tests all voxels in the grid sequentially
 * - Easy to understand and modify
 * - Good for small to medium grids (< 100x100x100)
 *
 * Trade-off: Simplicity over speed
 * - Original 3DDDA helper functions kept for reference (marked UNUSED)
 * - Can be restored for performance-critical applications
 */
class ARTRayTracer : public IRayTracer {
public:
    ARTRayTracer(const VoxelGrid& grid);
    ~ARTRayTracer() override = default;

    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) override;

    cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray) override;

    std::string getMethodName() const override { return "Voxel Grid (Simple)"; }

private:
    const VoxelGrid& voxelGrid;

    // Core ARTS/3DDDA algorithm
    cg_datastructures::RayHit traverse3DDDA(const cg_datastructures::Ray& ray);

    // 3DDDA traversal helper methods
    struct VoxelTraversalState {
        int ix, iy, iz;           // Current voxel indices
        int stepX, stepY, stepZ;  // Step direction
        float tMaxX, tMaxY, tMaxZ; // Ray parameter at next voxel boundary
        float tDeltaX, tDeltaY, tDeltaZ; // Parameter increment per voxel
    };

    VoxelTraversalState initializeVoxelTraversal(const cg_datastructures::Ray& ray,
                                                  float tMin) const;
    void computeVoxelStepDirection(const cg_datastructures::Ray& ray,
                                   VoxelTraversalState& state) const;
    void computeTMaxValues(const cg_datastructures::Ray& ray,
                           VoxelTraversalState& state) const;
    void advanceToNextVoxel(VoxelTraversalState& state) const;
    bool testVoxelTriangles(const cg_datastructures::Ray& ray,
                            int ix, int iy, int iz,
                            cg_datastructures::RayHit& result);

    // Voxel traversal helper functions
    cg_datastructures::Vec3 computeGridEntryPoint(const cg_datastructures::Ray& ray, float tMin) const;
    int worldToVoxelIndex(float worldCoord, float gridMin, float voxelSize, int maxIndex) const;
    float computeAxisTMax(float rayOrigin, float rayDirection,
                         float gridMin, float voxelSize, int voxelIndex) const;

    // Rendering helper methods
    cg_datastructures::Vec3 computeShading(const cg_datastructures::RayHit& hit, float v) const;
    cg_datastructures::Vec3 renderPixel(const cg_datastructures::Camera& camera,
                                       int x, int y, int width, int height);
    void resetRenderingStatistics();
    void logProgressUpdate(int currentRow, int totalRows) const;
    void logRenderingStatistics(int hitCount, int totalRays) const;

    // Helper functions
    cg_datastructures::Ray generateRay(const cg_datastructures::Camera& camera, float u, float v);
    bool rayBoxIntersection(const cg_datastructures::Ray& ray,
                           const cg_datastructures::Vec3& boxMin,
                           const cg_datastructures::Vec3& boxMax,
                           float& tMin, float& tMax);
    bool rayTriangleIntersection(const cg_datastructures::Ray& ray,
                                const cg_datastructures::Triangle& tri,
                                float& t, float& u, float& v);

    // Statistics
    mutable size_t voxelsTraversed = 0;
    mutable size_t trianglesTests = 0;
};
