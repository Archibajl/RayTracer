#pragma once

#include "IRayTracer.h"
#include "SceneCreator.h"
#include "VectorMath.h"
#include "Logger.h"
#include "GeometryUtils.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

/**
 * Voxel Grid Ray Tracer (formerly OctreeRayTracer)
 *
 * Implementation using uniform voxel grid traversal for ray tracing.
 * Uses 3D-DDA (Digital Differential Analyzer) algorithm to traverse
 * the voxel grid efficiently in ray-space order.
 *
 * Key features:
 * - Uniform grid subdivision (all voxels same size)
 * - Fast O(1) voxel lookup
 * - Predictable memory usage
 * - Efficient for uniformly distributed geometry
 * - Simple traversal algorithm
 */
class OctreeRayTracer : public IRayTracer {
public:
    OctreeRayTracer(const VoxelGrid& grid, int maxDepth = 8, int maxTrianglesPerNode = 10);
    ~OctreeRayTracer() override = default;

    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) override;

    cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray) override;

    std::string getMethodName() const override { return "VoxelGrid"; }

private:
    // Voxel traversal state structure
    struct VoxelTraversalState {
        int ix, iy, iz;           // Current voxel indices
        int stepX, stepY, stepZ;  // Step direction (+1 or -1)
        float tMaxX, tMaxY, tMaxZ; // Parameter values at next voxel boundary
        float tDeltaX, tDeltaY, tDeltaZ; // Parameter increment to cross one voxel
        float tCurrent;           // Current parameter value along ray
        cg_datastructures::Vec3 currentPosition; // Current world position along ray
    };

    const VoxelGrid& voxelGrid;

    // Voxel grid traversal
    cg_datastructures::RayHit traverseVoxelGrid(const cg_datastructures::Ray& ray);
    VoxelTraversalState initializeTraversal(const cg_datastructures::Ray& ray) const;
    bool testVoxelTriangles(const cg_datastructures::Ray& ray, int ix, int iy, int iz,
                           cg_datastructures::RayHit& result);
    void advanceToNextVoxel(VoxelTraversalState& state, const cg_datastructures::Ray& ray);
    bool isVoxelInBounds(int ix, int iy, int iz) const;

    // Rendering helper methods
    cg_datastructures::Vec3 computeShading(const cg_datastructures::RayHit& hit, float v) const;
    cg_datastructures::Vec3 renderPixel(const cg_datastructures::Camera& camera,
                                       int x, int y, int width, int height);
    void resetRenderingStatistics();
    void logProgressUpdate(int currentRow, int totalRows) const;
    void logRenderingStatistics(int hitCount, int totalRays) const;

    // Helper functions
    cg_datastructures::Ray generateRay(const cg_datastructures::Camera& camera, float u, float v);
    bool rayTriangleIntersection(const cg_datastructures::Ray& ray,
                                const cg_datastructures::Triangle& tri,
                                float& t, float& u, float& v);
    int worldToVoxelIndex(float worldCoord, float gridMin, float voxelSize, int maxIndex) const;
    cg_datastructures::Vec3 voxelIndexToWorld(int ix, int iy, int iz) const;

    // Statistics
    mutable unsigned int voxelsTraversed = 0;
    mutable unsigned int triangleTests = 0;
};
