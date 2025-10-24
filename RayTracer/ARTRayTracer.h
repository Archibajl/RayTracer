#pragma once

#include "IRayTracer.h"
#include "SceneCreator.h"
#include "VectorMath.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>

/**
 * ARTS (Accelerated Ray-Tracing System) Ray Tracer
 *
 * Implementation of Fujimoto's 3DDDA (3D Digital Differential Analyzer) algorithm
 * from the seminal 1986 paper "ARTS: Accelerated Ray-Tracing System" by
 * Akira Fujimoto, Takayuki Tanaka, and K. Iwata.
 *
 * Key features of ARTS:
 * - 3DDDA voxel traversal with spatial coherence
 * - Uniform spatial subdivision for O(1) voxel lookup
 * - Performance virtually independent of object count
 * - Optimized for scenes with many objects (1000+)
 *
 * This implementation follows the classic ARTS algorithm with modern optimizations.
 */
class ARTRayTracer : public IRayTracer {
public:
    ARTRayTracer(const VoxelGrid& grid);
    ~ARTRayTracer() override = default;

    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) override;

    cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray) override;

    std::string getMethodName() const override { return "ARTS (3DDDA)"; }

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

    // Rendering helper methods
    cg_datastructures::Vec3 computeShading(const cg_datastructures::RayHit& hit, float v) const;

    // Helper functions
    cg_datastructures::Ray generateRay(const cg_datastructures::Camera& camera, float u, float v);
    bool rayBoxIntersection(const cg_datastructures::Ray& ray,
                           const cg_datastructures::Vec3& boxMin,
                           const cg_datastructures::Vec3& boxMax,
                           float& tMin, float& tMax);
    bool rayTriangleIntersection(const cg_datastructures::Ray& ray,
                                const cg_datastructures::Triangle& tri,
                                float& t, float& u, float& v);
    size_t getVoxelIndex(int ix, int iy, int iz) const;

    // Statistics
    mutable size_t voxelsTraversed = 0;
    mutable size_t trianglesTests = 0;
};
