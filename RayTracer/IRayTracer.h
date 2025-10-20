#pragma once

#include "RayTracerCommon.h"
#include <vector>
#include <string>

/**
 * Abstract interface for ray tracers
 *
 * This allows different ray tracing acceleration structures and algorithms
 * to be used interchangeably. Implementations might include:
 * - VoxelDDARayTracer: Grid-based DDA traversal (fast for dense scenes)
 * - BVHRayTracer: Bounding Volume Hierarchy (good for complex meshes)
 * - OctreeRayTracer: Adaptive spatial subdivision
 * - BruteForceRayTracer: Direct triangle testing (simple, good for testing)
 */
class IRayTracer {
public:
    virtual ~IRayTracer() = default;

    /**
     * Render a full frame
     * @param camera The camera configuration
     * @param width Image width in pixels
     * @param height Image height in pixels
     * @return Vector of pixels (RGB values 0-1)
     */
    virtual std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) = 0;

    /**
     * Trace a single ray through the scene
     * @param ray The ray to trace
     * @return Hit information (or miss if hit=false)
     */
    virtual cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray) = 0;

    /**
     * Get a human-readable name for this ray tracing method
     * @return Method name (e.g., "Voxel Grid DDA", "BVH", etc.)
     */
    virtual std::string getMethodName() const = 0;
};
