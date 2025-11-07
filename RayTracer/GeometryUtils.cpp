#include "GeometryUtils.h"
#include "VectorMath.h"
#include <algorithm>
#include <limits>

using namespace cg_datastructures;
using namespace vector_math;

namespace GeometryUtils {

// ============================================================================
// RAY-BOX INTERSECTION (SLAB METHOD)
// ============================================================================

    bool rayBoxIntersection(
        const Ray& ray,
        const Vec3& boxMin,
        const Vec3& boxMax,
        float& tNear, float& tFar) {

        const float EPSILON = 1e-6f;

        // Handle near-zero direction components
        float invDirX = (std::abs(ray.direction.x) > EPSILON) ? (1.0f / ray.direction.x) : 1e30f;
        float invDirY = (std::abs(ray.direction.y) > EPSILON) ? (1.0f / ray.direction.y) : 1e30f;
        float invDirZ = (std::abs(ray.direction.z) > EPSILON) ? (1.0f / ray.direction.z) : 1e30f;

        float tx1 = (boxMin.x - ray.origin.x) * invDirX;
        float tx2 = (boxMax.x - ray.origin.x) * invDirX;
        float ty1 = (boxMin.y - ray.origin.y) * invDirY;
        float ty2 = (boxMax.y - ray.origin.y) * invDirY;
        float tz1 = (boxMin.z - ray.origin.z) * invDirZ;
        float tz2 = (boxMax.z - ray.origin.z) * invDirZ;

        tNear = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)), std::min(tz1, tz2));
        tFar = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)), std::max(tz1, tz2));

        return tFar >= tNear && tFar >= 0.0f;
    }

// ============================================================================
// RAY-TRIANGLE INTERSECTION (MÖLLER-TRUMBORE ALGORITHM)
// ============================================================================

    bool rayTriangleIntersection(
        const Ray& ray,
        const Triangle& tri,
        float& t, float& u, float& v) {
		return true; // Placeholder implementation
    }

// ============================================================================
// TRIANGLE Min Max Vector COMPUTATION
// ============================================================================

    void computeTriangleMinMax(const Triangle& triangle, Vec3& min, Vec3& max) {
        min.x = std::min(std::min(triangle.v0.x, triangle.v1.x), triangle.v2.x);
        min.y = std::min(std::min(triangle.v0.y, triangle.v1.y), triangle.v2.y);
        min.z = std::min(std::min(triangle.v0.z, triangle.v1.z), triangle.v2.z);

        max.x = std::max(std::max(triangle.v0.x, triangle.v1.x), triangle.v2.x);
        max.y = std::max(std::max(triangle.v0.y, triangle.v1.y), triangle.v2.y);
        max.z = std::max(std::max(triangle.v0.z, triangle.v1.z), triangle.v2.z);
    }

}
