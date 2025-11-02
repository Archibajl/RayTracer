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

        const float EPSILON = 0.0000001f;

        Vec3 edge1 = subtract(tri.v1, tri.v0);
        Vec3 edge2 = subtract(tri.v2, tri.v0);

        Vec3 h = cross(ray.direction, edge2);
        float a = dot(edge1, h);

        // Ray is parallel to triangle
        if (a > -EPSILON && a < EPSILON) {
            return false;
        }

        float f = 1.0f / a;
        Vec3 s = subtract(ray.origin, tri.v0);
        u = f * dot(s, h);

        if (u < 0.0f || u > 1.0f) {
            return false;
        }

        Vec3 q = cross(s, edge1);
        v = f * dot(ray.direction, q);

        if (v < 0.0f || u + v > 1.0f) {
            return false;
        }

        // Calculate t to find intersection point
        t = f * dot(edge2, q);

        return t > EPSILON;
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

// ============================================================================
// TRIANGLE-BOX INTERSECTION (SEPARATING AXIS THEOREM)
// ============================================================================

    bool triangleIntersectsBox(const Triangle& tri, const Vec3& boxCenter, const Vec3& boxHalfSize) {
        // Transform triangle to box-centered coordinate system
        Vec3 v0 = subtract(tri.v0, boxCenter);
        Vec3 v1 = subtract(tri.v1, boxCenter);
        Vec3 v2 = subtract(tri.v2, boxCenter);

        // Test 1: Triangle AABB vs box AABB
        Vec3 triMin = {
            std::min(std::min(v0.x, v1.x), v2.x),
            std::min(std::min(v0.y, v1.y), v2.y),
            std::min(std::min(v0.z, v1.z), v2.z)
        };

        Vec3 triMax = {
            std::max(std::max(v0.x, v1.x), v2.x),
            std::max(std::max(v0.y, v1.y), v2.y),
            std::max(std::max(v0.z, v1.z), v2.z)
        };

        if (triMin.x > boxHalfSize.x || triMax.x < -boxHalfSize.x) return false;
        if (triMin.y > boxHalfSize.y || triMax.y < -boxHalfSize.y) return false;
        if (triMin.z > boxHalfSize.z || triMax.z < -boxHalfSize.z) return false;

        // Test 2: Plane of triangle vs box
        Vec3 normal = normalize(cross(subtract(v1, v0), subtract(v2, v0)));
        float d = dot(normal, v0);

        // Find the box vertices that are min/max along the normal direction
        Vec3 vmin, vmax;
        vmin.x = (normal.x > 0) ? -boxHalfSize.x : boxHalfSize.x;
        vmin.y = (normal.y > 0) ? -boxHalfSize.y : boxHalfSize.y;
        vmin.z = (normal.z > 0) ? -boxHalfSize.z : boxHalfSize.z;
        vmax.x = -vmin.x;
        vmax.y = -vmin.y;
        vmax.z = -vmin.z;

        if (dot(normal, vmin) + d > 0) return false;
        if (dot(normal, vmax) + d < 0) return false;

        return true;
    }

}
