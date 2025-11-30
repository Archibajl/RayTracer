#pragma once

#include "RayTracerCommon.h"
#include "Voxels.h"

namespace GeometryUtils {
    using namespace cg_datastructures;

    // Ray-Box Intersection using slab method
    bool rayBoxIntersection(const Ray& ray, const Vec3& boxMin, const Vec3& boxMax, float& tNear, float& tFar);

    // Ray-Triangle Intersection using Möller-Trumbore algorithm
    bool rayTriangleIntersection(const Triangle& triangle, const Vec3& ray_vector, const Vec3& ray_origin, float& t);

    // Compute AABB for a triangle
    void computeTriangleMinMax(const Triangle& triangle, Vec3& min, Vec3& max);

    // Check if a triangle intersects with an AABB
    bool triangleIntersectsBox(const Triangle& triangle, const Vec3& boxCenter, const Vec3& boxHalfSize);
}
