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

        // Calculate inverse direction components
        float invDirX = 1.0f / ray.direction.x;
        float invDirY = 1.0f / ray.direction.y;
        float invDirZ = 1.0f / ray.direction.z;

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

	// ============================================================================
	// TRIANGLE-BOX INTERSECTION
	// implemens the muller method to 
	// ============================================================================
    bool triangleIntersectsBox(
        const Vec3& ray_vector,
		const Vec3& ray_origin,
        const Triangle& triangle)
    {
		//Smallest number to check against zero
        constexpr float EPSILON = std::numeric_limits<float>::epsilon();

        Vec3 edge1 = subtract(triangle.v1, triangle.v0);
        Vec3 edge2 = subtract(triangle.v2, triangle.v0);
        Vec3 ray_cross_e2 = cross(ray_vector, edge2);
        float det = dot(edge1, ray_cross_e2);

        if (det > -EPSILON && det < EPSILON)
            return false; //{};    // This ray is parallel to this triangle.

        float inv_det = 1.0 / det;
        Vec3 s = subtract(ray_origin, triangle.v0);
        float u = inv_det * dot(s, ray_cross_e2);

        if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
            return false; //{};

        Vec3 s_cross_e1 = cross(s, edge1);
        float v = inv_det * dot(ray_vector, s_cross_e1);

        if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
            return false; //{};

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = inv_det * dot(edge2, s_cross_e1);

        if (t > EPSILON) // ray intersection
        {
            return  true; //Vec3(add(ray_origin, multiply(ray_vector, t)));
        }
        else // This means that there is a line intersection but not a ray intersection.
            return false; //{}; Vec3 ray_cross_e2 = cross(ray_vector, edge2);
        
    }
	// ============================================================================
  //  bool triangleIntersectsBox(
  //      const Vec3& ray_vector,
		//const Vec3& ray_origin,
  //      const Triangle& triangle)
  //  {
		////Smallest number to check against zero
  //      constexpr float EPSILON = std::numeric_limits<float>::epsilon();

  //      Vec3 edge1 = subtract(triangle.v1, triangle.v0);
  //      Vec3 edge2 = subtract(triangle.v2, triangle.v0);

  //              Vec3 ray_cross_e2 = cross(ray_vector, edge2);
  //              float det = dot(edge1, ray_cross_e2);

  //              if (det > -EPSILON && det < EPSILON)
  //                  return false; //{};    // This ray is parallel to this triangle.

  //              float inv_det = 1.0 / det;
  //              Vec3 s = subtract(ray_origin, triangle.v0);
  //              float u = inv_det * dot(s, ray_cross_e2);

  //              if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
  //                  return false; //{};

  //              Vec3 s_cross_e1 = cross(s, edge1);
  //              float v = inv_det * dot(ray_vector, s_cross_e1);

  //              if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
  //                  return false; //{};

  //              // At this stage we can compute t to find out where the intersection point is on the line.
  //              float t = inv_det * dot(edge2, s_cross_e1);

  //              if (t > EPSILON) // ray intersection
  //              {
  //                  return  true; //Vec3(add(ray_origin, multiply(ray_vector, t)));
  //              }
  //              else // This means that there is a line intersection but not a ray intersection.
  //                  return false; //{}; Vec3 ray_cross_e2 = cross(ray_vector, edge2);
  //      
  //  }

}
