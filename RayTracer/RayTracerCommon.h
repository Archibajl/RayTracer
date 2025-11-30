#pragma once

#include "Voxels.h"
#include <limits>

namespace cg_datastructures {

// Ray structure
struct Ray {
    Vec3 origin;
    Vec3 direction;  // Should be normalized

    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
};

// Intersection result
struct RayHit {
    bool hit;
    float t;                    // Distance along ray
    Vec3 point;                 // Intersection point
	Vec3 origin;                // Ray origin
    Vec3 normal;                // Surface normal at intersection
    unsigned int triangleIndex;   // Index of hit triangle
    unsigned int voxelIndex;      // Index of hit voxel (if applicable)

    RayHit() : hit(false), t(std::numeric_limits<float>::max()),
               point{0.0f, 0.0f, 0.0f}, normal{0.0f, 0.0f, 0.0f},
               triangleIndex(0), voxelIndex(0) {}
};

// Camera structure
struct Camera {
    Vec3 position;
    Vec3 lookAt;
    Vec3 up;
    float fov;          // Field of view in degrees
    float aspectRatio;  // Width / Height

    Camera(const Vec3& pos, const Vec3& target, const Vec3& upVec,
           float fieldOfView, float aspect)
        : position(pos), lookAt(target), up(upVec),
          fov(fieldOfView), aspectRatio(aspect) {}
};

} // namespace cg_datastructures
