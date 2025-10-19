#pragma once

#include "SceneCreator.h"
#include "Voxels.h"
#include <vector>
#include <cmath>
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
    Vec3 normal;                // Surface normal at intersection
    unsigned int triangleIdx;   // Index of hit triangle
    unsigned int voxelIdx;      // Index of hit voxel

    RayHit() : hit(false), t(std::numeric_limits<float>::max()),
               triangleIdx(0), voxelIdx(0) {}
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

// Main ray tracer class
class RayTracerEngine {
public:
    RayTracerEngine(const VoxelGrid& grid);
    ~RayTracerEngine();

    // Render a frame
    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height);

    // Trace a single ray
    cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray);

private:
    const VoxelGrid& voxelGrid;

    // Helper functions
    cg_datastructures::Ray generateRay(
        const cg_datastructures::Camera& camera,
        float u, float v);

    bool rayTriangleIntersection(
        const cg_datastructures::Ray& ray,
        const cg_datastructures::Triangle& tri,
        float& t, float& u, float& v);

    bool rayBoxIntersection(
        const cg_datastructures::Ray& ray,
        const cg_datastructures::Vec3& boxMin,
        const cg_datastructures::Vec3& boxMax,
        float& tMin, float& tMax);

    size_t getVoxelIndex(size_t ix, size_t iy, size_t iz) const;

    // DDA-like voxel traversal
    cg_datastructures::RayHit traverseVoxelGridDDA(const cg_datastructures::Ray& ray);
};
