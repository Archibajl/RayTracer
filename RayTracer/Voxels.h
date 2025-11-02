#ifndef CG_DATASTRUCTURES_VOXELS_H
#define CG_DATASTRUCTURES_VOXELS_H

namespace cg_datastructures {

struct Vec3 {
    float x, y, z;
};

struct Triangle {
    Vec3 v0, v1, v2; // Three vertices of the triangle
    Vec3 normal;     // Optional: may be zeroed if not used
};

// CUDA-compatible Voxel structure
// For CUDA compatibility, we use indices instead of pointers
struct Voxel {
    bool occupied = false;
    unsigned int triangle_start_idx = 0;  // Starting index in global triangle index array
    unsigned int triangle_count = 0;      // Number of triangles in this voxel
    float density = 0.0f;
    Vec3 color = { 0.0f, 0.0f, 0.0f };
};


} // namespace cg_datastructures

#endif // CG_DATASTRUCTURES_VOXELS_H
