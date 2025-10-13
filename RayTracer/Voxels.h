#ifndef CG_DATASTRUCTURES_VOXELS_H
#define CG_DATASTRUCTURES_VOXELS_H

#include <cuda_runtime.h>
#include <vector_types.h>

namespace cg_datastructures {

struct Vec3 {
    float x, y, z;
};

struct Triangle {
    Vec3 v0, v1, v2; // Three vertices of the triangle
    Vec3 normal;     // Optional: may be zeroed if not used
};

// CUDA-compatible vector: use raw pointer and size
struct Voxel {
    bool occupied = false;
    const Triangle** triangles = nullptr; // Array of pointers to Triangle
    size_t triangle_count = 0;
    size_t triangle_capacity = 0;
    float density = 0.0f;
    Vec3 color = { 0.0f, 0.0f, 0.0f };

    // Add this method to allow adding triangle pointers
    void addTriangle(const Triangle* tri) {
        if (triangle_count == triangle_capacity) {
            // Grow capacity (double or start at 4)
            size_t new_capacity = triangle_capacity == 0 ? 4 : triangle_capacity * 2;
            const Triangle** new_triangles = new const Triangle*[new_capacity];
            for (size_t i = 0; i < triangle_count; ++i) {
                new_triangles[i] = triangles ? triangles[i] : nullptr;
            }
            delete[] triangles;
            triangles = new_triangles;
            triangle_capacity = new_capacity;
        }
        if (triangle_count < triangle_capacity) { // Ensure no buffer overrun
            triangles[triangle_count++] = tri;
        }
    }
};

} // namespace cg_datastructures

#endif // CG_DATASTRUCTURES_VOXELS_H
