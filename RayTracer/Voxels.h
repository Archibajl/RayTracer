#include <cuda_runtime.h>
#include <vector_types.h>

namespace cg_datastructures {

struct Vec3 {
    float x, y, z;
};

struct Triangle {
    Vec3 normal;     // Optional: may be zeroed if not used
    Vec3 v0, v1, v2; // Three vertices of the triangle
};

// CUDA-compatible vector: use raw pointer and size
struct Voxel {
    bool occupied = false;
    const Triangle** triangles = nullptr; // Array of pointers to Triangle
    size_t triangle_count = 0;
    size_t triangle_capacity = 0;
    float density = 0.0f;
    Vec3 color = { 0.0f, 0.0f, 0.0f };

    __host__ __device__
        void addTriangle(const Triangle* tri) {
        if (!hasTriangle(tri)) {
            if (triangle_count == triangle_capacity) {
                // Grow capacity (host only)
                #ifndef __CUDA_ARCH__
                size_t new_capacity = triangle_capacity == 0 ? 4 : triangle_capacity * 2;
                const Triangle** new_arr = new const Triangle * [new_capacity];
                for (size_t i = 0; i < triangle_count; ++i)
                    new_arr[i] = triangles[i];
                delete[] triangles;
                triangles = new_arr;
                triangle_capacity = new_capacity;
                #endif
            }
            if (triangle_count < triangle_capacity) {
                triangles[triangle_count++] = tri;
            }
        }
    }

    __host__ __device__
        bool hasTriangle(const Triangle* tri) const {
        for (size_t i = 0; i < triangle_count; ++i) {
            if (triangles[i] == tri)
                return true;
        }
        return false;
    }

    #ifndef __CUDA_ARCH__
    // Host-side destructor to free memory
    ~Voxel() {
        delete[] triangles;
    }
    #endif
    };

} // namespace Voxels
