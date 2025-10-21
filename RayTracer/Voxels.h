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

// CUDA-compatible Voxel structure
// For CUDA compatibility, we use indices instead of pointers
struct Voxel {
    bool occupied = false;
    unsigned int triangle_start_idx = 0;  // Starting index in global triangle index array
    unsigned int triangle_count = 0;      // Number of triangles in this voxel
    float density = 0.0f;
    Vec3 color = { 0.0f, 0.0f, 0.0f };
};

// Host-side Voxel for building the grid (with dynamic allocation)
struct VoxelHost {
    bool occupied = false;
    unsigned int* triangle_indices = nullptr;  // Array of triangle indices
    size_t triangle_count = 0;
    size_t triangle_capacity = 0;
    float density = 0.0f;
    Vec3 color = { 0.0f, 0.0f, 0.0f };

    // Default constructor
    VoxelHost() = default;

    // Destructor
    ~VoxelHost() {
        if (triangle_indices != nullptr) {
            delete[] triangle_indices;
            triangle_indices = nullptr;
        }
    }

    // Copy constructor
    VoxelHost(const VoxelHost& other)
        : occupied(other.occupied),
          triangle_count(other.triangle_count),
          triangle_capacity(other.triangle_capacity),
          density(other.density),
          color(other.color)
    {
        if (other.triangle_indices != nullptr && other.triangle_capacity > 0) {
            triangle_indices = new unsigned int[other.triangle_capacity];
            // Copy only up to the minimum of capacity and count (defensive)
            size_t count_to_copy = (other.triangle_count < other.triangle_capacity) ? other.triangle_count : other.triangle_capacity;
            for (size_t i = 0; i < count_to_copy; ++i) {
                triangle_indices[i] = other.triangle_indices[i];
            }
        } else {
            triangle_indices = nullptr;
        }
    }

    // Move constructor
    VoxelHost(VoxelHost&& other) noexcept
        : occupied(other.occupied),
          triangle_indices(other.triangle_indices),
          triangle_count(other.triangle_count),
          triangle_capacity(other.triangle_capacity),
          density(other.density),
          color(other.color)
    {
        other.triangle_indices = nullptr;
        other.triangle_count = 0;
        other.triangle_capacity = 0;
    }

    // Copy assignment operator
    VoxelHost& operator=(const VoxelHost& other) {
        if (this != &other) {
            delete[] triangle_indices;

            occupied = other.occupied;
            triangle_count = other.triangle_count;
            triangle_capacity = other.triangle_capacity;
            density = other.density;
            color = other.color;

            if (other.triangle_indices != nullptr && other.triangle_capacity > 0) {
                triangle_indices = new unsigned int[other.triangle_capacity];
                // Copy only up to the minimum of capacity and count (defensive)
                size_t count_to_copy = (other.triangle_count < other.triangle_capacity) ? other.triangle_count : other.triangle_capacity;
                for (size_t i = 0; i < count_to_copy; ++i) {
                    triangle_indices[i] = other.triangle_indices[i];
                }
            } else {
                triangle_indices = nullptr;
            }
        }
        return *this;
    }

    // Move assignment operator
    VoxelHost& operator=(VoxelHost&& other) noexcept {
        if (this != &other) {
            delete[] triangle_indices;

            occupied = other.occupied;
            triangle_indices = other.triangle_indices;
            triangle_count = other.triangle_count;
            triangle_capacity = other.triangle_capacity;
            density = other.density;
            color = other.color;

            other.triangle_indices = nullptr;
            other.triangle_count = 0;
            other.triangle_capacity = 0;
        }
        return *this;
    }

    // Add triangle index to this voxel
    void addTriangleIndex(unsigned int tri_idx) {
        if (triangle_count >= triangle_capacity) {
            size_t new_capacity = triangle_capacity == 0 ? 4 : triangle_capacity * 2;
            unsigned int* new_indices = new unsigned int[new_capacity];
            // Copy existing indices (triangle_count will always be <= old capacity here)
            for (size_t i = 0; i < triangle_count; ++i) {
                new_indices[i] = triangle_indices[i];
            }
            delete[] triangle_indices;
            triangle_indices = new_indices;
            triangle_capacity = new_capacity;
        }
        // Now we're guaranteed triangle_count < triangle_capacity
        triangle_indices[triangle_count] = tri_idx;
        triangle_count++;
    }
};

} // namespace cg_datastructures

#endif // CG_DATASTRUCTURES_VOXELS_H
