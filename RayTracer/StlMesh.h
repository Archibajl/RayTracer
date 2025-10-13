// Struct for use on the device (CUDA)
#include <builtin_types.h>
struct StlMesh
{
    const float* coords;    // [num_vrts * 3]
    const float* normals;   // [num_tris * 3]
    const unsigned int* tris; // [num_tris * 3]
    size_t num_vrts;
    size_t num_tris;

    // Add this method to StlMesh to provide tri_corner_coords functionality
    __host__ __device__
    const float* tri_corner_coords(size_t tri_idx, size_t corner_idx) const {
        // Each triangle has 3 corners, each corner is an index into the vertex array
        // tris is [num_tris * 3], so for triangle tri_idx and corner corner_idx:
        size_t vertex_idx = tris[tri_idx * 3 + corner_idx];
        return &coords[vertex_idx * 3];
    }

    // Add this method to StlMesh to provide tri_normal functionality
    __host__ __device__
    const float* tri_normal(size_t tri_idx) const {
        // Each triangle has a normal stored in normals array [num_tris * 3]
        return &normals[tri_idx * 3];
    }
};