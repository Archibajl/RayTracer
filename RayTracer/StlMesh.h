#ifndef STL_MESH_H
#define STL_MESH_H

// Struct for use on the device (CUDA)
#include <builtin_types.h>
struct StlMeshCuda
{
    const float* coords;    // [num_vrts * 3]
    const float* normals;   // [num_tris * 3]
    const unsigned int* tris; // [num_tris * 3]
    size_t num_vrts;
    size_t num_tris;

    // Returns the coordinates of the specified corner of a triangle
    __host__ __device__
    const float* tri_corner_coords(size_t tri_idx, size_t corner_idx) const {
        size_t vertex_idx = tris[tri_idx * 3 + corner_idx];
        return &coords[vertex_idx * 3];
    }

    // Returns the normal of the specified triangle
    __host__ __device__
    const float* tri_normal(size_t tri_idx) const {
        return &normals[tri_idx * 3];
    }
};

#endif // STL_MESH_H