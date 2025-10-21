#pragma once

#include <cstddef>

// CUDA-compatible mesh structure
// This structure holds device pointers to mesh data and can be used in both host and device code
struct StlMeshCuda {
    float* coords;               // Device pointer to vertex coordinates (num_vrts * 3)
    float* normals;              // Device pointer to normals (num_tris * 3)
    unsigned int* tris;          // Device pointer to triangle indices (num_tris * 3)
    unsigned int num_vrts;       // Number of vertices
    unsigned int num_tris;       // Number of triangles

    // Helper functions for accessing triangle data
    // Note: These work only when data is on host or accessible from host
    const float* tri_corner_coords(unsigned int tri_idx, unsigned int corner_idx) const {
        unsigned int vert_idx = tris[tri_idx * 3 + corner_idx];
        return &coords[vert_idx * 3];
    }

    const float* tri_normal(unsigned int tri_idx) const {
        return &normals[tri_idx * 3];
    }
};
