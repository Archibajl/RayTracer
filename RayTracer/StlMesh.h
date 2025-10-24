#pragma once

#include <cstddef>

// Host-side mesh structure (non-CUDA)
// This structure holds pointers to mesh data on the CPU
struct StlMesh {
    float* coords;               // Pointer to vertex coordinates (num_vrts * 3)
    float* normals;              // Pointer to normals (num_tris * 3)
    unsigned int* tris;          // Pointer to triangle indices (num_tris * 3)
    unsigned int num_vrts;       // Number of vertices
    unsigned int num_tris;       // Number of triangles

    // Helper functions for accessing triangle data
    const float* tri_corner_coords(unsigned int tri_idx, unsigned int corner_idx) const {
        unsigned int vert_idx = tris[tri_idx * 3 + corner_idx];
        return &coords[vert_idx * 3];
    }

    const float* tri_normal(unsigned int tri_idx) const {
        return &normals[tri_idx * 3];
    }
};
