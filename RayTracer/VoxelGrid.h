#pragma once

#include <vector>
#include <string>
#include "Voxels.h"

// VoxelGrid structure for CUDA-compatible voxel grid
struct VoxelGrid {
    std::vector<cg_datastructures::Voxel> voxels;       // CUDA-compatible voxels
    std::vector<unsigned int> triangle_indices;          // Flat array of triangle indices
    int nx, ny, nz;
    cg_datastructures::Vec3 minBound, maxBound;
    cg_datastructures::Vec3 voxelSize;
    std::vector<cg_datastructures::Triangle> triangles;  // Actual triangle data

    VoxelGrid(
        std::vector<cg_datastructures::Voxel>&& voxels_,
        int nx_, int ny_, int nz_,
        const cg_datastructures::Vec3& minBound_,
        const cg_datastructures::Vec3& maxBound_,
        const cg_datastructures::Vec3& voxelSize_,
        std::vector<cg_datastructures::Triangle>&& triangles_)
        : voxels(std::move(voxels_)),
          nx(nx_), ny(ny_), nz(nz_),
          minBound(minBound_), maxBound(maxBound_), voxelSize(voxelSize_),
          triangles(std::move(triangles_))
    {
    }

    // Helper function to get triangles for a specific voxel
    /*const unsigned int* getVoxelTriangles(int voxel_idx) const {
        if (voxel_idx >= voxels.size()) return nullptr;
        const auto& voxel = voxels[voxel_idx];
        if (voxel.triangle_count == 0) return nullptr;
        return &triangle_indices[voxel.triangle_start_idx];
    }*/

    // Serialize voxel grid to binary file
    bool saveToFile(const std::string& filename) const;

    // Deserialize voxel grid from binary file
    static VoxelGrid loadFromFile(const std::string& filename);
};
