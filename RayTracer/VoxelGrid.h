#pragma once

#include <vector>
#include <string>
#include "Voxels.h"

// VoxelGrid structure for CUDA-compatible voxel grid
struct VoxelGrid {
    // 3D array of voxels: voxels[x][y][z]
    std::vector<std::vector<std::vector<cg_datastructures::Voxel>>> voxels;
    std::vector<unsigned int> triangle_indices;          // Flat array of triangle indices
    int nx, ny, nz;
    cg_datastructures::Vec3 minBound, maxBound;
    cg_datastructures::Vec3 voxelSize;
    cg_datastructures::Vec3 center;                      // Center point of the grid
    std::vector<cg_datastructures::Triangle> triangles;  // Actual triangle data

    VoxelGrid(
        std::vector<std::vector<std::vector<cg_datastructures::Voxel>>>&& voxels_,
        std::vector<unsigned int>&& triangle_indices_,
        int nx_, int ny_, int nz_,
        const cg_datastructures::Vec3& minBound_,
        const cg_datastructures::Vec3& maxBound_,
        const cg_datastructures::Vec3& voxelSize_,
        std::vector<cg_datastructures::Triangle>&& triangles_)
        : voxels(std::move(voxels_)),
          triangle_indices(std::move(triangle_indices_)),
          nx(nx_), ny(ny_), nz(nz_),
          minBound(minBound_), maxBound(maxBound_), voxelSize(voxelSize_),
          center{(minBound_.x + maxBound_.x) * 0.5f,
                 (minBound_.y + maxBound_.y) * 0.5f,
                 (minBound_.z + maxBound_.z) * 0.5f},
          triangles(std::move(triangles_))
    {
    }

    // Serialize voxel grid to binary file
    bool saveToFile(const std::string& filename) const;

    // Deserialize voxel grid from binary file
    static VoxelGrid loadFromFile(const std::string& filename);
};
