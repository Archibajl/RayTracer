#pragma once

#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>
#include "StlMesh.h"
#include "Voxels.h"

// VoxelGrid structure for CUDA-compatible voxel grid
struct VoxelGrid {
    std::vector<cg_datastructures::Voxel> voxels;       // CUDA-compatible voxels
    std::vector<unsigned int> triangle_indices;          // Flat array of triangle indices
    size_t nx, ny, nz;
    cg_datastructures::Vec3 minBound, maxBound;
    cg_datastructures::Vec3 voxelSize;
    std::vector<cg_datastructures::Triangle> triangles;  // Actual triangle data

    VoxelGrid(
        std::vector<cg_datastructures::Voxel>&& voxels_,
        std::vector<unsigned int>&& triangle_indices_,
        size_t nx_, size_t ny_, size_t nz_,
        const cg_datastructures::Vec3& minBound_,
        const cg_datastructures::Vec3& maxBound_,
        const cg_datastructures::Vec3& voxelSize_,
        std::vector<cg_datastructures::Triangle>&& triangles_)
        : voxels(std::move(voxels_)),
          triangle_indices(std::move(triangle_indices_)),
          nx(nx_), ny(ny_), nz(nz_),
          minBound(minBound_), maxBound(maxBound_), voxelSize(voxelSize_),
          triangles(std::move(triangles_))
    {
    }

    // Helper function to get triangles for a specific voxel
    const unsigned int* getVoxelTriangles(size_t voxel_idx) const {
        if (voxel_idx >= voxels.size()) return nullptr;
        const auto& voxel = voxels[voxel_idx];
        if (voxel.triangle_count == 0) return nullptr;
        return &triangle_indices[voxel.triangle_start_idx];
    }
};

// Forward declarations for voxelization functions
VoxelGrid BuildVoxelGridFromStlMesh(StlMeshCuda& mesh, size_t nx, size_t ny, size_t nz);
VoxelGrid BuildVoxelGridFromStlMeshCuda(const StlMeshCuda& mesh, size_t nx, size_t ny, size_t nz);
