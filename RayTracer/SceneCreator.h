#pragma once

#include <cstddef>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
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

    // Serialize voxel grid to binary file
    bool saveToFile(const std::string& filename) const;

    // Deserialize voxel grid from binary file
    static VoxelGrid loadFromFile(const std::string& filename);
};

// Helper structures for voxelization
struct MeshBounds {
    cg_datastructures::Vec3 minBound;
    cg_datastructures::Vec3 maxBound;
};

struct VoxelizationParams {
    cg_datastructures::Vec3 voxelSize;
    cg_datastructures::Vec3 minBound;
    cg_datastructures::Vec3 maxBound;
    size_t nx, ny, nz;
};

// Forward declarations for voxelization functions
VoxelGrid BuildVoxelGridFromStlMesh(StlMesh& mesh, size_t nx, size_t ny, size_t nz);

// Helper functions for voxelization
MeshBounds computeMeshBounds(const StlMesh& mesh);
VoxelizationParams computeVoxelizationParams(const MeshBounds& bounds, size_t nx, size_t ny, size_t nz);
std::vector<cg_datastructures::Triangle> buildTriangleList(const StlMesh& mesh);
std::vector<cg_datastructures::VoxelHost> voxelizeTriangles(
    const std::vector<cg_datastructures::Triangle>& triangles,
    const VoxelizationParams& params);
void convertToFlatFormat(
    const std::vector<cg_datastructures::VoxelHost>& voxels_host,
    std::vector<cg_datastructures::Voxel>& voxels_out,
    std::vector<unsigned int>& triangle_indices_out);
