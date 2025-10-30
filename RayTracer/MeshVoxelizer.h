#pragma once

#include <vector>
#include "VoxelGrid.h"
#include "VoxelizationHelpers.h"
#include "STLReader.h"

// Forward declarations for STL reader
namespace stl_reader {
    template <class TNumber, class TIndex>
    class StlMesh;
}

// ============================================================================
// Main voxelization functions
// ============================================================================

// Build voxel grid from stl_reader::StlMesh
template <typename TNumber = float, typename TIndex = unsigned int>
VoxelGrid BuildVoxelGridFromStlMesh(const stl_reader::StlMesh<TNumber, TIndex>& mesh, int nx, int ny, int nz);

// ============================================================================
// Helper functions for voxelization
// ============================================================================

// Compute the bounding box of a mesh
template <typename TNumber, typename TIndex>
MeshBounds computeMeshBounds(const stl_reader::StlMesh<TNumber, TIndex>& mesh);

// Compute voxelization parameters from bounding box
VoxelizationParams computeVoxelizationParams(const MeshBounds& bounds, int nx, int ny, int nz);

// Build triangle list from mesh
template <typename TNumber, typename TIndex>
std::vector<cg_datastructures::Triangle> buildTriangleList(const stl_reader::StlMesh<TNumber, TIndex>& mesh);

// Voxelize triangles into a grid
std::vector<cg_datastructures::VoxelHost> voxelizeTriangles(
    const std::vector<cg_datastructures::Triangle>& triangles,
    const VoxelizationParams& params);

// Convert voxel host format to flat CUDA-compatible format
void convertToFlatFormat(
    const std::vector<cg_datastructures::VoxelHost>& voxels_host,
    std::vector<cg_datastructures::Voxel>& voxels_out,
    std::vector<unsigned int>& triangle_indices_out);
