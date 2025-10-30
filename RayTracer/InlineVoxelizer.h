#pragma once

#include <limits>
#include <algorithm>
#include "VoxelGrid.h"
#include "STLReader.h"

// ============================================================================
// INLINE TEMPLATE IMPLEMENTATION
// ============================================================================
// This is a header-only implementation for quick voxelization without
// needing to link against SceneCreator.cpp. For better organization and
// logging, use BuildVoxelGridFromStlMesh from MeshVoxelizer.h instead.

// Build voxel grid directly from stl_reader::StlMesh (header-only version)
template <typename TNumber, typename TIndex>
VoxelGrid BuildVoxelGridFromStlReaderMesh(const stl_reader::StlMesh<TNumber, TIndex>& stlMesh, int nx, int ny, int nz) {
    // Step 1: Compute mesh AABB (bounding box)
    cg_datastructures::Vec3 minBound = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    };
    cg_datastructures::Vec3 maxBound = {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()
    };

    for (size_t i = 0; i < stlMesh.num_vrts(); ++i) {
        const TNumber* coords = stlMesh.vrt_coords(i);
        minBound.x = std::min(minBound.x, static_cast<float>(coords[0]));
        minBound.y = std::min(minBound.y, static_cast<float>(coords[1]));
        minBound.z = std::min(minBound.z, static_cast<float>(coords[2]));
        maxBound.x = std::max(maxBound.x, static_cast<float>(coords[0]));
        maxBound.y = std::max(maxBound.y, static_cast<float>(coords[1]));
        maxBound.z = std::max(maxBound.z, static_cast<float>(coords[2]));
    }

    // Step 2: Compute voxel grid parameters with equal-sized voxels
    cg_datastructures::Vec3 gridSize = {
        maxBound.x - minBound.x,
        maxBound.y - minBound.y,
        maxBound.z - minBound.z
    };

    float maxDimension = std::max(std::max(gridSize.x, gridSize.y), gridSize.z);
    float maxResolution = static_cast<float>(std::max(std::max(nx, ny), nz));
    float uniformVoxelSize = maxDimension / maxResolution;

    cg_datastructures::Vec3 voxelSize = {
        uniformVoxelSize,
        uniformVoxelSize,
        uniformVoxelSize
    };

    // Step 3: Build triangle list
    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(stlMesh.num_tris());

    for (size_t t = 0; t < stlMesh.num_tris(); ++t) {
        const TIndex* indices = stlMesh.tri_corner_inds(t);

        const TNumber* v0_coords = stlMesh.vrt_coords(indices[0]);
        const TNumber* v1_coords = stlMesh.vrt_coords(indices[1]);
        const TNumber* v2_coords = stlMesh.vrt_coords(indices[2]);

        cg_datastructures::Vec3 v0 = {
            static_cast<float>(v0_coords[0]),
            static_cast<float>(v0_coords[1]),
            static_cast<float>(v0_coords[2])
        };
        cg_datastructures::Vec3 v1 = {
            static_cast<float>(v1_coords[0]),
            static_cast<float>(v1_coords[1]),
            static_cast<float>(v1_coords[2])
        };
        cg_datastructures::Vec3 v2 = {
            static_cast<float>(v2_coords[0]),
            static_cast<float>(v2_coords[1]),
            static_cast<float>(v2_coords[2])
        };

        const TNumber* nrm = stlMesh.tri_normal(t);
        cg_datastructures::Vec3 normal = {
            static_cast<float>(nrm[0]),
            static_cast<float>(nrm[1]),
            static_cast<float>(nrm[2])
        };

        triangles.emplace_back(cg_datastructures::Triangle{ v0, v1, v2, normal });
    }

    // Step 4: Voxelize triangles
    std::vector<cg_datastructures::VoxelHost> voxels_host(nx * ny * nz);

    auto voxelIndex = [nx, ny](size_t ix, size_t iy, size_t iz) -> size_t {
        return ix + nx * (iy + ny * iz);
    };

    for (size_t t = 0; t < triangles.size(); ++t) {
        const cg_datastructures::Triangle& tri = triangles[t];

        // Triangle AABB
        cg_datastructures::Vec3 triMin = {
            std::min(std::min(tri.v0.x, tri.v1.x), tri.v2.x),
            std::min(std::min(tri.v0.y, tri.v1.y), tri.v2.y),
            std::min(std::min(tri.v0.z, tri.v1.z), tri.v2.z)
        };
        cg_datastructures::Vec3 triMax = {
            std::max(std::max(tri.v0.x, tri.v1.x), tri.v2.x),
            std::max(std::max(tri.v0.y, tri.v1.y), tri.v2.y),
            std::max(std::max(tri.v0.z, tri.v1.z), tri.v2.z)
        };

        // Determine which voxels this triangle overlaps
        size_t ix0 = std::clamp(static_cast<size_t>((triMin.x - minBound.x) / voxelSize.x), size_t(0), size_t(nx - 1));
        size_t iy0 = std::clamp(static_cast<size_t>((triMin.y - minBound.y) / voxelSize.y), size_t(0), size_t(ny - 1));
        size_t iz0 = std::clamp(static_cast<size_t>((triMin.z - minBound.z) / voxelSize.z), size_t(0), size_t(nz - 1));
        size_t ix1 = std::clamp(static_cast<size_t>((triMax.x - minBound.x) / voxelSize.x), size_t(0), size_t(nx - 1));
        size_t iy1 = std::clamp(static_cast<size_t>((triMax.y - minBound.y) / voxelSize.y), size_t(0), size_t(ny - 1));
        size_t iz1 = std::clamp(static_cast<size_t>((triMax.z - minBound.z) / voxelSize.z), size_t(0), size_t(nz - 1));

        // Add triangle to overlapping voxels
        for (size_t iz = iz0; iz <= iz1; ++iz) {
            for (size_t iy = iy0; iy <= iy1; ++iy) {
                for (size_t ix = ix0; ix <= ix1; ++ix) {
                    size_t voxIdx = voxelIndex(ix, iy, iz);
                    voxels_host[voxIdx].addTriangleIndex(static_cast<unsigned int>(t));
                    voxels_host[voxIdx].occupied = true;
                }
            }
        }
    }

    // Step 5: Convert to flat CUDA-compatible format
    std::vector<unsigned int> triangle_indices_flat;
    std::vector<cg_datastructures::Voxel> voxels_final(nx * ny * nz);

    for (size_t i = 0; i < voxels_host.size(); ++i) {
        cg_datastructures::VoxelHost& vh = voxels_host[i];
        cg_datastructures::Voxel& vf = voxels_final[i];

        vf.occupied = vh.occupied;
        vf.density = vh.density;
        vf.color = vh.color;

        if (vh.triangle_count > 0) {
            vf.triangle_start_idx = static_cast<unsigned int>(triangle_indices_flat.size());
            vf.triangle_count = static_cast<unsigned int>(vh.triangle_count);

            for (size_t j = 0; j < vh.triangle_count; ++j) {
                triangle_indices_flat.push_back(vh.triangle_indices[j]);
            }
        } else {
            vf.triangle_start_idx = 0;
            vf.triangle_count = 0;
        }
    }

    return VoxelGrid(
        std::move(voxels_final),
        nx, ny, nz,
        minBound, maxBound, voxelSize,
        std::move(triangles)
    );
}
