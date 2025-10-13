#include "SceneCreator.h"
#include <iostream>
#include <vector>
#include "Voxels.h"

struct VoxelGrid {
    std::vector<cg_datastructures::Voxel> voxels;
    size_t nx, ny, nz;
    cg_datastructures::Vec3 minBound, maxBound;
    cg_datastructures::Vec3 voxelSize;
    std::vector<cg_datastructures::Triangle> triangles; // Store actual triangles here

    VoxelGrid(
        std::vector<cg_datastructures::Voxel>&& voxels_,
        size_t nx_, size_t ny_, size_t nz_,
        const cg_datastructures::Vec3& minBound_,
        const cg_datastructures::Vec3& maxBound_,
        const cg_datastructures::Vec3& voxelSize_,
        std::vector<cg_datastructures::Triangle>&& triangles_)
        : voxels(std::move(voxels_)), nx(nx_), ny(ny_), nz(nz_),
          minBound(minBound_), maxBound(maxBound_), voxelSize(voxelSize_),
          triangles(std::move(triangles_))
    {}
};

inline VoxelGrid BuildVoxelGridFromStlMesh(StlMesh& mesh, size_t nx, size_t ny, size_t nz) {
    // Compute mesh AABB
    cg_datastructures::Vec3 minBound = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    cg_datastructures::Vec3 maxBound = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
    for (size_t i = 0; i < mesh.num_vrts; ++i) {
        float x = mesh.coords[i * 3 + 0];
        float y = mesh.coords[i * 3 + 1];
        float z = mesh.coords[i * 3 + 2];
        minBound.x = std::min(minBound.x, x);
        minBound.y = std::min(minBound.y, y);
        minBound.z = std::min(minBound.z, z);
        maxBound.x = std::max(maxBound.x, x);
        maxBound.y = std::max(maxBound.y, y);
        maxBound.z = std::max(maxBound.z, z);
    }

    // Compute voxel size
    cg_datastructures::Vec3 gridSize = { maxBound.x - minBound.x, maxBound.y - minBound.y, maxBound.z - minBound.z };
    cg_datastructures::Vec3 voxelSize = { gridSize.x / nx, gridSize.y / ny, gridSize.z / nz };

    // Allocate voxels
    std::vector<cg_datastructures::Voxel> voxels(nx * ny * nz);

    // Build triangle list
    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(mesh.num_tris);
    for (size_t t = 0; t < mesh.num_tris; ++t) {
        unsigned int i0 = mesh.tris[t * 3 + 0];
        unsigned int i1 = mesh.tris[t * 3 + 1];
        unsigned int i2 = mesh.tris[t * 3 + 2];
        cg_datastructures::Vec3 v0 = { mesh.coords[i0 * 3 + 0], mesh.coords[i0 * 3 + 1], mesh.coords[i0 * 3 + 2] };
        cg_datastructures::Vec3 v1 = { mesh.coords[i1 * 3 + 0], mesh.coords[i1 * 3 + 1], mesh.coords[i1 * 3 + 2] };
        cg_datastructures::Vec3 v2 = { mesh.coords[i2 * 3 + 0], mesh.coords[i2 * 3 + 1], mesh.coords[i2 * 3 + 2] };
        const float* nrm = mesh.tri_normal(t);
        cg_datastructures::Vec3 normal = { nrm[0], nrm[1], nrm[2] };
        triangles.emplace_back(cg_datastructures::Triangle{ v0, v1, v2, normal });
    }

    // Helper lambda to get voxel index
    auto voxelIndex = [=](size_t ix, size_t iy, size_t iz) {
        return ix + nx * (iy + ny * iz);
    };

    // For each triangle, find overlapping voxels
    for (size_t t = 0; t < mesh.num_tris; ++t) {
        const cg_datastructures::Triangle* triPtr = &triangles[t];
        const cg_datastructures::Vec3& v0 = triPtr->v0;
        const cg_datastructures::Vec3& v1 = triPtr->v1;
        const cg_datastructures::Vec3& v2 = triPtr->v2;

        // Triangle AABB
        cg_datastructures::Vec3 triMin = { std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}), std::min({v0.z, v1.z, v2.z}) };
        cg_datastructures::Vec3 triMax = { std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}), std::max({v0.z, v1.z, v2.z}) };

        // Voxel range
        size_t ix0 = std::clamp(static_cast<size_t>((triMin.x - minBound.x) / voxelSize.x), size_t(0), nx - 1);
        size_t iy0 = std::clamp(static_cast<size_t>((triMin.y - minBound.y) / voxelSize.y), size_t(0), ny - 1);
        size_t iz0 = std::clamp(static_cast<size_t>((triMin.z - minBound.z) / voxelSize.z), size_t(0), nz - 1);
        size_t ix1 = std::clamp(static_cast<size_t>((triMax.x - minBound.x) / voxelSize.x), size_t(0), nx - 1);
        size_t iy1 = std::clamp(static_cast<size_t>((triMax.y - minBound.y) / voxelSize.y), size_t(0), ny - 1);
        size_t iz1 = std::clamp(static_cast<size_t>((triMax.z - minBound.z) / voxelSize.z), size_t(0), nz - 1);

        // Assign triangle pointer to all voxels in its AABB
        for (size_t ix = ix0; ix <= ix1; ++ix) {
            for (size_t iy = iy0; iy <= iy1; ++iy) {
                for (size_t iz = iz0; iz <= iz1; ++iz) {
                    size_t idx = voxelIndex(ix, iy, iz);
                    voxels[idx].occupied = true;
                    voxels[idx].addTriangle(triPtr);
                }
            }
        }
    }

    return VoxelGrid{ std::move(voxels), nx, ny, nz, minBound, maxBound, voxelSize, std::move(triangles) };
}