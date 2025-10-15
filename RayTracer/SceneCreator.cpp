#include "SceneCreator.h"
#include "Logger.h"
#include <limits>

// Helper function to compute the min of three floats
inline float min3(float a, float b, float c) {
    return std::min(std::min(a, b), c);
}

// Helper function to compute the max of three floats
inline float max3(float a, float b, float c) {
    return std::max(std::max(a, b), c);
}

// Helper function to convert world coordinates to voxel indices
inline void worldToVoxelIndex(
    const cg_datastructures::Vec3& pos,
    const cg_datastructures::Vec3& minBound,
    const cg_datastructures::Vec3& voxelSize,
    size_t nx, size_t ny, size_t nz,
    size_t& ix, size_t& iy, size_t& iz)
{
    float fx = (pos.x - minBound.x) / voxelSize.x;
    float fy = (pos.y - minBound.y) / voxelSize.y;
    float fz = (pos.z - minBound.z) / voxelSize.z;

    ix = std::clamp(static_cast<size_t>(fx), size_t(0), nx - 1);
    iy = std::clamp(static_cast<size_t>(fy), size_t(0), ny - 1);
    iz = std::clamp(static_cast<size_t>(fz), size_t(0), nz - 1);
}

inline VoxelGrid BuildVoxelGridFromStlMesh(StlMeshCuda& mesh, size_t nx, size_t ny, size_t nz) {
    // Step 1: Compute mesh AABB (bounding box)
    LOG_INFO("Computing mesh bounding box...");
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

    LOG_INFO("Bounding box: Min({}, {}, {}) Max({}, {}, {})",
        minBound.x, minBound.y, minBound.z,
        maxBound.x, maxBound.y, maxBound.z);

    // Step 2: Compute voxel grid parameters with equal-sized voxels
    cg_datastructures::Vec3 gridSize = {
        maxBound.x - minBound.x,
        maxBound.y - minBound.y,
        maxBound.z - minBound.z
    };

    // Find the maximum dimension to determine uniform voxel size
    float maxDimension = std::max(std::max(gridSize.x, gridSize.y), gridSize.z);
    float maxResolution = static_cast<float>(std::max(std::max(nx, ny), nz));
    float uniformVoxelSize = maxDimension / maxResolution;

    // Create uniform voxel size for all dimensions
    cg_datastructures::Vec3 voxelSize = {
        uniformVoxelSize,
        uniformVoxelSize,
        uniformVoxelSize
    };

    // Adjust grid dimensions to accommodate the mesh with uniform voxels
    // Center the mesh within the voxel grid
    cg_datastructures::Vec3 gridSizeAdjusted = {
        gridSize.x,
        gridSize.y,
        gridSize.z
    };

    LOG_INFO("Uniform voxel size: {}", uniformVoxelSize);
    LOG_INFO("Grid dimensions: ({}, {}, {})", gridSize.x, gridSize.y, gridSize.z);

    // Step 3: Build triangle list that will persist
    LOG_INFO("Building triangle list from {} triangles...", mesh.num_tris);
    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(mesh.num_tris);

    for (size_t t = 0; t < mesh.num_tris; ++t) {
        unsigned int i0 = mesh.tris[t * 3 + 0];
        unsigned int i1 = mesh.tris[t * 3 + 1];
        unsigned int i2 = mesh.tris[t * 3 + 2];

        cg_datastructures::Vec3 v0 = {
            mesh.coords[i0 * 3 + 0],
            mesh.coords[i0 * 3 + 1],
            mesh.coords[i0 * 3 + 2]
        };
        cg_datastructures::Vec3 v1 = {
            mesh.coords[i1 * 3 + 0],
            mesh.coords[i1 * 3 + 1],
            mesh.coords[i1 * 3 + 2]
        };
        cg_datastructures::Vec3 v2 = {
            mesh.coords[i2 * 3 + 0],
            mesh.coords[i2 * 3 + 1],
            mesh.coords[i2 * 3 + 2]
        };

        const float* nrm = mesh.tri_normal(t);
        cg_datastructures::Vec3 normal = { nrm[0], nrm[1], nrm[2] };

        triangles.emplace_back(cg_datastructures::Triangle{ v0, v1, v2, normal });
    }

    // Step 4: Allocate temporary host voxel grid for building
    LOG_INFO("Allocating voxel grid: {}x{}x{} = {} voxels", nx, ny, nz, nx * ny * nz);
    std::vector<cg_datastructures::VoxelHost> voxels_host(nx * ny * nz);

    // Helper lambda to get voxel index from (ix, iy, iz)
    auto voxelIndex = [nx, ny](size_t ix, size_t iy, size_t iz) -> size_t {
        return ix + nx * (iy + ny * iz);
    };

    // Step 5: Voxelize - assign triangles to voxels
    LOG_INFO("Voxelizing triangles...");
    size_t occupied_count = 0;

    for (size_t t = 0; t < triangles.size(); ++t) {
        const cg_datastructures::Vec3& v0 = triangles[t].v0;
        const cg_datastructures::Vec3& v1 = triangles[t].v1;
        const cg_datastructures::Vec3& v2 = triangles[t].v2;

        // Compute triangle AABB using optimized min3/max3 functions
        cg_datastructures::Vec3 triMin = {
            min3(v0.x, v1.x, v2.x),
            min3(v0.y, v1.y, v2.y),
            min3(v0.z, v1.z, v2.z)
        };
        cg_datastructures::Vec3 triMax = {
            max3(v0.x, v1.x, v2.x),
            max3(v0.y, v1.y, v2.y),
            max3(v0.z, v1.z, v2.z)
        };

        // Convert triangle AABB to voxel index range
        size_t ix0, iy0, iz0, ix1, iy1, iz1;
        worldToVoxelIndex(triMin, minBound, voxelSize, nx, ny, nz, ix0, iy0, iz0);
        worldToVoxelIndex(triMax, minBound, voxelSize, nx, ny, nz, ix1, iy1, iz1);

        // Assign triangle index to all voxels in its AABB
        for (size_t ix = ix0; ix <= ix1; ++ix) {
            for (size_t iy = iy0; iy <= iy1; ++iy) {
                for (size_t iz = iz0; iz <= iz1; ++iz) {
                    size_t idx = voxelIndex(ix, iy, iz);

                    // Mark voxel as occupied and add triangle index
                    if (!voxels_host[idx].occupied) {
                        voxels_host[idx].occupied = true;
                        occupied_count++;
                    }
                    voxels_host[idx].addTriangleIndex(static_cast<unsigned int>(t));
                }
            }
        }
    }

    LOG_INFO("Voxelization complete: {} voxels occupied out of {} total voxels",
        occupied_count, nx * ny * nz);

    // Step 6: Convert to CUDA-compatible format with flat triangle index array
    LOG_INFO("Converting to CUDA-compatible format...");
    std::vector<cg_datastructures::Voxel> voxels(nx * ny * nz);
    std::vector<unsigned int> triangle_indices;

    unsigned int current_start_idx = 0;
    for (size_t i = 0; i < voxels_host.size(); ++i) {
        voxels[i].occupied = voxels_host[i].occupied;
        voxels[i].density = voxels_host[i].density;
        voxels[i].color = voxels_host[i].color;
        voxels[i].triangle_start_idx = current_start_idx;
        voxels[i].triangle_count = static_cast<unsigned int>(voxels_host[i].triangle_count);

        // Append triangle indices to the flat array
        for (size_t j = 0; j < voxels_host[i].triangle_count; ++j) {
            triangle_indices.push_back(voxels_host[i].triangle_indices[j]);
        }

        current_start_idx += static_cast<unsigned int>(voxels_host[i].triangle_count);
    }

    LOG_INFO("Total triangle references: {}", triangle_indices.size());

    // Step 7: Return the complete voxel grid
    return VoxelGrid{
        std::move(voxels),
        std::move(triangle_indices),
        nx, ny, nz,
        minBound, maxBound, voxelSize,
        std::move(triangles)
    };
}