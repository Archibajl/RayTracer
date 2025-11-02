#include "MeshVoxelizer.h"
#include "VoxelizationHelpers.h"
#include "GeometryUtils.h"
#include "Logger.h"
#include <limits>
#include <string>

// ============================================================================
// INLINE HELPER FUNCTIONS
// ============================================================================

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
    int nx, int ny, int nz,
    int& ix, int& iy, int& iz)
{
    float fx = (pos.x - minBound.x) / voxelSize.x;
    float fy = (pos.y - minBound.y) / voxelSize.y;
    float fz = (pos.z - minBound.z) / voxelSize.z;

    ix = std::clamp(static_cast<int>(fx), int(0), nx - 1);
    iy = std::clamp(static_cast<int>(fy), int(0), ny - 1);
    iz = std::clamp(static_cast<int>(fz), int(0), nz - 1);
}

// Helper function to compute linear voxel index from 3D coordinates
inline int voxelIndex(int ix, int iy, int iz, int nx, int ny) {
    return ix + nx * (iy + ny * iz);
}

// ============================================================================
// MESH BOUNDS COMPUTATION
// ============================================================================

// Compute the axis-aligned bounding box (AABB) of the mesh
template <typename TNumber, typename TIndex>
MeshBounds computeMeshBounds(const stl_reader::StlMesh<TNumber, TIndex>& mesh) {
    LOG_INFO("Computing mesh bounding box...");

    MeshBounds bounds;
    bounds.minBound = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    };
    bounds.maxBound = {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()
    };

    for (int i = 0; i < mesh.num_vrts(); ++i) {
        const TNumber* coords = mesh.vrt_coords(i);
        float x = static_cast<float>(coords[0]);
        float y = static_cast<float>(coords[1]);
        float z = static_cast<float>(coords[2]);

        bounds.minBound.x = std::min(bounds.minBound.x, x);
        bounds.minBound.y = std::min(bounds.minBound.y, y);
        bounds.minBound.z = std::min(bounds.minBound.z, z);
        bounds.maxBound.x = std::max(bounds.maxBound.x, x);
        bounds.maxBound.y = std::max(bounds.maxBound.y, y);
        bounds.maxBound.z = std::max(bounds.maxBound.z, z);
    }

    LOG_INFO("Bounding box: Min({}, {}, {}) Max({}, {}, {})",
        bounds.minBound.x, bounds.minBound.y, bounds.minBound.z,
        bounds.maxBound.x, bounds.maxBound.y, bounds.maxBound.z);

    return bounds;
}

// ============================================================================
// VOXELIZATION PARAMETERS COMPUTATION
// ============================================================================

// Calculate voxelization parameters (uniform voxel size)
VoxelizationParams computeVoxelizationParams(const MeshBounds& bounds, int nx, int ny, int nz) {
    VoxelizationParams params;
    params.nx = nx;
    params.ny = ny;
    params.nz = nz;
    params.minBound = bounds.minBound;
    params.maxBound = bounds.maxBound;

    // Compute grid size
    cg_datastructures::Vec3 gridSize = {
        bounds.maxBound.x - bounds.minBound.x,
        bounds.maxBound.y - bounds.minBound.y,
        bounds.maxBound.z - bounds.minBound.z
    };

    // Find the maximum dimension to determine uniform voxel size
    float maxDimension = std::max(std::max(gridSize.x, gridSize.y), gridSize.z);
    float maxResolution = static_cast<float>(std::max(std::max(nx, ny), nz));
    float uniformVoxelSize = maxDimension / maxResolution;

    // Create uniform voxel size for all dimensions
    params.voxelSize = {
        uniformVoxelSize,
        uniformVoxelSize,
        uniformVoxelSize
    };

    LOG_INFO("Uniform voxel size: {}", uniformVoxelSize);
    LOG_INFO("Grid dimensions: ({}, {}, {})", gridSize.x, gridSize.y, gridSize.z);

    return params;
}

// ============================================================================
// TRIANGLE LIST BUILDING
// ============================================================================

// Build a list of triangles from the mesh
template <typename TNumber, typename TIndex>
std::vector<cg_datastructures::Triangle> buildTriangleList(const stl_reader::StlMesh<TNumber, TIndex>& mesh) {
    LOG_INFO("Building triangle list from {} triangles...", mesh.num_tris());

    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(mesh.num_tris());

    for (int t = 0; t < mesh.num_tris(); ++t) {
        const TIndex* tri_indices = mesh.tri_corner_inds(t);

        const TNumber* v0_coords = mesh.vrt_coords(tri_indices[0]);
        const TNumber* v1_coords = mesh.vrt_coords(tri_indices[1]);
        const TNumber* v2_coords = mesh.vrt_coords(tri_indices[2]);

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

        const TNumber* nrm = mesh.tri_normal(t);
        cg_datastructures::Vec3 normal = {
            static_cast<float>(nrm[0]),
            static_cast<float>(nrm[1]),
            static_cast<float>(nrm[2])
        };

        triangles.emplace_back(cg_datastructures::Triangle{ v0, v1, v2, normal });
    }

    return triangles;
}

// ============================================================================
// TRIANGLE VOXELIZATION - HELPER FUNCTIONS
// ============================================================================

// Convert triangle AABB to voxel index range
void triangleAABBToVoxelRange(
    const cg_datastructures::Vec3& triMin,
    const cg_datastructures::Vec3& triMax,
    const VoxelizationParams& params,
    int& ix0, int& iy0, int& iz0,
    int& ix1, int& iy1, int& iz1)
{
    worldToVoxelIndex(triMin, params.minBound, params.voxelSize,
        params.nx, params.ny, params.nz, ix0, iy0, iz0);
    worldToVoxelIndex(triMax, params.minBound, params.voxelSize,
        params.nx, params.ny, params.nz, ix1, iy1, iz1);
}

// ============================================================================
// TRIANGLE VOXELIZATION - MAIN FUNCTION (TWO-PASS APPROACH)
// ============================================================================

// Two-pass voxelization that directly produces flat CUDA-compatible format
// Pass 1: Build lists of triangle indices for each voxel
// Pass 2: Flatten into CUDA-compatible arrays (voxels + triangle_indices)
VoxelizationResult voxelizeTrianglesDirect(
    const std::vector<cg_datastructures::Triangle>& triangles,
    const VoxelizationParams& params)
{
    LOG_INFO("Allocating voxel grid: {}x{}x{} = {} voxels",
        params.nx, params.ny, params.nz, params.nx * params.ny * params.nz);

    const int total_voxels = params.nx * params.ny * params.nz;

    // Temporary storage: list of triangle indices for each voxel
    std::vector<std::vector<unsigned int>> voxel_triangle_lists(total_voxels);

    LOG_INFO("Pass 1: Assigning triangles to voxels...");

    // Pass 1: Build lists of triangle indices for each voxel
    for (int t = 0; t < triangles.size(); ++t) {
        // Compute triangle AABB using GeometryUtils
        cg_datastructures::Vec3 triMin, triMax;
        GeometryUtils::computeTriangleMinMax(triangles[t], triMin, triMax);

        // Convert triangle AABB to voxel index range
        int ix0, iy0, iz0, ix1, iy1, iz1;
        triangleAABBToVoxelRange(triMin, triMax, params, ix0, iy0, iz0, ix1, iy1, iz1);

        // Assign triangle index to all voxels in its AABB
        for (int ix = ix0; ix <= ix1; ++ix) {
            for (int iy = iy0; iy <= iy1; ++iy) {
                for (int iz = iz0; iz <= iz1; ++iz) {
                    int voxel_idx = voxelIndex(ix, iy, iz, params.nx, params.ny);
                    voxel_triangle_lists[voxel_idx].push_back(static_cast<unsigned int>(t));
                }
            }
        }
    }

    LOG_INFO("Pass 2: Building 3D voxel array...");

    // Pass 2: Build 3D voxel array and flat triangle index array
    VoxelizationResult result;

    // Initialize 3D array: voxels[x][y][z]
    result.voxels.resize(params.nx);
    for (int x = 0; x < params.nx; ++x) {
        result.voxels[x].resize(params.ny);
        for (int y = 0; y < params.ny; ++y) {
            result.voxels[x][y].resize(params.nz);
        }
    }

    result.occupied_count = 0;
    unsigned int current_start_idx = 0;

    // Convert 1D temporary data to 3D array
    for (int iz = 0; iz < params.nz; ++iz) {
        for (int iy = 0; iy < params.ny; ++iy) {
            for (int ix = 0; ix < params.nx; ++ix) {
                int flat_idx = ix + params.nx * (iy + params.ny * iz);
                const auto& tri_list = voxel_triangle_lists[flat_idx];

                auto& voxel = result.voxels[ix][iy][iz];
                voxel.occupied = !tri_list.empty();
                voxel.triangle_start_idx = current_start_idx;
                voxel.triangle_count = static_cast<unsigned int>(tri_list.size());
                voxel.density = 0.0f;
                voxel.color = { 0.0f, 0.0f, 0.0f };

                if (voxel.occupied) {
                    result.occupied_count++;
                }

                // Append this voxel's triangle indices to the flat array
                for (unsigned int tri_idx : tri_list) {
                    result.triangle_indices.push_back(tri_idx);
                }

                current_start_idx += static_cast<unsigned int>(tri_list.size());
            }
        }
    }

    LOG_INFO("Voxelization complete: {} occupied / {} total voxels, {} triangle references",
        result.occupied_count, total_voxels, result.triangle_indices.size());

    return result;
}


// ============================================================================
// VOXEL GRID CREATION
// ============================================================================

// Create VoxelGrid from voxels and parameters
VoxelGrid createVoxelGrid(
    std::vector<std::vector<std::vector<cg_datastructures::Voxel>>>&& voxels,
    std::vector<unsigned int>&& triangle_indices,
    const VoxelizationParams& params,
    std::vector<cg_datastructures::Triangle>&& triangles)
{
    return VoxelGrid{
        std::move(voxels),
        std::move(triangle_indices),
        params.nx, params.ny, params.nz,
        params.minBound, params.maxBound, params.voxelSize,
        std::move(triangles)
    };
}

// ============================================================================
// MAIN VOXEL GRID BUILDING FUNCTION
// ============================================================================

template <typename TNumber, typename TIndex>
VoxelGrid BuildVoxelGridFromStlMesh(const stl_reader::StlMesh<TNumber, TIndex>& mesh, int nx, int ny, int nz) {
    // Step 1: Compute mesh bounds
    MeshBounds bounds = computeMeshBounds(mesh);

    // Step 2: Compute voxelization parameters
    VoxelizationParams params = computeVoxelizationParams(bounds, nx, ny, nz);

    // Step 3: Build triangle list
    std::vector<cg_datastructures::Triangle> triangles = buildTriangleList(mesh);

    // Step 4: Voxelize triangles directly to flat format (no VoxelHost)
    VoxelizationResult voxel_result = voxelizeTrianglesDirect(triangles, params);

    // Step 5: Return the complete voxel grid with flat triangle index array
    return createVoxelGrid(
        std::move(voxel_result.voxels),
        std::move(voxel_result.triangle_indices),
        params,
        std::move(triangles)
    );
}

// ============================================================================
// TEMPLATE INSTANTIATIONS
// ============================================================================

// Explicit template instantiations for common types
template MeshBounds computeMeshBounds<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh);
template MeshBounds computeMeshBounds<double, unsigned int>(
    const stl_reader::StlMesh<double, unsigned int>& mesh);

template std::vector<cg_datastructures::Triangle> buildTriangleList<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh);

template VoxelGrid BuildVoxelGridFromStlMesh<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh, int nx, int ny, int nz);

