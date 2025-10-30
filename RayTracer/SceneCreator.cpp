#include "MeshVoxelizer.h"
#include "VoxelizationHelpers.h"
#include "Logger.h"
#include <limits>

// ============================================================================
// HELPER FUNCTIONS
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
// VOXELIZATION SUBFUNCTIONS
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

// Voxelize triangles into a temporary host voxel grid
std::vector<cg_datastructures::VoxelHost> voxelizeTriangles(
    const std::vector<cg_datastructures::Triangle>& triangles,
    const VoxelizationParams& params)
{
    LOG_INFO("Allocating voxel grid: {}x{}x{} = {} voxels",
        params.nx, params.ny, params.nz, params.nx * params.ny * params.nz);

    std::vector<cg_datastructures::VoxelHost> voxels_host(params.nx * params.ny * params.nz);

    LOG_INFO("Voxelizing triangles...");
    int occupied_count = 0;

    for (int t = 0; t < triangles.size(); ++t) {
        const cg_datastructures::Vec3& v0 = triangles[t].v0;
        const cg_datastructures::Vec3& v1 = triangles[t].v1;
        const cg_datastructures::Vec3& v2 = triangles[t].v2;

        // Compute triangle AABB
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
        int ix0, iy0, iz0, ix1, iy1, iz1;
        worldToVoxelIndex(triMin, params.minBound, params.voxelSize,
            params.nx, params.ny, params.nz, ix0, iy0, iz0);
        worldToVoxelIndex(triMax, params.minBound, params.voxelSize,
            params.nx, params.ny, params.nz, ix1, iy1, iz1);

        // Assign triangle index to all voxels in its AABB
        for (int ix = ix0; ix <= ix1; ++ix) {
            for (int iy = iy0; iy <= iy1; ++iy) {
                for (int iz = iz0; iz <= iz1; ++iz) {
                    int idx = voxelIndex(ix, iy, iz, params.nx, params.ny);

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
        occupied_count, params.nx * params.ny * params.nz);

    return voxels_host;
}

// Convert host voxel format to CUDA-compatible flat format
//void convertToFlatFormat(
//    const std::vector<cg_datastructures::VoxelHost>&    ,
//    std::vector<cg_datastructures::Voxel>& voxels_out,
//    std::vector<unsigned int>& triangle_indices_out)
//{
//    LOG_INFO("Converting to CUDA-compatible format...");
//
//    voxels_out.resize(voxels_host.size());
//    triangle_indices_out.clear();
//
//    unsigned int current_start_idx = 0;
//    for (int i = 0; i < voxels_host.size(); ++i) {
//        voxels_out[i].occupied = voxels_host[i].occupied;
//        voxels_out[i].density = voxels_host[i].density;
//        voxels_out[i].color = voxels_host[i].color;
//        voxels_out[i].triangle_start_idx = current_start_idx;
//        voxels_out[i].triangle_count = static_cast<unsigned int>(voxels_host[i].triangle_count);
//
//        // Append triangle indices to the flat array
//        for (int j = 0; j < voxels_host[i].triangle_count; ++j) {
//            triangle_indices_out.push_back(voxels_host[i].triangle_indices[j]);
//        }
//
//        current_start_idx += static_cast<unsigned int>(voxels_host[i].triangle_count);
//    }
//
//    LOG_INFO("Total triangle references: {}", triangle_indices_out.size());
//}

// ============================================================================
// MAIN VOXELIZATION FUNCTION
// ============================================================================

template <typename TNumber, typename TIndex>
VoxelGrid BuildVoxelGridFromStlMesh(const stl_reader::StlMesh<TNumber, TIndex>& mesh, int nx, int ny, int nz) {
    // Step 1: Compute mesh bounds
    MeshBounds bounds = computeMeshBounds(mesh);

    // Step 2: Compute voxelization parameters
    VoxelizationParams params = computeVoxelizationParams(bounds, nx, ny, nz);

    // Step 3: Build triangle list
    std::vector<cg_datastructures::Triangle> triangles = buildTriangleList(mesh);

    // Step 4: Voxelize triangles
    std::vector<cg_datastructures::VoxelHost> voxels_host = voxelizeTriangles(triangles, params);

    // Step 5: Convert to CUDA-compatible format
    std::vector<cg_datastructures::Voxel> voxels(voxels_host.size());
    for (size_t i = 0; i < voxels_host.size(); ++i) {
        voxels[i].occupied = voxels_host[i].occupied;
        voxels[i].density = voxels_host[i].density;
        voxels[i].color = voxels_host[i].color;
        voxels[i].triangle_start_idx = voxels_host[i].triangle_indices ? voxels_host[i].triangle_indices[0] : 0;
        voxels[i].triangle_count = static_cast<unsigned int>(voxels_host[i].triangle_count);
    }

    // Step 6: Return the complete voxel grid
    return VoxelGrid{
        std::move(voxels),
        params.nx, params.ny, params.nz,
        params.minBound, params.maxBound, params.voxelSize,
        std::move(triangles)
    };
}

// Explicit template instantiations for common types
template MeshBounds computeMeshBounds<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh);
template MeshBounds computeMeshBounds<double, unsigned int>(
    const stl_reader::StlMesh<double, unsigned int>& mesh);

template std::vector<cg_datastructures::Triangle> buildTriangleList<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh);

template VoxelGrid BuildVoxelGridFromStlMesh<float, unsigned int>(
    const stl_reader::StlMesh<float, unsigned int>& mesh, int nx, int ny, int nz);

