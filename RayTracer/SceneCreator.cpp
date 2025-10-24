#include "SceneCreator.h"
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

// Helper function to compute linear voxel index from 3D coordinates
inline size_t voxelIndex(size_t ix, size_t iy, size_t iz, size_t nx, size_t ny) {
    return ix + nx * (iy + ny * iz);
}

// ============================================================================
// VOXELIZATION SUBFUNCTIONS
// ============================================================================

// Compute the axis-aligned bounding box (AABB) of the mesh
MeshBounds computeMeshBounds(const StlMesh& mesh) {
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

    for (size_t i = 0; i < mesh.num_vrts; ++i) {
        float x = mesh.coords[i * 3 + 0];
        float y = mesh.coords[i * 3 + 1];
        float z = mesh.coords[i * 3 + 2];

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
VoxelizationParams computeVoxelizationParams(const MeshBounds& bounds, size_t nx, size_t ny, size_t nz) {
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
std::vector<cg_datastructures::Triangle> buildTriangleList(const StlMesh& mesh) {
    LOG_INFO("Building triangle list from {} triangles...", mesh.num_tris);

    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(mesh.num_tris);

    for (unsigned int t = 0; t < mesh.num_tris; ++t) {
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
    size_t occupied_count = 0;

    for (size_t t = 0; t < triangles.size(); ++t) {
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
        size_t ix0, iy0, iz0, ix1, iy1, iz1;
        worldToVoxelIndex(triMin, params.minBound, params.voxelSize,
            params.nx, params.ny, params.nz, ix0, iy0, iz0);
        worldToVoxelIndex(triMax, params.minBound, params.voxelSize,
            params.nx, params.ny, params.nz, ix1, iy1, iz1);

        // Assign triangle index to all voxels in its AABB
        for (size_t ix = ix0; ix <= ix1; ++ix) {
            for (size_t iy = iy0; iy <= iy1; ++iy) {
                for (size_t iz = iz0; iz <= iz1; ++iz) {
                    size_t idx = voxelIndex(ix, iy, iz, params.nx, params.ny);

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
void convertToFlatFormat(
    const std::vector<cg_datastructures::VoxelHost>& voxels_host,
    std::vector<cg_datastructures::Voxel>& voxels_out,
    std::vector<unsigned int>& triangle_indices_out)
{
    LOG_INFO("Converting to CUDA-compatible format...");

    voxels_out.resize(voxels_host.size());
    triangle_indices_out.clear();

    unsigned int current_start_idx = 0;
    for (size_t i = 0; i < voxels_host.size(); ++i) {
        voxels_out[i].occupied = voxels_host[i].occupied;
        voxels_out[i].density = voxels_host[i].density;
        voxels_out[i].color = voxels_host[i].color;
        voxels_out[i].triangle_start_idx = current_start_idx;
        voxels_out[i].triangle_count = static_cast<unsigned int>(voxels_host[i].triangle_count);

        // Append triangle indices to the flat array
        for (size_t j = 0; j < voxels_host[i].triangle_count; ++j) {
            triangle_indices_out.push_back(voxels_host[i].triangle_indices[j]);
        }

        current_start_idx += static_cast<unsigned int>(voxels_host[i].triangle_count);
    }

    LOG_INFO("Total triangle references: {}", triangle_indices_out.size());
}

// ============================================================================
// MAIN VOXELIZATION FUNCTION
// ============================================================================

VoxelGrid BuildVoxelGridFromStlMesh(StlMesh& mesh, size_t nx, size_t ny, size_t nz) {
    // Step 1: Compute mesh bounds
    MeshBounds bounds = computeMeshBounds(mesh);

    // Step 2: Compute voxelization parameters
    VoxelizationParams params = computeVoxelizationParams(bounds, nx, ny, nz);

    // Step 3: Build triangle list
    std::vector<cg_datastructures::Triangle> triangles = buildTriangleList(mesh);

    // Step 4: Voxelize triangles
    std::vector<cg_datastructures::VoxelHost> voxels_host = voxelizeTriangles(triangles, params);

    // Step 5: Convert to CUDA-compatible format
    std::vector<cg_datastructures::Voxel> voxels;
    std::vector<unsigned int> triangle_indices;
    convertToFlatFormat(voxels_host, voxels, triangle_indices);

    // Step 6: Return the complete voxel grid
    return VoxelGrid{
        std::move(voxels),
        std::move(triangle_indices),
        params.nx, params.ny, params.nz,
        params.minBound, params.maxBound, params.voxelSize,
        std::move(triangles)
    };
}

// ============================================================================
// VOXELGRID SERIALIZATION
// ============================================================================

bool VoxelGrid::saveToFile(const std::string& filename) const {
    LOG_INFO("Saving voxel grid to: {}", filename);

    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        LOG_ERROR("Failed to open file for writing: {}", filename);
        return false;
    }

    // Write dimensions
    file.write(reinterpret_cast<const char*>(&nx), sizeof(nx));
    file.write(reinterpret_cast<const char*>(&ny), sizeof(ny));
    file.write(reinterpret_cast<const char*>(&nz), sizeof(nz));

    // Write bounds
    file.write(reinterpret_cast<const char*>(&minBound), sizeof(minBound));
    file.write(reinterpret_cast<const char*>(&maxBound), sizeof(maxBound));
    file.write(reinterpret_cast<const char*>(&voxelSize), sizeof(voxelSize));

    // Write voxels
    size_t voxel_count = voxels.size();
    file.write(reinterpret_cast<const char*>(&voxel_count), sizeof(voxel_count));
    file.write(reinterpret_cast<const char*>(voxels.data()), voxel_count * sizeof(cg_datastructures::Voxel));

    // Write triangle indices
    size_t tri_index_count = triangle_indices.size();
    file.write(reinterpret_cast<const char*>(&tri_index_count), sizeof(tri_index_count));
    file.write(reinterpret_cast<const char*>(triangle_indices.data()), tri_index_count * sizeof(unsigned int));

    // Write triangles
    size_t triangle_count = triangles.size();
    file.write(reinterpret_cast<const char*>(&triangle_count), sizeof(triangle_count));
    file.write(reinterpret_cast<const char*>(triangles.data()), triangle_count * sizeof(cg_datastructures::Triangle));

    file.close();
    LOG_INFO("Voxel grid saved successfully");
    return true;
}

VoxelGrid VoxelGrid::loadFromFile(const std::string& filename) {
    LOG_INFO("Loading voxel grid from: {}", filename);

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        LOG_ERROR("Failed to open file for reading: {}", filename);
        throw std::runtime_error("Failed to open voxel grid file: " + filename);
    }

    // Read dimensions
    size_t nx, ny, nz;
    file.read(reinterpret_cast<char*>(&nx), sizeof(nx));
    file.read(reinterpret_cast<char*>(&ny), sizeof(ny));
    file.read(reinterpret_cast<char*>(&nz), sizeof(nz));

    // Read bounds
    cg_datastructures::Vec3 minBound, maxBound, voxelSize;
    file.read(reinterpret_cast<char*>(&minBound), sizeof(minBound));
    file.read(reinterpret_cast<char*>(&maxBound), sizeof(maxBound));
    file.read(reinterpret_cast<char*>(&voxelSize), sizeof(voxelSize));

    // Read voxels
    size_t voxel_count;
    file.read(reinterpret_cast<char*>(&voxel_count), sizeof(voxel_count));
    std::vector<cg_datastructures::Voxel> voxels(voxel_count);
    file.read(reinterpret_cast<char*>(voxels.data()), voxel_count * sizeof(cg_datastructures::Voxel));

    // Read triangle indices
    size_t tri_index_count;
    file.read(reinterpret_cast<char*>(&tri_index_count), sizeof(tri_index_count));
    std::vector<unsigned int> triangle_indices(tri_index_count);
    file.read(reinterpret_cast<char*>(triangle_indices.data()), tri_index_count * sizeof(unsigned int));

    // Read triangles
    size_t triangle_count;
    file.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
    std::vector<cg_datastructures::Triangle> triangles(triangle_count);
    file.read(reinterpret_cast<char*>(triangles.data()), triangle_count * sizeof(cg_datastructures::Triangle));

    file.close();
    LOG_INFO("Voxel grid loaded successfully");

    return VoxelGrid{
        std::move(voxels),
        std::move(triangle_indices),
        nx, ny, nz,
        minBound, maxBound, voxelSize,
        std::move(triangles)
    };
}