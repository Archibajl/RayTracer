#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cuda_runtime_api.h>
#include <algorithm>
#include <vector>
#include <limits>
#include <iostream>
#include "SceneCreator.h"

// IntelliSense-only definitions (don't affect NVCC compilation)
#ifdef __INTELLISENSE__
    #ifndef __CUDACC__
    #define __CUDACC__
    #endif

    #ifndef __global__
    #define __global__
    #endif

    #ifndef __forceinline__
    #define __forceinline__ inline
    #endif
#endif

// Forward declare kernels to help linters
__global__ void CountTrianglesPerVoxelKernel(
    const float* coords, const unsigned int* tris, size_t num_tris,
    size_t nx, size_t ny, size_t nz,
    cg_datastructures::Vec3 minBound, cg_datastructures::Vec3 voxelSize,
    unsigned int* voxel_tri_counts);

__global__ void FillTriangleIndicesKernel(
    const float* coords, const unsigned int* tris, size_t num_tris,
    size_t nx, size_t ny, size_t nz,
    cg_datastructures::Vec3 minBound, cg_datastructures::Vec3 voxelSize,
    const unsigned int* voxel_start_indices,
    unsigned int* voxel_current_counts,
    unsigned int* triangle_indices);

// IntelliSense compatibility stubs
// Detect IntelliSense: it doesn't define __CUDA_ARCH__ and is not the actual CUDA compiler
#if !defined(__CUDA_ARCH__) && (defined(__INTELLISENSE__) || !defined(__CUDACC_VER__))

// Define __device__ for IntelliSense if not already defined
#ifndef __device__
#define __device__
#endif

#ifndef __host__
#define __host__
#endif

// Stub for atomicAdd to satisfy IntelliSense (will be replaced by actual CUDA version during compilation)
__device__ inline unsigned int atomicAdd(unsigned int* address, unsigned int val) {
    unsigned int old = *address;
    *address += val;
    return old;
}
#endif

// Device-compatible min/max/clamp
__device__ __forceinline__ float dmin(float a, float b) { return a < b ? a : b; }
__device__ __forceinline__ float dmax(float a, float b) { return a > b ? a : b; }
__device__ __forceinline__ size_t dclamp(size_t v, size_t lo, size_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Kernel to count how many triangles intersect each voxel
__global__ void CountTrianglesPerVoxelKernel(
    const float* coords, const unsigned int* tris, size_t num_tris,
    size_t nx, size_t ny, size_t nz,
    cg_datastructures::Vec3 minBound, cg_datastructures::Vec3 voxelSize,
    unsigned int* voxel_tri_counts)
{
    size_t t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= num_tris) return;

    unsigned int i0 = tris[t * 3 + 0];
    unsigned int i1 = tris[t * 3 + 1];
    unsigned int i2 = tris[t * 3 + 2];

    cg_datastructures::Vec3 v0 = { coords[i0 * 3 + 0], coords[i0 * 3 + 1], coords[i0 * 3 + 2] };
    cg_datastructures::Vec3 v1 = { coords[i1 * 3 + 0], coords[i1 * 3 + 1], coords[i1 * 3 + 2] };
    cg_datastructures::Vec3 v2 = { coords[i2 * 3 + 0], coords[i2 * 3 + 1], coords[i2 * 3 + 2] };

    // Triangle AABB
    cg_datastructures::Vec3 triMin = {
        dmin(dmin(v0.x, v1.x), v2.x),
        dmin(dmin(v0.y, v1.y), v2.y),
        dmin(dmin(v0.z, v1.z), v2.z)
    };
    cg_datastructures::Vec3 triMax = {
        dmax(dmax(v0.x, v1.x), v2.x),
        dmax(dmax(v0.y, v1.y), v2.y),
        dmax(dmax(v0.z, v1.z), v2.z)
    };

    // Voxel range
    size_t ix0 = dclamp((size_t)((triMin.x - minBound.x) / voxelSize.x), 0, nx - 1);
    size_t iy0 = dclamp((size_t)((triMin.y - minBound.y) / voxelSize.y), 0, ny - 1);
    size_t iz0 = dclamp((size_t)((triMin.z - minBound.z) / voxelSize.z), 0, nz - 1);
    size_t ix1 = dclamp((size_t)((triMax.x - minBound.x) / voxelSize.x), 0, nx - 1);
    size_t iy1 = dclamp((size_t)((triMax.y - minBound.y) / voxelSize.y), 0, ny - 1);
    size_t iz1 = dclamp((size_t)((triMax.z - minBound.z) / voxelSize.z), 0, nz - 1);

    // Count triangles per voxel using atomic operations
    for (size_t ix = ix0; ix <= ix1; ++ix) {
        for (size_t iy = iy0; iy <= iy1; ++iy) {
            for (size_t iz = iz0; iz <= iz1; ++iz) {
                size_t idx = ix + nx * (iy + ny * iz);
                atomicAdd(&voxel_tri_counts[idx], 1U);
            }
        }
    }
}

// Kernel to fill in triangle indices for each voxel
__global__ void FillTriangleIndicesKernel(
    const float* coords, const unsigned int* tris, size_t num_tris,
    size_t nx, size_t ny, size_t nz,
    cg_datastructures::Vec3 minBound, cg_datastructures::Vec3 voxelSize,
    const unsigned int* voxel_start_indices,
    unsigned int* voxel_current_counts,
    unsigned int* triangle_indices)
{
    size_t t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= num_tris) return;

    unsigned int i0 = tris[t * 3 + 0];
    unsigned int i1 = tris[t * 3 + 1];
    unsigned int i2 = tris[t * 3 + 2];

    cg_datastructures::Vec3 v0 = { coords[i0 * 3 + 0], coords[i0 * 3 + 1], coords[i0 * 3 + 2] };
    cg_datastructures::Vec3 v1 = { coords[i1 * 3 + 0], coords[i1 * 3 + 1], coords[i1 * 3 + 2] };
    cg_datastructures::Vec3 v2 = { coords[i2 * 3 + 0], coords[i2 * 3 + 1], coords[i2 * 3 + 2] };

    // Triangle AABB
    cg_datastructures::Vec3 triMin = {
        dmin(dmin(v0.x, v1.x), v2.x),
        dmin(dmin(v0.y, v1.y), v2.y),
        dmin(dmin(v0.z, v1.z), v2.z)
    };
    cg_datastructures::Vec3 triMax = {
        dmax(dmax(v0.x, v1.x), v2.x),
        dmax(dmax(v0.y, v1.y), v2.y),
        dmax(dmax(v0.z, v1.z), v2.z)
    };

    // Voxel range
    size_t ix0 = dclamp((size_t)((triMin.x - minBound.x) / voxelSize.x), 0, nx - 1);
    size_t iy0 = dclamp((size_t)((triMin.y - minBound.y) / voxelSize.y), 0, ny - 1);
    size_t iz0 = dclamp((size_t)((triMin.z - minBound.z) / voxelSize.z), 0, nz - 1);
    size_t ix1 = dclamp((size_t)((triMax.x - minBound.x) / voxelSize.x), 0, nx - 1);
    size_t iy1 = dclamp((size_t)((triMax.y - minBound.y) / voxelSize.y), 0, ny - 1);
    size_t iz1 = dclamp((size_t)((triMax.z - minBound.z) / voxelSize.z), 0, nz - 1);

    // Fill triangle indices using atomic operations
    for (size_t ix = ix0; ix <= ix1; ++ix) {
        for (size_t iy = iy0; iy <= iy1; ++iy) {
            for (size_t iz = iz0; iz <= iz1; ++iz) {
                size_t voxel_idx = ix + nx * (iy + ny * iz);
                unsigned int start_idx = voxel_start_indices[voxel_idx];
                unsigned int offset = atomicAdd(&voxel_current_counts[voxel_idx], 1U);
                triangle_indices[start_idx + offset] = static_cast<unsigned int>(t);
            }
        }
    }
}

VoxelGrid BuildVoxelGridFromStlMeshCuda(const StlMeshCuda& mesh, size_t nx, size_t ny, size_t nz) {
    std::cout << "[CUDA] Starting CUDA voxelization..." << std::endl;

    // Step 1: Compute mesh AABB on host
    std::cout << "[CUDA] Computing bounding box..." << std::endl;
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

    std::vector<float> h_coords(mesh.num_vrts * 3);
    cudaMemcpy(h_coords.data(), mesh.coords, mesh.num_vrts * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < mesh.num_vrts; ++i) {
        float x = h_coords[i * 3 + 0];
        float y = h_coords[i * 3 + 1];
        float z = h_coords[i * 3 + 2];
        minBound.x = std::min(minBound.x, x);
        minBound.y = std::min(minBound.y, y);
        minBound.z = std::min(minBound.z, z);
        maxBound.x = std::max(maxBound.x, x);
        maxBound.y = std::max(maxBound.y, y);
        maxBound.z = std::max(maxBound.z, z);
    }

    std::cout << "[CUDA] Bounding box: Min(" << minBound.x << ", " << minBound.y << ", " << minBound.z
              << ") Max(" << maxBound.x << ", " << maxBound.y << ", " << maxBound.z << ")" << std::endl;

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

    std::cout << "[CUDA] Uniform voxel size: " << uniformVoxelSize << std::endl;
    std::cout << "[CUDA] Grid dimensions: (" << gridSize.x << ", " << gridSize.y << ", " << gridSize.z << ")" << std::endl;

    // Step 2: Allocate device memory for counting triangles per voxel
    size_t num_voxels = nx * ny * nz;
    unsigned int* d_voxel_tri_counts;
    cudaMalloc(&d_voxel_tri_counts, num_voxels * sizeof(unsigned int));
    cudaMemset(d_voxel_tri_counts, 0, num_voxels * sizeof(unsigned int));

    // Step 3: Launch kernel to count triangles per voxel
    std::cout << "[CUDA] Counting triangles per voxel..." << std::endl;
    int threadsPerBlock = 128;
    int blocks = (int)((mesh.num_tris + threadsPerBlock - 1) / threadsPerBlock);

    CountTrianglesPerVoxelKernel<<<blocks, threadsPerBlock>>>(
        mesh.coords, mesh.tris, mesh.num_tris,
        nx, ny, nz, minBound, voxelSize, d_voxel_tri_counts
    );
    cudaDeviceSynchronize();

    // Step 4: Copy counts back to host and compute prefix sum
    std::vector<unsigned int> h_voxel_tri_counts(num_voxels);
    cudaMemcpy(h_voxel_tri_counts.data(), d_voxel_tri_counts, num_voxels * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    std::vector<unsigned int> h_voxel_start_indices(num_voxels);
    unsigned int total_tri_refs = 0;
    size_t occupied_count = 0;

    for (size_t i = 0; i < num_voxels; ++i) {
        h_voxel_start_indices[i] = total_tri_refs;
        total_tri_refs += h_voxel_tri_counts[i];
        if (h_voxel_tri_counts[i] > 0) {
            occupied_count++;
        }
    }

    std::cout << "[CUDA] Occupied voxels with triangles: " << occupied_count << " / " << num_voxels << std::endl;
    std::cout << "[CUDA] Total triangle references: " << total_tri_refs << std::endl;

    // Additional stats
    if (occupied_count > 0) {
        float avgTrianglesPerOccupiedVoxel = static_cast<float>(total_tri_refs) / occupied_count;
        std::cout << "[CUDA] Average triangles per occupied voxel: " << avgTrianglesPerOccupiedVoxel << std::endl;
    }

    // Step 5: Allocate device memory for triangle indices
    unsigned int* d_voxel_start_indices;
    unsigned int* d_voxel_current_counts;
    unsigned int* d_triangle_indices;

    cudaMalloc(&d_voxel_start_indices, num_voxels * sizeof(unsigned int));
    cudaMalloc(&d_voxel_current_counts, num_voxels * sizeof(unsigned int));
    cudaMalloc(&d_triangle_indices, total_tri_refs * sizeof(unsigned int));

    cudaMemcpy(d_voxel_start_indices, h_voxel_start_indices.data(), num_voxels * sizeof(unsigned int), cudaMemcpyHostToDevice);
    cudaMemset(d_voxel_current_counts, 0, num_voxels * sizeof(unsigned int));

    // Step 6: Launch kernel to fill triangle indices
    std::cout << "[CUDA] Filling triangle indices..." << std::endl;
    FillTriangleIndicesKernel<<<blocks, threadsPerBlock>>>(
        mesh.coords, mesh.tris, mesh.num_tris,
        nx, ny, nz, minBound, voxelSize,
        d_voxel_start_indices, d_voxel_current_counts, d_triangle_indices
    );
    cudaDeviceSynchronize();

    // Step 7: Copy triangle indices back to host
    std::vector<unsigned int> h_triangle_indices(total_tri_refs);
    cudaMemcpy(h_triangle_indices.data(), d_triangle_indices, total_tri_refs * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    // Step 8: Build triangle list on host
    std::cout << "[CUDA] Building triangle list..." << std::endl;
    std::vector<cg_datastructures::Triangle> triangles;
    triangles.reserve(mesh.num_tris);

    std::vector<float> h_normals(mesh.num_tris * 3);
    cudaMemcpy(h_normals.data(), mesh.normals, mesh.num_tris * 3 * sizeof(float), cudaMemcpyDeviceToHost);

    std::vector<unsigned int> h_tris(mesh.num_tris * 3);
    cudaMemcpy(h_tris.data(), mesh.tris, mesh.num_tris * 3 * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    for (size_t t = 0; t < mesh.num_tris; ++t) {
        unsigned int i0 = h_tris[t * 3 + 0];
        unsigned int i1 = h_tris[t * 3 + 1];
        unsigned int i2 = h_tris[t * 3 + 2];

        cg_datastructures::Vec3 v0 = {
            h_coords[i0 * 3 + 0],
            h_coords[i0 * 3 + 1],
            h_coords[i0 * 3 + 2]
        };
        cg_datastructures::Vec3 v1 = {
            h_coords[i1 * 3 + 0],
            h_coords[i1 * 3 + 1],
            h_coords[i1 * 3 + 2]
        };
        cg_datastructures::Vec3 v2 = {
            h_coords[i2 * 3 + 0],
            h_coords[i2 * 3 + 1],
            h_coords[i2 * 3 + 2]
        };
        cg_datastructures::Vec3 normal = {
            h_normals[t * 3 + 0],
            h_normals[t * 3 + 1],
            h_normals[t * 3 + 2]
        };

        triangles.emplace_back(cg_datastructures::Triangle{ v0, v1, v2, normal });
    }

    // Step 9: Create CUDA-compatible voxels
    std::cout << "[CUDA] Creating final voxel grid..." << std::endl;
    std::vector<cg_datastructures::Voxel> voxels(num_voxels);

    for (size_t i = 0; i < num_voxels; ++i) {
        voxels[i].occupied = (h_voxel_tri_counts[i] > 0);
        voxels[i].triangle_start_idx = h_voxel_start_indices[i];
        voxels[i].triangle_count = h_voxel_tri_counts[i];
        voxels[i].density = 0.0f;
        voxels[i].color = { 0.0f, 0.0f, 0.0f };
    }

    // Cleanup device memory
    cudaFree(d_voxel_tri_counts);
    cudaFree(d_voxel_start_indices);
    cudaFree(d_voxel_current_counts);
    cudaFree(d_triangle_indices);

    std::cout << "[CUDA] CUDA voxelization complete!" << std::endl;

    // Return the complete voxel grid
    return VoxelGrid{
        std::move(voxels),
        std::move(h_triangle_indices),
        nx, ny, nz,
        minBound, maxBound, voxelSize,
        std::move(triangles)
    };
}