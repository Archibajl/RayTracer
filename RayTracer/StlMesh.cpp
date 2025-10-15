#include "StlMesh.h"
#include "STLReader.h"
#include <cuda_runtime_api.h>

// Host-side mesh (for CPU code)
struct HostStlMesh {
    std::vector<float> coords;
    std::vector<float> normals;
    std::vector<unsigned int> tris;
    size_t num_vrts;
    size_t num_tris;
};

// Device-side mesh (for CUDA kernels)
struct DeviceStlMesh {
    const float* coords;
    const float* normals;
    const unsigned int* tris;
    size_t num_vrts;
    size_t num_tris;
    // __host__ __device__ methods as needed
};

// Convert stl_reader::StlMesh to HostStlMesh
HostStlMesh toHostMesh(const stl_reader::StlMesh<float, unsigned int>& mesh) {
    HostStlMesh hostMesh;
    hostMesh.num_vrts = mesh.num_vrts();
    hostMesh.num_tris = mesh.num_tris();
    hostMesh.coords.assign(mesh.raw_coords(), mesh.raw_coords() + hostMesh.num_vrts * 3);
    hostMesh.normals.assign(mesh.raw_normals(), mesh.raw_normals() + hostMesh.num_tris * 3);
    hostMesh.tris.assign(mesh.raw_tris(), mesh.raw_tris() + hostMesh.num_tris * 3);
    return hostMesh;
}

// Copy HostStlMesh to device and return DeviceStlMesh
DeviceStlMesh toDeviceMesh(const HostStlMesh& hostMesh) {
    DeviceStlMesh deviceMesh;
    deviceMesh.num_vrts = hostMesh.num_vrts;
    deviceMesh.num_tris = hostMesh.num_tris;
    cudaMalloc((void**)&deviceMesh.coords, hostMesh.coords.size() * sizeof(float));
    cudaMalloc((void**)&deviceMesh.normals, hostMesh.normals.size() * sizeof(float));
    cudaMalloc((void**)&deviceMesh.tris, hostMesh.tris.size() * sizeof(unsigned int));
    cudaMemcpy((void*)deviceMesh.coords, hostMesh.coords.data(), hostMesh.coords.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)deviceMesh.normals, hostMesh.normals.data(), hostMesh.normals.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy((void*)deviceMesh.tris, hostMesh.tris.data(), hostMesh.tris.size() * sizeof(unsigned int), cudaMemcpyHostToDevice);
    return deviceMesh;
}
