#include "MeshGenerator.h"
#include <filesystem>

//Cuda Compatible Mesh generator
StlMeshCuda MeshGenerator::generateMeshes(const std::string filePath) {
        stl_reader::StlMesh<float, unsigned int> stlObject = readStl(filePath);

        LOG_INFO("Read STL file with {} triangles", stlObject.num_tris());

        StlMeshCuda mesh = convertMesh(stlObject);

	return mesh;
}

stl_reader::StlMesh<float, unsigned int> MeshGenerator::readStl(std::string filename) {
        // Convert to absolute path for better error messages
        std::filesystem::path filePath(filename);
        std::string absolutePath;

        try {
            absolutePath = std::filesystem::absolute(filePath).string();
        } catch (...) {
            absolutePath = filename; // Fallback to original path
        }

        LOG_INFO("Attempting to load STL file: {}", absolutePath);

        // Check if file exists
        if (!std::filesystem::exists(filePath)) {
            std::string error = "STL file not found: " + absolutePath;
            LOG_ERROR(error);

            // Try to provide helpful suggestions
            if (std::filesystem::exists(filePath.parent_path())) {
                LOG_INFO("Directory exists, but file '{}' not found in it", filePath.filename().string());
                LOG_INFO("Checking directory contents...");

                try {
                    int fileCount = 0;
                    for (const auto& entry : std::filesystem::directory_iterator(filePath.parent_path())) {
                        if (entry.is_regular_file() && entry.path().extension() == ".stl") {
                            LOG_INFO("  Found: {}", entry.path().filename().string());
                            fileCount++;
                        }
                    }
                    if (fileCount == 0) {
                        LOG_INFO("  No .stl files found in directory");
                    }
                } catch (...) {
                    // Ignore errors listing directory
                }
            } else {
                LOG_ERROR("Directory does not exist: {}", filePath.parent_path().string());
            }

            throw std::runtime_error(error);
        }

        LOG_INFO("File exists, loading...");

        try {
            stl_reader::StlMesh<float, unsigned int> mesh(absolutePath);
            LOG_INFO("Successfully loaded {} triangles from STL file", mesh.num_tris());
            return mesh;
        }
        catch (const std::exception& e) {
            LOG_ERROR("Failed to parse STL file: {}", e.what());
            LOG_ERROR("File path: {}", absolutePath);
            throw;
        }
    }

StlMeshCuda MeshGenerator::convertMesh(stl_reader::StlMesh<float, unsigned int> mesh) {

        // Allocate device memory
        float* d_coords;
        float* d_normals;
        unsigned int* d_tris;

        size_t num_coords = mesh.num_vrts() * 3;
        size_t num_normals = mesh.num_tris() * 3;
        size_t num_tris = mesh.num_tris() * 3;

        // Fix: Cast the address to (void**) for cudaMalloc
        cudaMalloc((void**)&d_coords, num_coords * sizeof(float));
        cudaMalloc((void**)&d_normals, num_normals * sizeof(float));
        cudaMalloc((void**)&d_tris, num_tris * sizeof(unsigned int));

        cudaMemcpy(d_coords, mesh.raw_coords(), num_coords * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_normals, mesh.raw_normals(), num_normals * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_tris, mesh.raw_tris(), num_tris * sizeof(unsigned int), cudaMemcpyHostToDevice);

        StlMeshCuda h_mesh_dev;
        h_mesh_dev.coords = d_coords;
        h_mesh_dev.normals = d_normals;
        h_mesh_dev.tris = d_tris;
        h_mesh_dev.num_vrts = mesh.num_vrts();
        h_mesh_dev.num_tris = mesh.num_tris();

        StlMeshCuda* d_mesh_dev;
        // Fix the cudaMalloc call by casting the address to (void**)
        cudaMalloc((void**)&d_mesh_dev, sizeof(StlMeshCuda));
        cudaMemcpy(d_mesh_dev, &h_mesh_dev, sizeof(StlMeshCuda), cudaMemcpyHostToDevice);

        return h_mesh_dev;
}

void MeshGenerator::printMeshInfoStlMesh(const stl_reader::StlMesh<float, unsigned int>& mesh) {
        LOG_DEBUG("Number of vertices: {}", mesh.num_vrts());
        LOG_DEBUG("Number of triangles: {}", mesh.num_tris());

        // Iterate through all triangles
        for (size_t i = 0; i < mesh.num_tris(); ++i) {
            LOG_DEBUG("Triangle {}: ", i);
            for (size_t j = 0; j < 3; ++j) {
                const float* c = mesh.tri_corner_coords(i, j);
                //LOG_DEBUG("({}, {}, {}) ", c[0], c[1], c[2]);
            }
            const float* n = mesh.tri_normal(i);
            //LOG_DEBUG("Normal: ({}, {}, {})", n[0], n[1], n[2]);
        }
}

void MeshGenerator::printMeshInfoStlMesh(const StlMeshCuda& mesh) {
        LOG_DEBUG("Number of vertices: {}\n", mesh.num_vrts);
        LOG_DEBUG("Number of triangles: {}\n", mesh.num_tris);

        // Iterate through all triangles
        for (unsigned int i = 0; i < mesh.num_tris; ++i) {
            LOG_DEBUG("Triangle {}: ", i);
            for (unsigned int j = 0; j < 3; ++j) {
                const float* c = mesh.tri_corner_coords(i, j);
                LOG_DEBUG("({}, {}, {}) ", c[0], c[1], c[2]);
            }
            const float* n = mesh.tri_normal(i);
            LOG_DEBUG("Normal: ({}, {}, {})\n", n[0], n[1], n[2]);
        }
}