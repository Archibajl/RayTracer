#pragma once

#include <string>
#include <vector>
#include "SceneCreator.h"

namespace cg_datastructures {
    struct Vec3;
}

class TraceImages {
public:
    void TraceImage(std::string gridFileLocation, std::string outputFileName);

private:
    void genateImageFromGrid(VoxelGrid voxelGrid, std::string filename);
    VoxelGrid generateVoxelGridFromFile(const std::string filepath, int nx, int ny, int nz);
    VoxelGrid generateVoxelGridFromMesh(StlMeshCuda mesh, int nx, int ny, int nz);
    void SaveImage(const std::string& filename,
                   const std::vector<cg_datastructures::Vec3>& pixels,
                   int width, int height);
};
