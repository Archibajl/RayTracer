#pragma once

#include <iostream>
#include <cuda_device_runtime_api.h>
#include <cuda_runtime_api.h>
#include "StlMesh.h"
#include "STLReader.h"
#include "Logger.h"

class MeshGenerator {
public:
    StlMeshCuda generateMeshes(const std::string filePath);

private:
    stl_reader::StlMesh<float, unsigned int> readStl(std::string filename);
    StlMeshCuda convertMesh(stl_reader::StlMesh<float, unsigned int> mesh);
    void printMeshInfoStlMesh(const stl_reader::StlMesh<float, unsigned int>& mesh);
    void printMeshInfoStlMesh(const StlMeshCuda& mesh);
};
