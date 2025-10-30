#pragma once

#include "Voxels.h"

// Helper structures for voxelization
struct MeshBounds {
    cg_datastructures::Vec3 minBound;
    cg_datastructures::Vec3 maxBound;
};

struct VoxelizationParams {
    cg_datastructures::Vec3 voxelSize;
    cg_datastructures::Vec3 minBound;
    cg_datastructures::Vec3 maxBound;
    int nx, ny, nz;
};
