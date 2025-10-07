#pragma once
#include <vector>
using namespace std;

class Voxels
{
public : struct Vec3 {
        float x, y, z;
    };

public : struct Triangle {
        Vec3 normal;     // Optional: may be zeroed if not used
        Vec3 v0, v1, v2; // Three vertices of the triangle
    };

public : struct Voxel {
        bool occupied = false;                  // Is this voxel filled?
        vector<int> triangleIndices;       // Indices of triangles intersecting this voxel
        float density = 0.0f;                   // Optional: for volumetric rendering
        Vec3 color = { 0.0f, 0.0f, 0.0f };         // Optional: for debug or shading
    };


};

