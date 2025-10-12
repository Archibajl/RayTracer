#pragma once
#include <vector>
#include <algorithm> // For std::find

namespace cg_datastructures {

struct Vec3 {
    float x, y, z;
};

struct Triangle {
    Vec3 normal;     // Optional: may be zeroed if not used
    Vec3 v0, v1, v2; // Three vertices of the triangle
};

struct Voxel {
    bool occupied = false;                        // Is this voxel filled?
    std::vector<const Triangle*> triangles;      // Use const if triangles are not modified
    float density = 0.0f;                         // Optional: for volumetric rendering
    Vec3 color = { 0.0f, 0.0f, 0.0f };            // Optional: for debug or shading

    // Add a triangle pointer if not already present
    void addTriangle(const Triangle* tri) {
        if (std::find(triangles.begin(), triangles.end(), tri) == triangles.end()) {
            triangles.push_back(tri);
        }
    }

    // Check if a triangle pointer is present
    bool hasTriangle(const Triangle* tri) const {
        return std::find(triangles.begin(), triangles.end(), tri) != triangles.end();
    }
};

} // namespace Voxels
