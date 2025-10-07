#pragma once
#include "Voxels.h"

#include <vector>
#include <string>

using namespace std;
class SceneCreator
{

    struct STLMesh {
        std::vector<Voxels::Triangle> triangles;

        // Optional metadata
        string name;
        Voxels::Vec3 boundingBoxMin;
        Voxels::Vec3 boundingBoxMax;

        // Compute bounding box after loading
        void computeBoundingBox() {
            if (triangles.empty()) return;
            boundingBoxMin = boundingBoxMax = triangles[0].v0;
            for (const auto& tri : triangles) {
                for (const Voxels::Vec3& v : { tri.v0, tri.v1, tri.v2 }) {
                    boundingBoxMin.x = std::min(boundingBoxMin.x, v.x);
                    boundingBoxMin.y = std::min(boundingBoxMin.y, v.y);
                    boundingBoxMin.z = std::min(boundingBoxMin.z, v.z);
                    boundingBoxMax.x = std::max(boundingBoxMax.x, v.x);
                    boundingBoxMax.y = std::max(boundingBoxMax.y, v.y);
                    boundingBoxMax.z = std::max(boundingBoxMax.z, v.z);
                }
            }
        }
    };

};

