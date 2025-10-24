#pragma once

#include "IRayTracer.h"
#include "SceneCreator.h"
#include "VectorMath.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

/**
 * Octree Ray Tracer
 *
 * Implementation of octree-based space subdivision for ray tracing,
 * based on Andrew S. Glassner's 1984 paper "Space subdivision for fast ray tracing"
 * (IEEE Computer Graphics and Applications, Vol. 4, no. 10, pp 15-22).
 *
 * Key features of octree subdivision:
 * - Non-uniform adaptive spatial subdivision
 * - Density-dependent hierarchy: subdivides only regions with many objects
 * - Automatic generation of spatial hierarchy based on scene complexity
 * - Only tests objects in octree nodes along the ray path
 * - Superior performance for non-uniformly distributed geometry
 *
 * Revolutionary for 1984: enabled ray tracing of hundreds/thousands of objects,
 * which was previously "just unthinkable for ray tracing" at the time.
 */
class OctreeRayTracer : public IRayTracer {
public:
    OctreeRayTracer(const VoxelGrid& grid, int maxDepth = 8, int maxTrianglesPerNode = 10);
    ~OctreeRayTracer() override = default;

    std::vector<cg_datastructures::Vec3> render(
        const cg_datastructures::Camera& camera,
        int width, int height) override;

    cg_datastructures::RayHit traceRay(const cg_datastructures::Ray& ray) override;

    std::string getMethodName() const override { return "Octree"; }

private:
    // Octree node structure
    struct OctreeNode {
        cg_datastructures::Vec3 minBound;
        cg_datastructures::Vec3 maxBound;
        std::vector<unsigned int> triangleIndices;  // Triangles in this node
        std::unique_ptr<OctreeNode> children[8];     // 8 children for octree
        bool isLeaf;

        OctreeNode(const cg_datastructures::Vec3& min, const cg_datastructures::Vec3& max)
            : minBound(min), maxBound(max), isLeaf(true) {
            for (int i = 0; i < 8; ++i) {
                children[i] = nullptr;
            }
        }
    };

    const VoxelGrid& voxelGrid;
    std::unique_ptr<OctreeNode> root;
    int maxDepth;
    int maxTrianglesPerNode;

    // Octree building
    void buildOctree();
    void subdivideNode(OctreeNode* node, int depth);
    void createChildNodes(OctreeNode* node, const cg_datastructures::Vec3& center);
    void distributeTrianglesToChildren(OctreeNode* node);
    bool triangleIntersectsBox(const cg_datastructures::Triangle& tri,
                               const cg_datastructures::Vec3& boxMin,
                               const cg_datastructures::Vec3& boxMax);
    bool checkTriangleBoxAABBOverlap(const cg_datastructures::Vec3& v0,
                                     const cg_datastructures::Vec3& v1,
                                     const cg_datastructures::Vec3& v2,
                                     const cg_datastructures::Vec3& boxHalfSize) const;
    bool checkTrianglePlaneBoxIntersection(const cg_datastructures::Vec3& v0,
                                           const cg_datastructures::Vec3& v1,
                                           const cg_datastructures::Vec3& v2,
                                           const cg_datastructures::Vec3& boxHalfSize) const;

    // Octree traversal
    cg_datastructures::RayHit traverseOctree(const cg_datastructures::Ray& ray);
    void traverseNode(const cg_datastructures::Ray& ray,
                     const OctreeNode* node,
                     cg_datastructures::RayHit& result);
    void testLeafNodeTriangles(const cg_datastructures::Ray& ray,
                               const OctreeNode* node,
                               cg_datastructures::RayHit& result);

    // Rendering helper methods
    cg_datastructures::Vec3 computeShading(const cg_datastructures::RayHit& hit, float v) const;

    // Helper functions
    cg_datastructures::Ray generateRay(const cg_datastructures::Camera& camera, float u, float v);
    bool rayBoxIntersection(const cg_datastructures::Ray& ray,
                           const cg_datastructures::Vec3& boxMin,
                           const cg_datastructures::Vec3& boxMax,
                           float& tMin, float& tMax);
    bool rayTriangleIntersection(const cg_datastructures::Ray& ray,
                                const cg_datastructures::Triangle& tri,
                                float& t, float& u, float& v);

    // Statistics
    mutable unsigned int nodesVisited = 0;
    mutable unsigned int triangleTests = 0;
    unsigned int totalNodes = 0;
    unsigned int leafNodes = 0;
};
