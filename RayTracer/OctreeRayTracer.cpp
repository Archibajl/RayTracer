#include "OctreeRayTracer.h"

using namespace cg_datastructures;
using namespace vector_math;

OctreeRayTracer::OctreeRayTracer(const VoxelGrid& grid, int maxDepth, int maxTrianglesPerNode)
    : voxelGrid(grid), maxDepth(maxDepth), maxTrianglesPerNode(maxTrianglesPerNode) {

    LOG_INFO("[{}] Initializing octree...", getMethodName());
    LOG_INFO("[{}] Max depth: {}, Max triangles per node: {}",
        getMethodName(), maxDepth, maxTrianglesPerNode);
    LOG_INFO("[{}] Based on Glassner 1984 space subdivision algorithm", getMethodName());

    buildOctree();

    LOG_INFO("[{}] Octree built: {} total nodes, {} leaf nodes",
        getMethodName(), totalNodes, leafNodes);
}

Ray OctreeRayTracer::generateRay(const Camera& camera, float u, float v) {
    // Convert FOV to radians
    float fovRad = camera.fov * 3.14159265f / 180.0f;
    float halfHeight = std::tan(fovRad / 2.0f);
    float halfWidth = camera.aspectRatio * halfHeight;

    // Build camera basis
    Vec3 forward = normalize(subtract(camera.lookAt, camera.position));
    Vec3 right = normalize(cross(forward, camera.up));
    Vec3 up = cross(right, forward);

    // Calculate ray direction
    Vec3 horizontal = multiply(right, 2.0f * halfWidth);
    Vec3 vertical = multiply(up, 2.0f * halfHeight);

    // Image plane positioned 1 unit in front of camera
    Vec3 lowerLeft = add(subtract(subtract(camera.position, multiply(horizontal, 0.5f)),
        multiply(vertical, 0.5f)), forward);

    Vec3 direction = normalize(subtract(add(add(lowerLeft, multiply(horizontal, u)),
        multiply(vertical, v)), camera.position));

    return Ray(camera.position, direction);
}

std::vector<Vec3> OctreeRayTracer::render(const Camera& camera, int width, int height) {
    std::vector<Vec3> frameBuffer(width * height);

    LOG_INFO("[{}] Rendering {}x{} image...", getMethodName(), width, height);

    // Reset statistics
    nodesVisited = 0;
    triangleTests = 0;
    int hitCount = 0;
    int totalRays = width * height;

    for (int y = 0; y < height; ++y) {
        if (y % (height / 10) == 0) {
            LOG_INFO("Progress: {}%", 100 * y / height);
        }

        for (int x = 0; x < width; ++x) {
            // Calculate normalized device coordinates (0 to 1)
            float u = static_cast<float>(x) / static_cast<float>(width);
            float v = static_cast<float>(y) / static_cast<float>(height);

            // Generate ray
            Ray ray = generateRay(camera, u, v);

            // Trace ray using octree
            RayHit hit = traceRay(ray);

            // Simple shading
            Vec3 color;
            if (hit.hit) {
                hitCount++;
                // Normal-based shading
                Vec3 lightDir = normalize({ 1.0f, 1.0f, -1.0f });
                float diffuse = std::max(0.0f, dot(hit.normal, lightDir));
                float ambient = 0.3f;
                float intensity = std::min(1.0f, ambient + diffuse * 0.7f);

                // Octree gets a purple/magenta tint
                color = { intensity * 0.9f, intensity * 0.5f, intensity * 0.9f };
            }
            else {
                // Background gradient
                float t = v;
                color = { 0.5f + 0.3f * t, 0.5f + 0.3f * t, 0.7f + 0.3f * t }; // Purple gradient
            }

            frameBuffer[y * width + x] = color;
        }
    }

    LOG_INFO("[{}] Rendering complete!", getMethodName());
    LOG_INFO("[{}] Ray hits: {} / {} ({:.1f}%)",
        getMethodName(), hitCount, totalRays, 100.0f * hitCount / totalRays);
    LOG_INFO("[{}] Octree nodes visited: {}, Avg per ray: {:.1f}",
        getMethodName(), nodesVisited, (float)nodesVisited / totalRays);
    LOG_INFO("[{}] Triangle tests: {}, Avg per ray: {:.1f}",
        getMethodName(), triangleTests, (float)triangleTests / totalRays);

    return frameBuffer;
}

RayHit OctreeRayTracer::traceRay(const Ray& ray) {
    return traverseOctree(ray);
}

bool OctreeRayTracer::rayBoxIntersection(
    const Ray& ray,
    const Vec3& boxMin,
    const Vec3& boxMax,
    float& tMin, float& tMax) {

    const float EPSILON = 1e-6f;

    // Handle near-zero direction components
    float invDirX = (std::abs(ray.direction.x) > EPSILON) ? (1.0f / ray.direction.x) : 1e30f;
    float invDirY = (std::abs(ray.direction.y) > EPSILON) ? (1.0f / ray.direction.y) : 1e30f;
    float invDirZ = (std::abs(ray.direction.z) > EPSILON) ? (1.0f / ray.direction.z) : 1e30f;

    float tx1 = (boxMin.x - ray.origin.x) * invDirX;
    float tx2 = (boxMax.x - ray.origin.x) * invDirX;
    float ty1 = (boxMin.y - ray.origin.y) * invDirY;
    float ty2 = (boxMax.y - ray.origin.y) * invDirY;
    float tz1 = (boxMin.z - ray.origin.z) * invDirZ;
    float tz2 = (boxMax.z - ray.origin.z) * invDirZ;

    tMin = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)), std::min(tz1, tz2));
    tMax = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)), std::max(tz1, tz2));

    return tMax >= tMin && tMax >= 0.0f;
}

bool OctreeRayTracer::rayTriangleIntersection(
    const Ray& ray,
    const Triangle& tri,
    float& t, float& u, float& v) {

    const float EPSILON = 0.0000001f;

    Vec3 edge1 = subtract(tri.v1, tri.v0);
    Vec3 edge2 = subtract(tri.v2, tri.v0);

    Vec3 h = cross(ray.direction, edge2);
    float a = dot(edge1, h);

    // Ray is parallel to triangle
    if (a > -EPSILON && a < EPSILON) {
        return false;
    }

    float f = 1.0f / a;
    Vec3 s = subtract(ray.origin, tri.v0);
    u = f * dot(s, h);

    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    Vec3 q = cross(s, edge1);
    v = f * dot(ray.direction, q);

    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }

    // Calculate t to find intersection point
    t = f * dot(edge2, q);

    return t > EPSILON;
}

/**
 * Build octree from scene triangles
 *
 * Glassner's key insight: subdivide space adaptively based on object density.
 * This creates a hierarchy that naturally adapts to scene complexity.
 */
void OctreeRayTracer::buildOctree() {
    // Create root node encompassing entire scene
    root = std::make_unique<OctreeNode>(voxelGrid.minBound, voxelGrid.maxBound);
    totalNodes = 1;

    // Add all triangles to root
    for (unsigned int i = 0; i < voxelGrid.triangles.size(); ++i) {
        root->triangleIndices.push_back(i);
    }

    LOG_INFO("[{}] Starting subdivision with {} triangles",
        getMethodName(), voxelGrid.triangles.size());

    // Recursively subdivide
    subdivideNode(root.get(), 0);

    LOG_INFO("[{}] Subdivision complete", getMethodName());
}

/**
 * Subdivide a node if it contains too many triangles
 *
 * This implements Glassner's density-dependent subdivision:
 * - Only subdivide if node has more triangles than threshold
 * - Stop at maximum depth
 * - Distribute triangles to child nodes based on overlap
 */
void OctreeRayTracer::subdivideNode(OctreeNode* node, int depth) {
    // Stop conditions
    if (depth >= maxDepth || node->triangleIndices.size() <= maxTrianglesPerNode) {
        leafNodes++;
        return;
    }

    // Calculate octant bounds
    Vec3 center = {
        (node->minBound.x + node->maxBound.x) * 0.5f,
        (node->minBound.y + node->maxBound.y) * 0.5f,
        (node->minBound.z + node->maxBound.z) * 0.5f
    };

    // Create 8 children (octants)
    Vec3 childBounds[8][2] = {
        // Bottom 4 octants (z = min to center)
        { {node->minBound.x, node->minBound.y, node->minBound.z}, {center.x, center.y, center.z} },  // 0: ---
        { {center.x, node->minBound.y, node->minBound.z}, {node->maxBound.x, center.y, center.z} },  // 1: +--
        { {node->minBound.x, center.y, node->minBound.z}, {center.x, node->maxBound.y, center.z} },  // 2: -+-
        { {center.x, center.y, node->minBound.z}, {node->maxBound.x, node->maxBound.y, center.z} },  // 3: ++-
        // Top 4 octants (z = center to max)
        { {node->minBound.x, node->minBound.y, center.z}, {center.x, center.y, node->maxBound.z} },  // 4: --+
        { {center.x, node->minBound.y, center.z}, {node->maxBound.x, center.y, node->maxBound.z} },  // 5: +-+
        { {node->minBound.x, center.y, center.z}, {center.x, node->maxBound.y, node->maxBound.z} },  // 6: -++
        { {center.x, center.y, center.z}, {node->maxBound.x, node->maxBound.y, node->maxBound.z} }   // 7: +++
    };

    // Create child nodes
    for (int i = 0; i < 8; ++i) {
        node->children[i] = std::make_unique<OctreeNode>(childBounds[i][0], childBounds[i][1]);
        totalNodes++;
    }

    // Distribute triangles to children
    for (unsigned int triIdx : node->triangleIndices) {
        const Triangle& tri = voxelGrid.triangles[triIdx];

        // Check which children this triangle overlaps
        for (int i = 0; i < 8; ++i) {
            if (triangleIntersectsBox(tri, childBounds[i][0], childBounds[i][1])) {
                node->children[i]->triangleIndices.push_back(triIdx);
            }
        }
    }

    // Clear parent's triangle list (moved to children)
    node->triangleIndices.clear();
    node->isLeaf = false;

    // Recursively subdivide children
    for (int i = 0; i < 8; ++i) {
        subdivideNode(node->children[i].get(), depth + 1);
    }
}

/**
 * Check if a triangle intersects an axis-aligned bounding box
 * Uses separating axis theorem
 */
bool OctreeRayTracer::triangleIntersectsBox(
    const Triangle& tri,
    const Vec3& boxMin,
    const Vec3& boxMax) {

    Vec3 boxCenter = {
        (boxMin.x + boxMax.x) * 0.5f,
        (boxMin.y + boxMax.y) * 0.5f,
        (boxMin.z + boxMax.z) * 0.5f
    };

    Vec3 boxHalfSize = {
        (boxMax.x - boxMin.x) * 0.5f,
        (boxMax.y - boxMin.y) * 0.5f,
        (boxMax.z - boxMin.z) * 0.5f
    };

    // Translate triangle to box space (box centered at origin)
    Vec3 v0 = subtract(tri.v0, boxCenter);
    Vec3 v1 = subtract(tri.v1, boxCenter);
    Vec3 v2 = subtract(tri.v2, boxCenter);

    // Test 1: Triangle AABB vs box AABB
    Vec3 triMin = {
        std::min(std::min(v0.x, v1.x), v2.x),
        std::min(std::min(v0.y, v1.y), v2.y),
        std::min(std::min(v0.z, v1.z), v2.z)
    };

    Vec3 triMax = {
        std::max(std::max(v0.x, v1.x), v2.x),
        std::max(std::max(v0.y, v1.y), v2.y),
        std::max(std::max(v0.z, v1.z), v2.z)
    };

    if (triMin.x > boxHalfSize.x || triMax.x < -boxHalfSize.x) return false;
    if (triMin.y > boxHalfSize.y || triMax.y < -boxHalfSize.y) return false;
    if (triMin.z > boxHalfSize.z || triMax.z < -boxHalfSize.z) return false;

    // Test 2: Plane of triangle vs box
    Vec3 normal = normalize(cross(subtract(v1, v0), subtract(v2, v0)));
    float d = dot(normal, v0);

    Vec3 vmin, vmax;
    vmin.x = (normal.x > 0) ? -boxHalfSize.x : boxHalfSize.x;
    vmin.y = (normal.y > 0) ? -boxHalfSize.y : boxHalfSize.y;
    vmin.z = (normal.z > 0) ? -boxHalfSize.z : boxHalfSize.z;
    vmax.x = -vmin.x;
    vmax.y = -vmin.y;
    vmax.z = -vmin.z;

    if (dot(normal, vmin) + d > 0) return false;
    if (dot(normal, vmax) + d < 0) return false;

    return true;
}

/**
 * Traverse octree to find ray-triangle intersections
 *
 * Glassner's traversal: recursively visit nodes that the ray intersects,
 * testing triangles only in leaf nodes.
 */
RayHit OctreeRayTracer::traverseOctree(const Ray& ray) {
    RayHit result;
    result.hit = false;
    result.t = std::numeric_limits<float>::max();

    if (root) {
        traverseNode(ray, root.get(), result);
    }

    return result;
}

/**
 * Recursively traverse octree node
 */
void OctreeRayTracer::traverseNode(const Ray& ray, const OctreeNode* node, RayHit& result) {
    nodesVisited++; // Statistics

    // Check if ray intersects this node's bounding box
    float tMin, tMax;
    if (!rayBoxIntersection(ray, node->minBound, node->maxBound, tMin, tMax)) {
        return; // Ray doesn't intersect this node
    }

    // Early exit if we already found a closer hit
    if (result.hit && tMin > result.t) {
        return;
    }

    if (node->isLeaf) {
        // Leaf node: test all triangles
        for (unsigned int triIdx : node->triangleIndices) {
            triangleTests++; // Statistics

            const Triangle& tri = voxelGrid.triangles[triIdx];

            float t, u, v;
            if (rayTriangleIntersection(ray, tri, t, u, v)) {
                if (t < result.t) {
                    result.hit = true;
                    result.t = t;
                    result.point = add(ray.origin, multiply(ray.direction, t));
                    result.normal = normalize(tri.normal);
                    result.triangleIdx = triIdx;
                }
            }
        }
    }
    else {
        // Internal node: recursively visit children
        // Visit children in front-to-back order for early termination
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]) {
                traverseNode(ray, node->children[i].get(), result);
            }
        }
    }
}
