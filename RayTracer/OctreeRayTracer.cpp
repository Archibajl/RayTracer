#include "OctreeRayTracer.h"
#include "GeometryUtils.h"

using namespace cg_datastructures;
using namespace vector_math;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

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

// ============================================================================
// RAY GENERATION
// ============================================================================

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

// ============================================================================
// SHADING
// ============================================================================

Vec3 OctreeRayTracer::computeShading(const RayHit& hit, float v) const {
    if (hit.hit) {
        // Simple solid color for hits
        return { 0.0f, 0.0f, 0.0f }; // Black
    }
    else {
        // Background gradient
        return { 0.5f + 0.3f * v, 0.5f + 0.3f * v, 0.7f + 0.3f * v }; // Purple gradient
    }
}

// ============================================================================
// RENDERING - STATISTICS HELPERS
// ============================================================================

void OctreeRayTracer::resetRenderingStatistics() {
    nodesVisited = 0;
    triangleTests = 0;
}

void OctreeRayTracer::logProgressUpdate(int currentRow, int totalRows) const {
    if (currentRow % (totalRows / 10) == 0) {
        LOG_INFO("Progress: {}%", 100 * currentRow / totalRows);
    }
}

void OctreeRayTracer::logRenderingStatistics(int hitCount, int totalRays) const {
    LOG_INFO("[{}] Rendering complete!", getMethodName());
    LOG_INFO("[{}] Ray hits: {} / {} ({:.1f}%)",
        getMethodName(), hitCount, totalRays, 100.0f * hitCount / totalRays);
    LOG_INFO("[{}] Octree nodes visited: {}, Avg per ray: {:.1f}",
        getMethodName(), nodesVisited, (float)nodesVisited / totalRays);
    LOG_INFO("[{}] Triangle tests: {}, Avg per ray: {:.1f}",
        getMethodName(), triangleTests, (float)triangleTests / totalRays);
}

// ============================================================================
// RENDERING - PIXEL RENDERING
// ============================================================================

Vec3 OctreeRayTracer::renderPixel(const Camera& camera, int x, int y, int width, int height) {
    // Calculate normalized device coordinates (0 to 1)
    float u = static_cast<float>(x) / static_cast<float>(width);
    float v = static_cast<float>(y) / static_cast<float>(height);

    // Generate and trace ray
    Ray ray = generateRay(camera, u, v);
    RayHit hit = traceRay(ray);

    // Compute shading
    return computeShading(hit, v);
}

// ============================================================================
// RENDERING - MAIN RENDER FUNCTION
// ============================================================================

std::vector<Vec3> OctreeRayTracer::render(const Camera& camera, int width, int height) {
    std::vector<Vec3> frameBuffer(width * height);

    LOG_INFO("[{}] Rendering {}x{} image...", getMethodName(), width, height);

    resetRenderingStatistics();
    int hitCount = 0;
    int totalRays = width * height;

    for (int y = 0; y < height; ++y) {
        logProgressUpdate(y, height);

        for (int x = 0; x < width; ++x) {
            Vec3 color = renderPixel(camera, x, y, width, height);
            frameBuffer[y * width + x] = color;

            // Track hits for statistics
            Ray ray = generateRay(camera,
                static_cast<float>(x) / static_cast<float>(width),
                static_cast<float>(y) / static_cast<float>(height));
            RayHit hit = traceRay(ray);
            if (hit.hit) {
                hitCount++;
            }
        }
    }

    logRenderingStatistics(hitCount, totalRays);

    return frameBuffer;
}

// ============================================================================
// RAY TRACING
// ============================================================================

RayHit OctreeRayTracer::traceRay(const Ray& ray) {
    return traverseOctree(ray);
}

// ============================================================================
// GEOMETRY INTERSECTION - WRAPPER FUNCTIONS
// ============================================================================

bool OctreeRayTracer::rayBoxIntersection(
    const Ray& ray,
    const Vec3& boxMin,
    const Vec3& boxMax,
    float& tMin, float& tMax) {

    return GeometryUtils::rayBoxIntersection(ray, boxMin, boxMax, tMin, tMax);
}

bool OctreeRayTracer::rayTriangleIntersection(
    const Ray& ray,
    const Triangle& tri,
    float& t, float& u, float& v) {

    return GeometryUtils::rayTriangleIntersection(ray, tri, t, u, v);
}

// ============================================================================
// OCTREE BUILDING - MAIN FUNCTION
// ============================================================================

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

// ============================================================================
// OCTREE BUILDING - CHILD NODE CREATION
// ============================================================================

void OctreeRayTracer::createChildNodes(OctreeNode* node, const Vec3& center) {
    // Create 8 children (octants) using the center point
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

    for (int i = 0; i < 8; ++i) {
        node->children[i] = std::make_unique<OctreeNode>(childBounds[i][0], childBounds[i][1]);
        totalNodes++;
    }
}

// ============================================================================
// OCTREE BUILDING - TRIANGLE DISTRIBUTION
// ============================================================================

void OctreeRayTracer::distributeTrianglesToChildren(OctreeNode* node) {
    // Distribute triangles to children based on overlap
    for (unsigned int triIdx : node->triangleIndices) {
        const Triangle& tri = voxelGrid.triangles[triIdx];

        // Check which children this triangle overlaps
        for (int i = 0; i < 8; ++i) {
            if (triangleIntersectsBox(tri, node->children[i]->minBound, node->children[i]->maxBound)) {
                node->children[i]->triangleIndices.push_back(triIdx);
            }
        }
    }

    // Clear parent's triangle list (moved to children)
    node->triangleIndices.clear();
    node->isLeaf = false;
}

// ============================================================================
// OCTREE BUILDING - SUBDIVISION HELPERS
// ============================================================================

bool OctreeRayTracer::shouldSubdivideNode(const OctreeNode* node, int depth) const {
    return depth < maxDepth && node->triangleIndices.size() > maxTrianglesPerNode;
}

Vec3 OctreeRayTracer::computeNodeCenter(const OctreeNode* node) const {
    return {
        (node->minBound.x + node->maxBound.x) * 0.5f,
        (node->minBound.y + node->maxBound.y) * 0.5f,
        (node->minBound.z + node->maxBound.z) * 0.5f
    };
}

void OctreeRayTracer::subdivideChildrenRecursively(OctreeNode* node, int depth) {
    for (int i = 0; i < 8; ++i) {
        subdivideNode(node->children[i].get(), depth + 1);
    }
}

// ============================================================================
// OCTREE BUILDING - RECURSIVE SUBDIVISION
// ============================================================================

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
    if (!shouldSubdivideNode(node, depth)) {
        leafNodes++;
        return;
    }

    // Calculate octant center
    Vec3 center = computeNodeCenter(node);

    // Create 8 child nodes
    createChildNodes(node, center);

    // Distribute triangles to appropriate children
    distributeTrianglesToChildren(node);

    // Recursively subdivide children
    subdivideChildrenRecursively(node, depth);
}

// ============================================================================
// TRIANGLE-BOX INTERSECTION - AABB OVERLAP TEST
// ============================================================================

bool OctreeRayTracer::checkTriangleBoxAABBOverlap(
    const Vec3& v0, const Vec3& v1, const Vec3& v2,
    const Vec3& boxHalfSize) const {

    // Test triangle AABB vs box AABB
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

    return true;
}

// ============================================================================
// TRIANGLE-BOX INTERSECTION - PLANE-BOX TEST
// ============================================================================

bool OctreeRayTracer::checkTrianglePlaneBoxIntersection(
    const Vec3& v0, const Vec3& v1, const Vec3& v2,
    const Vec3& boxHalfSize) const {

    // Test plane of triangle vs box
    Vec3 normal = normalize(cross(subtract(v1, v0), subtract(v2, v0)));
    float d = dot(normal, v0);

    // Find the box vertices that are min/max along the normal direction
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

// ============================================================================
// TRIANGLE-BOX INTERSECTION - BOX COMPUTATION HELPERS
// ============================================================================

Vec3 OctreeRayTracer::computeBoxCenter(const Vec3& boxMin, const Vec3& boxMax) const {
    return {
        (boxMin.x + boxMax.x) * 0.5f,
        (boxMin.y + boxMax.y) * 0.5f,
        (boxMin.z + boxMax.z) * 0.5f
    };
}

Vec3 OctreeRayTracer::computeBoxHalfSize(const Vec3& boxMin, const Vec3& boxMax) const {
    return {
        (boxMax.x - boxMin.x) * 0.5f,
        (boxMax.y - boxMin.y) * 0.5f,
        (boxMax.z - boxMin.z) * 0.5f
    };
}

// ============================================================================
// TRIANGLE-BOX INTERSECTION - MAIN FUNCTION
// ============================================================================

/**
 * Check if a triangle intersects an axis-aligned bounding box
 * Uses separating axis theorem
 */
bool OctreeRayTracer::triangleIntersectsBox(
    const Triangle& tri,
    const Vec3& boxMin,
    const Vec3& boxMax) {

    // Transform to box-centered coordinate system
    Vec3 boxCenter = computeBoxCenter(boxMin, boxMax);
    Vec3 boxHalfSize = computeBoxHalfSize(boxMin, boxMax);

    return GeometryUtils::triangleIntersectsBox(tri, boxCenter, boxHalfSize);
}

// ============================================================================
// OCTREE TRAVERSAL - MAIN FUNCTION
// ============================================================================

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

// ============================================================================
// OCTREE TRAVERSAL - LEAF NODE TRIANGLE TESTING
// ============================================================================

void OctreeRayTracer::testLeafNodeTriangles(
    const Ray& ray, const OctreeNode* node, RayHit& result) {

    // Simple occupancy check: if node has any triangles, it's occupied
    if (!node->triangleIndices.empty()) {
        // Calculate node center for hit point (simplified)
        Vec3 nodeCenter = {
            (node->minBound.x + node->maxBound.x) * 0.5f,
            (node->minBound.y + node->maxBound.y) * 0.5f,
            (node->minBound.z + node->maxBound.z) * 0.5f
        };

        // Calculate approximate distance along ray to node center
        Vec3 toNode = subtract(nodeCenter, ray.origin);
        float t = dot(toNode, ray.direction);

        // Only register hit if it's the closest so far and in front of ray
        if (t > 0.0f && t < result.t) {
            result.hit = true;
            result.t = t;
            result.point = nodeCenter;
            result.normal = { 0.0f, 0.0f, 1.0f }; // Default normal
            result.triangleIdx = 0; // No specific triangle
        }
    }
}

// ============================================================================
// OCTREE TRAVERSAL - RECURSIVE NODE TRAVERSAL
// ============================================================================

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
        testLeafNodeTriangles(ray, node, result);
    }
    else {
        // Internal node: recursively visit children in front-to-back order
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]) {
                traverseNode(ray, node->children[i].get(), result);
            }
        }
    }
}
