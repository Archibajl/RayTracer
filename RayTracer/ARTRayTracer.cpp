#include "ARTRayTracer.h"
#include "GeometryUtils.h"

using namespace cg_datastructures;
using namespace vector_math;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

ARTRayTracer::ARTRayTracer(const VoxelGrid& grid)
    : voxelGrid(grid) {
    LOG_INFO("[{}] Initialized with uniform grid: {}x{}x{}",
        getMethodName(), grid.nx, grid.ny, grid.nz);
    LOG_INFO("[{}] Using simple array-based traversal (all voxels)", getMethodName());
    LOG_INFO("[{}] Total voxels to test per ray: {}", getMethodName(), grid.nx * grid.ny * grid.nz);
}

// ============================================================================
// RAY GENERATION
// ============================================================================

Ray ARTRayTracer::generateRay(const Camera& camera, float u, float v) {
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

Vec3 ARTRayTracer::computeShading(const RayHit& hit, float v) const {
    if (hit.hit) {
        // Simple solid color for hits
        return { 0.0f, 0.0f, 0.0f }; // Black
    }
    else {
        // Background gradient
        return { 0.6f + 0.4f * v, 0.6f + 0.4f * v, 0.9f }; // Gray-blue gradient
    }
}

// ============================================================================
// RENDERING - STATISTICS HELPERS
// ============================================================================

void ARTRayTracer::resetRenderingStatistics() {
    voxelsTraversed = 0;
    trianglesTests = 0;
}

void ARTRayTracer::logProgressUpdate(int currentRow, int totalRows) const {
    if (currentRow % (totalRows / 10) == 0) {
        LOG_INFO("Progress: {}%", 100 * currentRow / totalRows);
    }
}

void ARTRayTracer::logRenderingStatistics(int hitCount, int totalRays) const {
    LOG_INFO("[{}] Rendering complete!", getMethodName());
    LOG_INFO("[{}] Ray hits: {} / {} ({:.1f}%)",
        getMethodName(), hitCount, totalRays, 100.0f * hitCount / totalRays);
    LOG_INFO("[{}] Voxels traversed: {}, Avg per ray: {:.1f}",
        getMethodName(), voxelsTraversed, (float)voxelsTraversed / totalRays);
    LOG_INFO("[{}] Triangle tests: {}, Avg per ray: {:.1f}",
        getMethodName(), trianglesTests, (float)trianglesTests / totalRays);
}

// ============================================================================
// RENDERING - PIXEL RENDERING
// ============================================================================

Vec3 ARTRayTracer::renderPixel(const Camera& camera, int x, int y, int width, int height) {
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

std::vector<Vec3> ARTRayTracer::render(const Camera& camera, int width, int height) {
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

RayHit ARTRayTracer::traceRay(const Ray& ray) {
    return traverse3DDDA(ray);
}

// ============================================================================
// GEOMETRY INTERSECTION - WRAPPER FUNCTIONS
// ============================================================================

bool ARTRayTracer::rayBoxIntersection(
    const Ray& ray,
    const Vec3& boxMin,
    const Vec3& boxMax,
    float& tMin, float& tMax) {

    return GeometryUtils::rayBoxIntersection(ray, boxMin, boxMax, tMin, tMax);
}

bool ARTRayTracer::rayTriangleIntersection(
    const Ray& ray,
    const Triangle& tri,
    float& t, float& u, float& v) {

    return GeometryUtils::rayTriangleIntersection(ray, tri, t, u, v);
}

// ============================================================================
// VOXEL GRID HELPERS
// ============================================================================

Vec3 ARTRayTracer::computeGridEntryPoint(const Ray& ray, float tMin) const {
    if (tMin > 0.0f) {
        // Ray origin is outside grid, use entry point with small offset
        return add(ray.origin, multiply(ray.direction, tMin + 1e-5f));
    }
    return ray.origin;
}

int ARTRayTracer::worldToVoxelIndex(float worldCoord, float gridMin, float voxelSize, int maxIndex) const {
    int index = static_cast<int>((worldCoord - gridMin) / voxelSize);
    return std::max(0, std::min(maxIndex, index));
}

// ============================================================================
// 3DDDA TRAVERSAL - INITIALIZATION (UNUSED - kept for reference)
// ============================================================================
// NOTE: These functions are from the original Fujimoto 3DDDA algorithm
// They are currently unused but kept for reference/future optimization

ARTRayTracer::VoxelTraversalState ARTRayTracer::initializeVoxelTraversal(
    const Ray& ray, float tMin) const {

    VoxelTraversalState state;

    // Calculate entry point into grid
    Vec3 startPoint = computeGridEntryPoint(ray, tMin);

    // Convert world position to voxel indices (O(1) lookup)
    state.ix = worldToVoxelIndex(startPoint.x, voxelGrid.minBound.x,
        voxelGrid.voxelSize.x, static_cast<int>(voxelGrid.nx) - 1);
    state.iy = worldToVoxelIndex(startPoint.y, voxelGrid.minBound.y,
        voxelGrid.voxelSize.y, static_cast<int>(voxelGrid.ny) - 1);
    state.iz = worldToVoxelIndex(startPoint.z, voxelGrid.minBound.z,
        voxelGrid.voxelSize.z, static_cast<int>(voxelGrid.nz) - 1);

    return state;
}

// ============================================================================
// 3DDDA TRAVERSAL - STEP DIRECTION COMPUTATION (UNUSED - kept for reference)
// ============================================================================

void ARTRayTracer::computeVoxelStepDirection(const Ray& ray, VoxelTraversalState& state) const {
    const float EPSILON = 1e-6f;

    // Determine step direction (Fujimoto's directional indices)
    state.stepX = (ray.direction.x > 0) ? 1 : -1;
    state.stepY = (ray.direction.y > 0) ? 1 : -1;
    state.stepZ = (ray.direction.z > 0) ? 1 : -1;

    // Calculate tDelta - parameter increment to cross one voxel
    state.tDeltaX = (std::abs(ray.direction.x) > EPSILON) ?
        std::abs(voxelGrid.voxelSize.x / ray.direction.x) : std::numeric_limits<float>::max();
    state.tDeltaY = (std::abs(ray.direction.y) > EPSILON) ?
        std::abs(voxelGrid.voxelSize.y / ray.direction.y) : std::numeric_limits<float>::max();
    state.tDeltaZ = (std::abs(ray.direction.z) > EPSILON) ?
        std::abs(voxelGrid.voxelSize.z / ray.direction.z) : std::numeric_limits<float>::max();
}

// ============================================================================
// 3DDDA TRAVERSAL - TMAX COMPUTATION (UNUSED - kept for reference)
// ============================================================================

float ARTRayTracer::computeAxisTMax(
    float rayOrigin, float rayDirection,
    float gridMin, float voxelSize, int voxelIndex) const {

    const float EPSILON = 1e-6f;

    if (std::abs(rayDirection) > EPSILON) {
        float boundaryOffset = (rayDirection > 0) ? (voxelIndex + 1) : voxelIndex;
        float boundary = gridMin + boundaryOffset * voxelSize;
        return (boundary - rayOrigin) / rayDirection;
    }
    else {
        return std::numeric_limits<float>::max();
    }
}

void ARTRayTracer::computeTMaxValues(const Ray& ray, VoxelTraversalState& state) const {
    // Calculate tMax - parameter value at next voxel boundary for each axis
    state.tMaxX = computeAxisTMax(ray.origin.x, ray.direction.x,
        voxelGrid.minBound.x, voxelGrid.voxelSize.x, state.ix);

    state.tMaxY = computeAxisTMax(ray.origin.y, ray.direction.y,
        voxelGrid.minBound.y, voxelGrid.voxelSize.y, state.iy);

    state.tMaxZ = computeAxisTMax(ray.origin.z, ray.direction.z,
        voxelGrid.minBound.z, voxelGrid.voxelSize.z, state.iz);
}

// ============================================================================
// 3DDDA TRAVERSAL - VOXEL ADVANCEMENT (UNUSED - kept for reference)
// ============================================================================

void ARTRayTracer::advanceToNextVoxel(VoxelTraversalState& state) const {
    // Choose the axis with the smallest tMax (closest boundary)
    if (state.tMaxX < state.tMaxY) {
        if (state.tMaxX < state.tMaxZ) {
            // Step in X direction
            state.ix += state.stepX;
            state.tMaxX += state.tDeltaX;
        }
        else {
            // Step in Z direction
            state.iz += state.stepZ;
            state.tMaxZ += state.tDeltaZ;
        }
    }
    else {
        if (state.tMaxY < state.tMaxZ) {
            // Step in Y direction
            state.iy += state.stepY;
            state.tMaxY += state.tDeltaY;
        }
        else {
            // Step in Z direction
            state.iz += state.stepZ;
            state.tMaxZ += state.tDeltaZ;
        }
    }
}

// ============================================================================
// 3DDDA TRAVERSAL - TRIANGLE TESTING
// ============================================================================

bool ARTRayTracer::testVoxelTriangles(const Ray& ray, int ix, int iy, int iz, RayHit& result) {
    voxelsTraversed++; // Statistics

    const Voxel& voxel = voxelGrid.voxels[ix][iy][iz];

    // If voxel is occupied, test triangles (spatial coherence)
    if (!voxel.occupied || voxel.triangle_count == 0) {
        return false;
    }

    for (unsigned int i = 0; i < voxel.triangle_count; ++i) {
        trianglesTests++; // Statistics

        // Get triangle index from the flat triangle_indices array
        unsigned int triIdx = voxelGrid.triangle_indices[voxel.triangle_start_idx + i];
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

    return result.hit;
}

// ============================================================================
// 3DDDA TRAVERSAL - MAIN ALGORITHM
// ============================================================================

/**
 * Simple Array-Based Voxel Traversal
 *
 * This implements a straightforward brute-force approach that traverses
 * all voxels in the grid using simple nested for-loops.
 *
 * Trade-off: Simple to understand but tests all voxels (not just along ray path).
 * Good for: Small grids, debugging, or when simplicity is preferred over speed.
 */
RayHit ARTRayTracer::traverse3DDDA(const Ray& ray) {
    RayHit result;
    result.hit = false;
    result.t = std::numeric_limits<float>::max();

    // Check if ray intersects the grid bounding box
    float tMin, tMax;
    if (!rayBoxIntersection(ray, voxelGrid.minBound, voxelGrid.maxBound, tMin, tMax)) {
        return result; // Ray misses the entire grid
    }

    // Simple nested loop approach - traverse all voxels like a normal 3D array
    for (int iz = 0; iz < static_cast<int>(voxelGrid.nz); ++iz) {
        for (int iy = 0; iy < static_cast<int>(voxelGrid.ny); ++iy) {
            for (int ix = 0; ix < static_cast<int>(voxelGrid.nx); ++ix) {

                // Test triangles in current voxel
                testVoxelTriangles(ray, ix, iy, iz, result);
            }
        }
    }

    return result;
}
