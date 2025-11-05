#include "OctreeRayTracer.h"
#include "GeometryUtils.h"

using namespace cg_datastructures;
using namespace vector_math;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

OctreeRayTracer::OctreeRayTracer(const VoxelGrid& grid, int maxDepth, int maxTrianglesPerNode)
    : voxelGrid(grid) {

    LOG_INFO("[{}] Initializing voxel grid ray tracer...", getMethodName());
    LOG_INFO("[{}] Grid dimensions: {}x{}x{}", getMethodName(),
        voxelGrid.nx, voxelGrid.ny, voxelGrid.nz);
    LOG_INFO("[{}] Total voxels: {}", getMethodName(),
        voxelGrid.nx * voxelGrid.ny * voxelGrid.nz);
    LOG_INFO("[{}] Using 3D-DDA traversal algorithm", getMethodName());
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
    voxelsTraversed = 0;
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
    LOG_INFO("[{}] Voxels traversed: {}, Avg per ray: {:.1f}",
        getMethodName(), voxelsTraversed, (float)voxelsTraversed / totalRays);
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
    return traverseVoxelGrid(ray);
}

// ============================================================================
// GEOMETRY INTERSECTION - WRAPPER FUNCTIONS
// ============================================================================

bool OctreeRayTracer::rayTriangleIntersection(
    const Ray& ray,
    const Triangle& tri,
    float& t, float& u, float& v) {

    return GeometryUtils::rayTriangleIntersection(ray, tri, t, u, v);
}

// ============================================================================
// VOXEL GRID HELPERS
// ============================================================================

int OctreeRayTracer::worldToVoxelIndex(float worldCoord, float gridMin, float voxelSize, int maxIndex) const {
    int index = static_cast<int>((worldCoord - gridMin) / voxelSize);
    return std::max(0, std::min(index, maxIndex));
}

Vec3 OctreeRayTracer::voxelIndexToWorld(int ix, int iy, int iz) const {
    return {
        voxelGrid.minBound.x + (ix + 0.5f) * voxelGrid.voxelSize.x,
        voxelGrid.minBound.y + (iy + 0.5f) * voxelGrid.voxelSize.y,
        voxelGrid.minBound.z + (iz + 0.5f) * voxelGrid.voxelSize.z
    };
}

bool OctreeRayTracer::isVoxelInBounds(int ix, int iy, int iz) const {
    return ix >= 0 && ix < voxelGrid.nx &&
           iy >= 0 && iy < voxelGrid.ny &&
           iz >= 0 && iz < voxelGrid.nz;
}

// ============================================================================
// VOXEL GRID TRAVERSAL - INITIALIZATION
// ============================================================================

OctreeRayTracer::VoxelTraversalState OctreeRayTracer::initializeTraversal(const Ray& ray) const {
    OctreeRayTracer::VoxelTraversalState state;
    const float EPSILON = 1e-6f;

    // Find ray entry point into grid
    Vec3 startPoint = ray.origin;

    // Convert world position to voxel indices
    state.ix = worldToVoxelIndex(startPoint.x, voxelGrid.minBound.x,
        voxelGrid.voxelSize.x, voxelGrid.nx - 1);
    state.iy = worldToVoxelIndex(startPoint.y, voxelGrid.minBound.y,
        voxelGrid.voxelSize.y, voxelGrid.ny - 1);
    state.iz = worldToVoxelIndex(startPoint.z, voxelGrid.minBound.z,
        voxelGrid.voxelSize.z, voxelGrid.nz - 1);

    // Determine step direction
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

    // Calculate tMax - parameter value at next voxel boundary for each axis
    auto computeTMax = [&](float rayOrigin, float rayDir, float gridMin, float voxelSize, int voxelIdx) -> float {
        if (std::abs(rayDir) > EPSILON) {
            float boundaryOffset = (rayDir > 0) ? (voxelIdx + 1) : voxelIdx;
            float boundary = gridMin + boundaryOffset * voxelSize;
            return (boundary - rayOrigin) / rayDir;
        }
        return std::numeric_limits<float>::max();
    };

    state.tMaxX = computeTMax(ray.origin.x, ray.direction.x, voxelGrid.minBound.x, voxelGrid.voxelSize.x, state.ix);
    state.tMaxY = computeTMax(ray.origin.y, ray.direction.y, voxelGrid.minBound.y, voxelGrid.voxelSize.y, state.iy);
    state.tMaxZ = computeTMax(ray.origin.z, ray.direction.z, voxelGrid.minBound.z, voxelGrid.voxelSize.z, state.iz);

    // Initialize current position at ray origin
    state.tCurrent = 0.0f;
    state.currentPosition = ray.origin;

    return state;
}

// ============================================================================
// VOXEL GRID TRAVERSAL - VOXEL ADVANCEMENT
// ============================================================================

void OctreeRayTracer::advanceToNextVoxel(OctreeRayTracer::VoxelTraversalState& state, const Ray& ray) {
    // Choose the axis with the smallest tMax (closest boundary) and step to next voxel
    // Also update the current position to the voxel exit point
    if (state.tMaxX < state.tMaxY) {
        if (state.tMaxX < state.tMaxZ) {
            // Exit through X face - update position to exit point
            state.tCurrent = state.tMaxX;
            state.currentPosition = {
                ray.origin.x + state.tCurrent * ray.direction.x,
                ray.origin.y + state.tCurrent * ray.direction.y,
                ray.origin.z + state.tCurrent * ray.direction.z
            };

            // Step in X direction
            state.ix += state.stepX;
            state.tMaxX += state.tDeltaX;
        }
        else {
            // Exit through Z face - update position to exit point
            state.tCurrent = state.tMaxZ;
            state.currentPosition = {
                ray.origin.x + state.tCurrent * ray.direction.x,
                ray.origin.y + state.tCurrent * ray.direction.y,
                ray.origin.z + state.tCurrent * ray.direction.z
            };

            // Step in Z direction
            state.iz += state.stepZ;
            state.tMaxZ += state.tDeltaZ;
        }
    }
    else {
        if (state.tMaxY < state.tMaxZ) {
            // Exit through Y face - update position to exit point
            state.tCurrent = state.tMaxY;
            state.currentPosition = {
                ray.origin.x + state.tCurrent * ray.direction.x,
                ray.origin.y + state.tCurrent * ray.direction.y,
                ray.origin.z + state.tCurrent * ray.direction.z
            };

            // Step in Y direction
            state.iy += state.stepY;
            state.tMaxY += state.tDeltaY;
        }
        else {
            // Exit through Z face - update position to exit point
            state.tCurrent = state.tMaxZ;
            state.currentPosition = {
                ray.origin.x + state.tCurrent * ray.direction.x,
                ray.origin.y + state.tCurrent * ray.direction.y,
                ray.origin.z + state.tCurrent * ray.direction.z
            };

            // Step in Z direction
            state.iz += state.stepZ;
            state.tMaxZ += state.tDeltaZ;
        }
    }
}


// ============================================================================
// VOXEL GRID TRAVERSAL - VOXEL TESTING
// ============================================================================

bool OctreeRayTracer::testVoxelTriangles(const Ray& ray, int ix, int iy, int iz, RayHit& result) {
    voxelsTraversed++; // Statistics

    const cg_datastructures::Voxel& voxel = voxelGrid.voxels[ix][iy][iz];

    // Simple occupancy check
    if (voxel.occupied) {
        // Calculate voxel center for hit point
        Vec3 voxelCenter = voxelIndexToWorld(ix, iy, iz);

        // Calculate approximate distance along ray to voxel center
        Vec3 toVoxel = subtract(voxelCenter, ray.origin);
        float t = dot(toVoxel, ray.direction);

        // Only register hit if it's the closest so far and in front of ray
        if (t > 0.0f && t < result.t) {
            result.hit = true;
            result.t = t;
            result.point = voxelCenter;
            result.normal = { 0.0f, 0.0f, 1.0f }; // Default normal
            result.triangleIdx = 0; // No specific triangle
        }
        return true;
    }

    return false;
}

// ============================================================================
// VOXEL GRID TRAVERSAL - MAIN FUNCTION
// ============================================================================

/**
 * Traverse voxel grid using 3D-DDA algorithm
 *
 * Based on Amanatides & Woo's "A Fast Voxel Traversal Algorithm"
 *
 * This algorithm steps through voxels along the ray path, testing only
 * the triangles in occupied voxels for intersection.
 */
RayHit OctreeRayTracer::traverseVoxelGrid(const Ray& ray) {
    RayHit result;
    result.hit = false;
    result.t = std::numeric_limits<float>::max();

    // Initialize traversal state
    OctreeRayTracer::VoxelTraversalState state = initializeTraversal(ray);

    // Traverse voxels along the ray
    const int MAX_STEPS = voxelGrid.nx + voxelGrid.ny + voxelGrid.nz;
    for (int step = 0; step < MAX_STEPS; ++step) {
        // Check if current voxel is in bounds
        if (!isVoxelInBounds(state.ix, state.iy, state.iz)) {
            break; // Ray has left the grid
        }

        // Test triangles in current voxel
        if (testVoxelTriangles(ray, state.ix, state.iy, state.iz, result)) {
            // Found a hit, can return early
            break;
        }

        // Advance to next voxel and update position to exit point
        advanceToNextVoxel(state, ray);
    }

    return result;
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


