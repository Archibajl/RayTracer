#include "ARTRayTracer.h"

using namespace cg_datastructures;
using namespace vector_math;

ARTRayTracer::ARTRayTracer(const VoxelGrid& grid)
    : voxelGrid(grid) {
    LOG_INFO("[{}] Initialized with uniform grid: {}x{}x{}",
        getMethodName(), grid.nx, grid.ny, grid.nz);
    LOG_INFO("[{}] Based on Fujimoto et al. 1986 ARTS algorithm", getMethodName());
}

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

Vec3 ARTRayTracer::computeShading(const RayHit& hit, float v) const {
    if (hit.hit) {
        // Simple solid color for hits
        return { 0.5f, 0.5f, 0.5f }; // White
    }
    else {
        // Background gradient
        return { 0.6f + 0.4f * v, 0.6f + 0.4f * v, 0.9f }; // Gray-blue gradient
    }
}

std::vector<Vec3> ARTRayTracer::render(const Camera& camera, int width, int height) {
    std::vector<Vec3> frameBuffer(width * height);

    LOG_INFO("[{}] Rendering {}x{} image...", getMethodName(), width, height);

    // Reset statistics
    voxelsTraversed = 0;
    trianglesTests = 0;
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

            // Generate and trace ray
            Ray ray = generateRay(camera, u, v);
            RayHit hit = traceRay(ray);

            if (hit.hit) {
                hitCount++;
            }

            // Compute shading
            Vec3 color = computeShading(hit, v);
            frameBuffer[y * width + x] = color;
        }
    }

    LOG_INFO("[{}] Rendering complete!", getMethodName());
    LOG_INFO("[{}] Ray hits: {} / {} ({:.1f}%)",
        getMethodName(), hitCount, totalRays, 100.0f * hitCount / totalRays);
    LOG_INFO("[{}] Voxels traversed: {}, Avg per ray: {:.1f}",
        getMethodName(), voxelsTraversed, (float)voxelsTraversed / totalRays);
    LOG_INFO("[{}] Triangle tests: {}, Avg per ray: {:.1f}",
        getMethodName(), trianglesTests, (float)trianglesTests / totalRays);

    return frameBuffer;
}

RayHit ARTRayTracer::traceRay(const Ray& ray) {
    return traverse3DDDA(ray);
}

bool ARTRayTracer::rayBoxIntersection(
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

bool ARTRayTracer::rayTriangleIntersection(
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

size_t ARTRayTracer::getVoxelIndex(int ix, int iy, int iz) const {
    return ix + voxelGrid.nx * (iy + voxelGrid.ny * iz);
}

ARTRayTracer::VoxelTraversalState ARTRayTracer::initializeVoxelTraversal(
    const Ray& ray, float tMin) const {

    VoxelTraversalState state;

    // Calculate entry point into grid
    Vec3 startPoint = ray.origin;
    if (tMin > 0.0f) {
        // Ray origin is outside grid, use entry point
        startPoint = add(ray.origin, multiply(ray.direction, tMin + 1e-5f));
    }

    // Convert world position to voxel indices (O(1) lookup)
    state.ix = static_cast<int>((startPoint.x - voxelGrid.minBound.x) / voxelGrid.voxelSize.x);
    state.iy = static_cast<int>((startPoint.y - voxelGrid.minBound.y) / voxelGrid.voxelSize.y);
    state.iz = static_cast<int>((startPoint.z - voxelGrid.minBound.z) / voxelGrid.voxelSize.z);

    // Clamp to grid bounds
    state.ix = std::max(0, std::min(static_cast<int>(voxelGrid.nx) - 1, state.ix));
    state.iy = std::max(0, std::min(static_cast<int>(voxelGrid.ny) - 1, state.iy));
    state.iz = std::max(0, std::min(static_cast<int>(voxelGrid.nz) - 1, state.iz));

    return state;
}

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

void ARTRayTracer::computeTMaxValues(const Ray& ray, VoxelTraversalState& state) const {
    const float EPSILON = 1e-6f;

    // Calculate tMax - parameter value at next voxel boundary
    if (std::abs(ray.direction.x) > EPSILON) {
        if (ray.direction.x > 0) {
            state.tMaxX = ((voxelGrid.minBound.x + (state.ix + 1) * voxelGrid.voxelSize.x) - ray.origin.x) / ray.direction.x;
        }
        else {
            state.tMaxX = ((voxelGrid.minBound.x + state.ix * voxelGrid.voxelSize.x) - ray.origin.x) / ray.direction.x;
        }
    }
    else {
        state.tMaxX = std::numeric_limits<float>::max();
    }

    if (std::abs(ray.direction.y) > EPSILON) {
        if (ray.direction.y > 0) {
            state.tMaxY = ((voxelGrid.minBound.y + (state.iy + 1) * voxelGrid.voxelSize.y) - ray.origin.y) / ray.direction.y;
        }
        else {
            state.tMaxY = ((voxelGrid.minBound.y + state.iy * voxelGrid.voxelSize.y) - ray.origin.y) / ray.direction.y;
        }
    }
    else {
        state.tMaxY = std::numeric_limits<float>::max();
    }

    if (std::abs(ray.direction.z) > EPSILON) {
        if (ray.direction.z > 0) {
            state.tMaxZ = ((voxelGrid.minBound.z + (state.iz + 1) * voxelGrid.voxelSize.z) - ray.origin.z) / ray.direction.z;
        }
        else {
            state.tMaxZ = ((voxelGrid.minBound.z + state.iz * voxelGrid.voxelSize.z) - ray.origin.z) / ray.direction.z;
        }
    }
    else {
        state.tMaxZ = std::numeric_limits<float>::max();
    }
}

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

bool ARTRayTracer::testVoxelTriangles(const Ray& ray, int ix, int iy, int iz, RayHit& result) {
    voxelsTraversed++; // Statistics

    size_t voxelIdx = getVoxelIndex(ix, iy, iz);
    const Voxel& voxel = voxelGrid.voxels[voxelIdx];

    // If voxel is occupied, test triangles (spatial coherence)
    if (!voxel.occupied || voxel.triangle_count == 0) {
        return false;
    }

    for (unsigned int i = 0; i < voxel.triangle_count; ++i) {
        trianglesTests++; // Statistics

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
                result.voxelIdx = static_cast<unsigned int>(voxelIdx);
            }
        }
    }

    return result.hit;
}

/**
 * ARTS 3DDDA Algorithm
 *
 * This implements Fujimoto's classic 3DDDA (3D Digital Differential Analyzer)
 * algorithm for efficient voxel grid traversal.
 *
 * Key principle: Incrementally step through voxels along the ray path,
 * testing only triangles in occupied voxels. The algorithm maintains
 * tMax values for each axis to determine which voxel boundary is crossed next.
 */
RayHit ARTRayTracer::traverse3DDDA(const Ray& ray) {
    RayHit result;
    result.hit = false;
    result.t = std::numeric_limits<float>::max();

    // Step 1: Check if ray intersects the grid bounding box
    float tMin, tMax;
    if (!rayBoxIntersection(ray, voxelGrid.minBound, voxelGrid.maxBound, tMin, tMax)) {
        return result; // Ray misses the entire grid
    }

    // Step 2: Initialize voxel traversal state
    VoxelTraversalState state = initializeVoxelTraversal(ray, tMin);

    // Step 3: Compute step directions and delta values
    computeVoxelStepDirection(ray, state);

    // Step 4: Compute initial tMax values
    computeTMaxValues(ray, state);

    // Step 5: 3DDDA main loop - traverse voxels until we exit the grid or find a hit
    int maxSteps = static_cast<int>(voxelGrid.nx + voxelGrid.ny + voxelGrid.nz);
    int steps = 0;

    while (state.ix >= 0 && state.ix < static_cast<int>(voxelGrid.nx) &&
           state.iy >= 0 && state.iy < static_cast<int>(voxelGrid.ny) &&
           state.iz >= 0 && state.iz < static_cast<int>(voxelGrid.nz) &&
           steps < maxSteps) {

        steps++;

        // Test triangles in current voxel
        if (testVoxelTriangles(ray, state.ix, state.iy, state.iz, result)) {
            // ARTS optimization: Return first hit found (front-to-back traversal)
            return result;
        }

        // Advance to next voxel
        advanceToNextVoxel(state);
    }

    return result;
}
