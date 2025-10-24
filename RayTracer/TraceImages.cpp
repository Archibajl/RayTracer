#include "TraceImages.h"
#include "Logger.h"
#include <filesystem>
#include <chrono>
#include "SceneCreator.h"
#include "ARTRayTracer.h"
#include "OctreeRayTracer.h"
#include "ImageSaver.h"
#include "STLReader.h"
using namespace std;
using namespace cg_datastructures;

void TraceImages::TraceImage(std::string gridFileLocation, std::string outputFileName, RayTracingMethod method) {
	LOG_INFO("========================================");
	LOG_INFO("Starting image generation");
	LOG_INFO("Input file: {}", gridFileLocation);
	LOG_INFO("Output file: {}", outputFileName);

	// Log the ray tracing method
	std::string methodName;
	switch (method) {
		case RayTracingMethod::VOXEL_DDA: methodName = "Voxel Grid DDA"; break;
		case RayTracingMethod::ART: methodName = "ARTS (3DDDA)"; break;
		case RayTracingMethod::OCTREE: methodName = "Octree"; break;
		default: methodName = "Unknown"; break;
	}
	LOG_INFO("Ray tracing method: {}", methodName);
	LOG_INFO("========================================");

	// Start total timer
	auto totalStart = std::chrono::high_resolution_clock::now();

	try {
		// Build or load voxel grid
		auto voxelStart = std::chrono::high_resolution_clock::now();
		VoxelGrid voxelGrid = loadOrGenerateVoxelGrid(gridFileLocation, 50, 50, 50);
		auto voxelEnd = std::chrono::high_resolution_clock::now();
		auto voxelDuration = std::chrono::duration_cast<std::chrono::milliseconds>(voxelEnd - voxelStart);
		LOG_INFO("Voxel grid load/generation time: {:.3f} seconds", voxelDuration.count() / 1000.0);

		// Generate images using specified method
		auto raytraceStart = std::chrono::high_resolution_clock::now();
		genateImageFromGrid(voxelGrid, outputFileName, method);
		auto raytraceEnd = std::chrono::high_resolution_clock::now();
		auto raytraceDuration = std::chrono::duration_cast<std::chrono::milliseconds>(raytraceEnd - raytraceStart);
		LOG_INFO("Ray tracing time: {:.3f} seconds", raytraceDuration.count() / 1000.0);

		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		LOG_INFO("========================================");
		LOG_INFO("Total time: {:.3f} seconds", totalDuration.count() / 1000.0);
		LOG_INFO("Successfully generated image: {}", outputFileName);
	}
	catch (const std::exception& e) {
		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		LOG_ERROR("========================================");
		LOG_ERROR("FAILED to generate image for: {}", gridFileLocation);
		LOG_ERROR("Error: {}", e.what());
		LOG_ERROR("Time before failure: {:.3f} seconds", totalDuration.count() / 1000.0);
		LOG_ERROR("========================================");
		// Don't re-throw, just continue to next image
	}
}

void TraceImages::genateImageFromGrid(VoxelGrid voxelGrid, std::string filename, RayTracingMethod method) {
	// Create ray tracer using factory method
	std::unique_ptr<IRayTracer> raytracer = createRayTracer(voxelGrid, method);

	// Calculate mesh center and size for camera positioning
	Vec3 center = {
		(voxelGrid.minBound.x + voxelGrid.maxBound.x) * 0.5f,
		(voxelGrid.minBound.y + voxelGrid.maxBound.y) * 0.5f,
		(voxelGrid.minBound.z + voxelGrid.maxBound.z) * 0.5f
	};

	Vec3 size = {
		voxelGrid.maxBound.x - voxelGrid.minBound.x,
		voxelGrid.maxBound.y - voxelGrid.minBound.y,
		voxelGrid.maxBound.z - voxelGrid.minBound.z
	};

	float maxSize = std::max(std::max(size.x, size.y), size.z);
	float cameraDistance = maxSize * 2.5f; // Position camera 2.5x the max dimension away

	LOG_INFO("Mesh center: ({}, {}, {})", center.x, center.y, center.z);
	LOG_INFO("Mesh size: ({}, {}, {})", size.x, size.y, size.z);
	LOG_INFO("Camera distance: {}", cameraDistance);

	// Setup camera positioned to view the entire mesh
	Camera camera(
		{ center.x, center.y, center.z - cameraDistance },  // Position (back from center)
		{ center.x, center.y, center.z },                    // Look at center
		{ 0.0f, 1.0f, 0.0f },                                // Up vector
		60.0f,                                                // FOV
		800.0f / 600.0f                                       // Aspect ratio
	);

	// Render image
	int width = 800, height = 600;
	LOG_INFO("Rendering {}x{} image...", width, height);

	auto renderStart = std::chrono::high_resolution_clock::now();
	auto pixels = raytracer->render(camera, width, height);
	auto renderEnd = std::chrono::high_resolution_clock::now();
	auto renderDuration = std::chrono::duration_cast<std::chrono::milliseconds>(renderEnd - renderStart);

	LOG_INFO("Rendering complete!");
	LOG_INFO("Image rendered with {} pixels in {:.3f} seconds", pixels.size(), renderDuration.count() / 1000.0);
	LOG_INFO("Render performance: {:.2f} rays/sec", (width * height) / (renderDuration.count() / 1000.0));

	// Output image
	auto saveStart = std::chrono::high_resolution_clock::now();
	SaveImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\GeneratedFiles\\" + filename, pixels, width, height);
	auto saveEnd = std::chrono::high_resolution_clock::now();
	auto saveDuration = std::chrono::duration_cast<std::chrono::milliseconds>(saveEnd - saveStart);
	LOG_INFO("Image save time: {:.3f} seconds", saveDuration.count() / 1000.0);
}

VoxelGrid TraceImages::generateVoxelGridFromFile(const std::string filepath, int nx, int ny, int nz) {
	// Load mesh from STL file using stl_reader
	LOG_INFO("Loading STL file: {}", filepath);
	stl_reader::StlMesh<float, unsigned int> stlMesh(filepath);
	LOG_INFO("Loaded mesh with {} triangles", stlMesh.num_tris());

	// Convert stl_reader::StlMesh to our format for voxelization
	// Step 1: Compute mesh AABB (bounding box)
	LOG_INFO("Computing mesh bounding box...");
	Vec3 minBound = {
		std::numeric_limits<float>::max(),
		std::numeric_limits<float>::max(),
		std::numeric_limits<float>::max()
	};
	Vec3 maxBound = {
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest()
	};

	for (size_t i = 0; i < stlMesh.num_vrts(); ++i) {
		const float* coords = stlMesh.vrt_coords(i);
		minBound.x = std::min(minBound.x, coords[0]);
		minBound.y = std::min(minBound.y, coords[1]);
		minBound.z = std::min(minBound.z, coords[2]);
		maxBound.x = std::max(maxBound.x, coords[0]);
		maxBound.y = std::max(maxBound.y, coords[1]);
		maxBound.z = std::max(maxBound.z, coords[2]);
	}

	LOG_INFO("Bounding box: Min({}, {}, {}) Max({}, {}, {})",
		minBound.x, minBound.y, minBound.z,
		maxBound.x, maxBound.y, maxBound.z);

	// Step 2: Compute voxel grid parameters with equal-sized voxels
	Vec3 gridSize = {
		maxBound.x - minBound.x,
		maxBound.y - minBound.y,
		maxBound.z - minBound.z
	};

	float maxDimension = std::max(std::max(gridSize.x, gridSize.y), gridSize.z);
	float maxResolution = static_cast<float>(std::max(std::max(nx, ny), nz));
	float uniformVoxelSize = maxDimension / maxResolution;

	Vec3 voxelSize = {
		uniformVoxelSize,
		uniformVoxelSize,
		uniformVoxelSize
	};

	LOG_INFO("Uniform voxel size: {}", uniformVoxelSize);
	LOG_INFO("Grid dimensions: ({}, {}, {})", gridSize.x, gridSize.y, gridSize.z);

	// Step 3: Build triangle list
	LOG_INFO("Building triangle list from {} triangles...", stlMesh.num_tris());
	std::vector<Triangle> triangles;
	triangles.reserve(stlMesh.num_tris());

	for (size_t t = 0; t < stlMesh.num_tris(); ++t) {
		const unsigned int* indices = stlMesh.tri_corner_inds(t);

		const float* v0_coords = stlMesh.vrt_coords(indices[0]);
		const float* v1_coords = stlMesh.vrt_coords(indices[1]);
		const float* v2_coords = stlMesh.vrt_coords(indices[2]);

		Vec3 v0 = { v0_coords[0], v0_coords[1], v0_coords[2] };
		Vec3 v1 = { v1_coords[0], v1_coords[1], v1_coords[2] };
		Vec3 v2 = { v2_coords[0], v2_coords[1], v2_coords[2] };

		const float* nrm = stlMesh.tri_normal(t);
		Vec3 normal = { nrm[0], nrm[1], nrm[2] };

		triangles.emplace_back(Triangle{ v0, v1, v2, normal });
	}

	// Step 4: Build voxel grid directly (similar to BuildVoxelGridFromStlMesh)
	LOG_INFO("Allocating voxel grid: {}x{}x{} = {} voxels", nx, ny, nz, nx * ny * nz);
	std::vector<VoxelHost> voxels_host(nx * ny * nz);

	auto voxelIndex = [nx, ny](size_t ix, size_t iy, size_t iz) -> size_t {
		return ix + nx * (iy + ny * iz);
	};

	// Voxelize triangles
	LOG_INFO("Voxelizing {} triangles...", triangles.size());
	for (size_t t = 0; t < triangles.size(); ++t) {
		const Triangle& tri = triangles[t];

		// Triangle AABB
		Vec3 triMin = {
			std::min(std::min(tri.v0.x, tri.v1.x), tri.v2.x),
			std::min(std::min(tri.v0.y, tri.v1.y), tri.v2.y),
			std::min(std::min(tri.v0.z, tri.v1.z), tri.v2.z)
		};
		Vec3 triMax = {
			std::max(std::max(tri.v0.x, tri.v1.x), tri.v2.x),
			std::max(std::max(tri.v0.y, tri.v1.y), tri.v2.y),
			std::max(std::max(tri.v0.z, tri.v1.z), tri.v2.z)
		};

		// Determine which voxels this triangle overlaps
		size_t ix0 = std::clamp(static_cast<size_t>((triMin.x - minBound.x) / voxelSize.x), size_t(0), size_t(nx - 1));
		size_t iy0 = std::clamp(static_cast<size_t>((triMin.y - minBound.y) / voxelSize.y), size_t(0), size_t(ny - 1));
		size_t iz0 = std::clamp(static_cast<size_t>((triMin.z - minBound.z) / voxelSize.z), size_t(0), size_t(nz - 1));
		size_t ix1 = std::clamp(static_cast<size_t>((triMax.x - minBound.x) / voxelSize.x), size_t(0), size_t(nx - 1));
		size_t iy1 = std::clamp(static_cast<size_t>((triMax.y - minBound.y) / voxelSize.y), size_t(0), size_t(ny - 1));
		size_t iz1 = std::clamp(static_cast<size_t>((triMax.z - minBound.z) / voxelSize.z), size_t(0), size_t(nz - 1));

		// Add triangle to overlapping voxels
		for (size_t iz = iz0; iz <= iz1; ++iz) {
			for (size_t iy = iy0; iy <= iy1; ++iy) {
				for (size_t ix = ix0; ix <= ix1; ++ix) {
					size_t voxIdx = voxelIndex(ix, iy, iz);
					voxels_host[voxIdx].addTriangleIndex(static_cast<unsigned int>(t));
					voxels_host[voxIdx].occupied = true;
				}
			}
		}
	}

	// Step 5: Convert to flat CUDA-compatible format
	LOG_INFO("Converting to CUDA-compatible format...");
	std::vector<unsigned int> triangle_indices_flat;
	std::vector<Voxel> voxels_final(nx * ny * nz);

	for (size_t i = 0; i < voxels_host.size(); ++i) {
		VoxelHost& vh = voxels_host[i];
		Voxel& vf = voxels_final[i];

		vf.occupied = vh.occupied;
		vf.density = vh.density;
		vf.color = vh.color;

		if (vh.triangle_count > 0) {
			vf.triangle_start_idx = static_cast<unsigned int>(triangle_indices_flat.size());
			vf.triangle_count = static_cast<unsigned int>(vh.triangle_count);

			for (size_t j = 0; j < vh.triangle_count; ++j) {
				triangle_indices_flat.push_back(vh.triangle_indices[j]);
			}
		} else {
			vf.triangle_start_idx = 0;
			vf.triangle_count = 0;
		}
	}

	LOG_INFO("Voxel grid created: {} voxels, {} triangle references",
		voxels_final.size(), triangle_indices_flat.size());

	return VoxelGrid(
		std::move(voxels_final),
		std::move(triangle_indices_flat),
		nx, ny, nz,
		minBound, maxBound, voxelSize,
		std::move(triangles)
	);
}

void TraceImages::SaveImage(const std::string& filename,
	const std::vector<cg_datastructures::Vec3>& pixels,
	int width, int height) {
		// Save to file - create directory if it doesn't exist
		std::string outputPath = filename;
		std::filesystem::path filePath(outputPath);
		std::filesystem::path dirPath = filePath.parent_path();

		try {
			// Create directory if it doesn't exist
			if (!dirPath.empty() && !std::filesystem::exists(dirPath)) {
				std::filesystem::create_directories(dirPath);
				LOG_INFO("Created output directory: {}", dirPath.string());
			}

			if (saveToPPM(outputPath, pixels, width, height)) {
				LOG_INFO("Image saved to: {}", outputPath);
			}
			else {
				LOG_ERROR("Failed to save image to: {}", outputPath);
			}
		}
		catch (const std::exception& e) {
			LOG_ERROR("Error saving image: {}", e.what());

			// Fallback: save to current directory
			std::string fallbackPath = "output.ppm";
			LOG_INFO("Attempting to save to current directory: {}", fallbackPath);
			if (saveToPPM(fallbackPath, pixels, width, height)) {
				LOG_INFO("Image saved to: {}", std::filesystem::absolute(fallbackPath).string());
			}
			else {
				LOG_ERROR("Failed to save to fallback location");
			}
		}
	}

VoxelGrid TraceImages::loadOrGenerateVoxelGrid(const std::string& filepath, int nx, int ny, int nz) {
	// Check if filepath is a .voxgrid file (pre-saved voxel grid)
	std::filesystem::path path(filepath);
	std::string extension = path.extension().string();

	// Convert extension to lowercase for case-insensitive comparison
	std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

	if (extension == ".voxgrid") {
		// Load pre-saved voxel grid
		LOG_INFO("Loading pre-saved voxel grid from: {}", filepath);
		try {
			VoxelGrid grid = VoxelGrid::loadFromFile(filepath);
			LOG_INFO("Successfully loaded voxel grid from file");
			return grid;
		}
		catch (const std::exception& e) {
			LOG_ERROR("Failed to load voxel grid: {}", e.what());
			throw;
		}
	}
	else if (extension == ".stl") {
		// Generate voxel grid from STL file
		LOG_INFO("Generating voxel grid from STL file: {}", filepath);
		VoxelGrid grid = generateVoxelGridFromFile(filepath, nx, ny, nz);

		// Optionally save the generated voxel grid for faster loading next time
		std::string voxgridPath = path.replace_extension(".voxgrid").string();
		LOG_INFO("Saving voxel grid to: {}", voxgridPath);
		if (grid.saveToFile(voxgridPath)) {
			LOG_INFO("Voxel grid cached successfully");
		}
		else {
			LOG_WARN("Failed to cache voxel grid");
		}

		return grid;
	}
	else {
		LOG_ERROR("Unsupported file format: {}. Expected .stl or .voxgrid", extension);
		throw std::runtime_error("Unsupported file format: " + extension);
	}
}

// Factory method to create appropriate ray tracer
std::unique_ptr<IRayTracer> TraceImages::createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method) {
	switch (method) {
		case RayTracingMethod::VOXEL_DDA:
			// VOXEL_DDA uses ARTS (3DDDA) which is a voxel-based DDA algorithm
			return std::make_unique<ARTRayTracer>(voxelGrid);

		case RayTracingMethod::ART:
			return std::make_unique<ARTRayTracer>(voxelGrid);

		case RayTracingMethod::OCTREE:
			return std::make_unique<OctreeRayTracer>(voxelGrid);

		// Future methods can be added here:
		// case RayTracingMethod::BVH:
		//     return std::make_unique<BVHRayTracer>(voxelGrid);

		default:
			LOG_WARN("Unknown ray tracing method, defaulting to ARTS (3DDDA)");
			return std::make_unique<ARTRayTracer>(voxelGrid);
	}
}