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

	// Use the new direct conversion function from SceneCreator
	return BuildVoxelGridFromStlReaderMesh(stlMesh, nx, ny, nz);
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
		// Generate voxel grid directly from STL file
		LOG_INFO("Generating voxel grid from STL file: {}", filepath);
		return generateVoxelGridFromFile(filepath, nx, ny, nz);
	}
	else {
		LOG_ERROR("Unsupported file format: {}. Expected .stl or .voxgrid", extension);
		throw std::runtime_error("Unsupported file format: " + extension);
	}
}

// ============================================================================
// VOXELGRID SERIALIZATION
// ============================================================================

bool VoxelGrid::saveToFile(const std::string& filename) const {
	LOG_INFO("Saving voxel grid to: {}", filename);

	std::ofstream file(filename, std::ios::binary);
	if (!file) {
		LOG_ERROR("Failed to open file for writing: {}", filename);
		return false;
	}

	// Write dimensions
	file.write(reinterpret_cast<const char*>(&nx), sizeof(nx));
	file.write(reinterpret_cast<const char*>(&ny), sizeof(ny));
	file.write(reinterpret_cast<const char*>(&nz), sizeof(nz));

	// Write bounds
	file.write(reinterpret_cast<const char*>(&minBound), sizeof(minBound));
	file.write(reinterpret_cast<const char*>(&maxBound), sizeof(maxBound));
	file.write(reinterpret_cast<const char*>(&voxelSize), sizeof(voxelSize));

	// Write voxels
	size_t voxel_count = voxels.size();
	file.write(reinterpret_cast<const char*>(&voxel_count), sizeof(voxel_count));
	file.write(reinterpret_cast<const char*>(voxels.data()), voxel_count * sizeof(cg_datastructures::Voxel));

	// Write triangle indices
	size_t tri_index_count = triangle_indices.size();
	file.write(reinterpret_cast<const char*>(&tri_index_count), sizeof(tri_index_count));
	file.write(reinterpret_cast<const char*>(triangle_indices.data()), tri_index_count * sizeof(unsigned int));

	// Write triangles
	size_t triangle_count = triangles.size();
	file.write(reinterpret_cast<const char*>(&triangle_count), sizeof(triangle_count));
	file.write(reinterpret_cast<const char*>(triangles.data()), triangle_count * sizeof(cg_datastructures::Triangle));

	file.close();
	LOG_INFO("Voxel grid saved successfully");
	return true;
}

VoxelGrid VoxelGrid::loadFromFile(const std::string& filename) {
	LOG_INFO("Loading voxel grid from: {}", filename);

	std::ifstream file(filename, std::ios::binary);
	if (!file) {
		LOG_ERROR("Failed to open file for reading: {}", filename);
		throw std::runtime_error("Failed to open voxel grid file: " + filename);
	}

	// Read dimensions
	size_t nx, ny, nz;
	file.read(reinterpret_cast<char*>(&nx), sizeof(nx));
	file.read(reinterpret_cast<char*>(&ny), sizeof(ny));
	file.read(reinterpret_cast<char*>(&nz), sizeof(nz));

	// Read bounds
	cg_datastructures::Vec3 minBound, maxBound, voxelSize;
	file.read(reinterpret_cast<char*>(&minBound), sizeof(minBound));
	file.read(reinterpret_cast<char*>(&maxBound), sizeof(maxBound));
	file.read(reinterpret_cast<char*>(&voxelSize), sizeof(voxelSize));

	// Read voxels
	size_t voxel_count;
	file.read(reinterpret_cast<char*>(&voxel_count), sizeof(voxel_count));
	std::vector<cg_datastructures::Voxel> voxels(voxel_count);
	file.read(reinterpret_cast<char*>(voxels.data()), voxel_count * sizeof(cg_datastructures::Voxel));

	// Read triangle indices
	size_t tri_index_count;
	file.read(reinterpret_cast<char*>(&tri_index_count), sizeof(tri_index_count));
	std::vector<unsigned int> triangle_indices(tri_index_count);
	file.read(reinterpret_cast<char*>(triangle_indices.data()), tri_index_count * sizeof(unsigned int));

	// Read triangles
	size_t triangle_count;
	file.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
	std::vector<cg_datastructures::Triangle> triangles(triangle_count);
	file.read(reinterpret_cast<char*>(triangles.data()), triangle_count * sizeof(cg_datastructures::Triangle));

	file.close();
	LOG_INFO("Voxel grid loaded successfully");

	return VoxelGrid{
		std::move(voxels),
		std::move(triangle_indices),
		nx, ny, nz,
		minBound, maxBound, voxelSize,
		std::move(triangles)
	};
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