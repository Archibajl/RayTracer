#include "TraceImages.h"
#include "Logger.h"
#include <filesystem>
#include <chrono>
#include "SceneCreator.h"
#include "OctreeRayTracer.h"
#include "ImageSaver.h"
#include "STLReader.h"
using namespace std;
using namespace cg_datastructures;

// ============================================================================
// LOGGING HELPERS
// ============================================================================

std::string TraceImages::getRayTracingMethodName(RayTracingMethod method) {
	switch (method) {
		case RayTracingMethod::VOXEL_DDA: return "Voxel Grid DDA";
		case RayTracingMethod::ART: return "ARTS (3DDDA)";
		case RayTracingMethod::OCTREE: return "Octree";
		default: return "Unknown";
	}
}

void TraceImages::logStartInfo(const std::string& gridFileLocation, const std::string& outputFileName, RayTracingMethod method) {
	LOG_INFO("========== Starting image generation ==========");
	LOG_INFO("Input: {} | Output: {} | Method: {}", gridFileLocation, outputFileName, getRayTracingMethodName(method));
}

void TraceImages::logCompletionInfo(double totalTimeSeconds, const std::string& outputFileName) {
	LOG_INFO("Success! Generated {} in {:.3f}s", outputFileName, totalTimeSeconds);
}

void TraceImages::logErrorInfo(const std::string& gridFileLocation, const std::string& errorMessage, double timeSeconds) {
	LOG_ERROR("Failed: {} - {} (after {:.3f}s)", gridFileLocation, errorMessage, timeSeconds);
}

void TraceImages::renderImageWithTiming(VoxelGrid& voxelGrid, const std::string& outputFileName, RayTracingMethod method) {
	auto start = std::chrono::high_resolution_clock::now();
	genateImageFromGrid(voxelGrid, outputFileName, method);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	LOG_INFO("Ray tracing time: {:.3f}s", duration.count() / 1000.0);
}

// ============================================================================
// MAIN IMAGE TRACING FUNCTION
// ============================================================================

void TraceImages::TraceImage(std::string gridFileLocation, std::string outputFileName, RayTracingMethod method) {
	logStartInfo(gridFileLocation, outputFileName, method);

	// Start total timer
	auto totalStart = std::chrono::high_resolution_clock::now();

	try {
		// Build or load voxel grid
		VoxelGrid voxelGrid = loadOrGenerateVoxelGrid(gridFileLocation, 50, 50, 50);

		// Generate images using specified method
		renderImageWithTiming(voxelGrid, outputFileName, method);

		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		logCompletionInfo(totalDuration.count() / 1000.0, outputFileName);
	}
	catch (const std::exception& e) {
		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		logErrorInfo(gridFileLocation, e.what(), totalDuration.count() / 1000.0);
		// Don't re-throw, just continue to next image
	}
}

void TraceImages::TraceImageMultiView(std::string gridFileLocation, std::string baseOutputName, RayTracingMethod method) {
	LOG_INFO("========== Starting multi-view image generation ==========");
	LOG_INFO("Input: {} | Base output: {} | Method: {}", gridFileLocation, baseOutputName, getRayTracingMethodName(method));

	// Start total timer
	auto totalStart = std::chrono::high_resolution_clock::now();

	try {
		// Build or load voxel grid (only once for all views)
		VoxelGrid voxelGrid = loadOrGenerateVoxelGrid(gridFileLocation, 50, 50, 50);

		// Extract base name and extension from output filename
		std::filesystem::path basePath(baseOutputName);
		std::string extension = basePath.extension().string();
		std::string nameWithoutExt = basePath.stem().string();

		// Generate images from different views
		std::vector<std::pair<CameraView, std::string>> views = {
			{CameraView::FRONT, nameWithoutExt + "_front" + extension},
			{CameraView::SIDE, nameWithoutExt + "_side" + extension},
			{CameraView::TOP, nameWithoutExt + "_top" + extension}
		};

		for (const auto& [view, filename] : views) {
			auto start = std::chrono::high_resolution_clock::now();
			generateImageFromGridWithView(voxelGrid, filename, method, view);
			auto end = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			LOG_INFO("View rendered in {:.3f}s", duration.count() / 1000.0);
		}

		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		LOG_INFO("Success! Generated all views in {:.3f}s", totalDuration.count() / 1000.0);
	}
	catch (const std::exception& e) {
		auto totalEnd = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart);
		logErrorInfo(gridFileLocation, e.what(), totalDuration.count() / 1000.0);
	}
}

// ============================================================================
// CAMERA SETUP
// ============================================================================

Camera TraceImages::setupCamera(const VoxelGrid& voxelGrid) {
	Vec3 center = voxelGrid.center;
	Vec3 size = {
		voxelGrid.maxBound.x - voxelGrid.minBound.x,
		voxelGrid.maxBound.y - voxelGrid.minBound.y,
		voxelGrid.maxBound.z - voxelGrid.minBound.z
	};

	float maxSize = std::max({size.x, size.y, size.z});
	float cameraDistance = maxSize * 2.5f;

	LOG_INFO("Camera: center({:.2f}, {:.2f}, {:.2f}), size({:.2f}, {:.2f}, {:.2f}), dist={:.2f}",
		center.x, center.y, center.z, size.x, size.y, size.z, cameraDistance);

	return Camera(
		{center.x, center.y, center.z - cameraDistance},  // Position
		center,                                            // Look at
		{0.0f, 1.0f, 0.0f},                               // Up
		60.0f,                                             // FOV
		800.0f / 600.0f                                    // Aspect ratio
	);
}

Camera TraceImages::setupCameraWithView(const VoxelGrid& voxelGrid, CameraView view) {
	Vec3 center = voxelGrid.center;
	Vec3 size = {
		voxelGrid.maxBound.x - voxelGrid.minBound.x,
		voxelGrid.maxBound.y - voxelGrid.minBound.y,
		voxelGrid.maxBound.z - voxelGrid.minBound.z
	};

	float maxSize = std::max({size.x, size.y, size.z});
	float cameraDistance = maxSize * 2.5f;

	Vec3 cameraPosition;
	Vec3 upVector;
	std::string viewName;

	switch (view) {
		case CameraView::FRONT:
			// Looking from front (negative Z direction)
			cameraPosition = {center.x, center.y, center.z - cameraDistance};
			upVector = {0.0f, 1.0f, 0.0f};
			viewName = "FRONT";
			break;

		case CameraView::SIDE:
			// Looking from side (positive X direction)
			cameraPosition = {center.x + cameraDistance, center.y, center.z};
			upVector = {0.0f, 1.0f, 0.0f};
			viewName = "SIDE";
			break;

		case CameraView::TOP:
			// Looking from top (positive Y direction)
			cameraPosition = {center.x, center.y + cameraDistance, center.z};
			upVector = {0.0f, 0.0f, 1.0f};  // Z is up when looking from top
			viewName = "TOP";
			break;
	}

	LOG_INFO("Camera {}: position({:.2f}, {:.2f}, {:.2f}), lookAt({:.2f}, {:.2f}, {:.2f})",
		viewName, cameraPosition.x, cameraPosition.y, cameraPosition.z,
		center.x, center.y, center.z);

	return Camera(
		cameraPosition,  // Position
		center,          // Look at
		upVector,        // Up
		60.0f,           // FOV
		800.0f / 600.0f  // Aspect ratio
	);
}

std::vector<Vec3> TraceImages::renderWithTiming(IRayTracer* raytracer, const Camera& camera, int width, int height) {
	auto start = std::chrono::high_resolution_clock::now();
	auto pixels = raytracer->render(camera, width, height);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	LOG_INFO("Rendered {} pixels in {:.3f}s ({:.0f} rays/sec)",
		pixels.size(), duration.count() / 1000.0, (width * height) / (duration.count() / 1000.0));

	return pixels;
}

// ============================================================================
// IMAGE GENERATION FROM VOXEL GRID
// ============================================================================

void TraceImages::genateImageFromGrid(VoxelGrid voxelGrid, std::string filename, RayTracingMethod method) {
	// Create ray tracer using factory method
	std::unique_ptr<IRayTracer> raytracer = createRayTracer(voxelGrid, method);

	// Setup camera
	Camera camera = setupCamera(voxelGrid);

	// Render image
	int width = 800, height = 600;
	auto pixels = renderWithTiming(raytracer.get(), camera, width, height);

	// Output image
	std::string filepath = "C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\GeneratedFiles\\" + filename;
	SaveImage(filepath, pixels, width, height);
}

void TraceImages::generateImageFromGridWithView(VoxelGrid voxelGrid, std::string filename, RayTracingMethod method, CameraView view) {
	// Create ray tracer using factory method
	std::unique_ptr<IRayTracer> raytracer = createRayTracer(voxelGrid, method);

	// Setup camera with specific view
	Camera camera = setupCameraWithView(voxelGrid, view);

	// Render image
	int width = 800, height = 600;
	auto pixels = renderWithTiming(raytracer.get(), camera, width, height);

	// Output image
	std::string filepath = "C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\GeneratedFiles\\" + filename;
	SaveImage(filepath, pixels, width, height);
}

// ============================================================================
// STL MESH LOADING
// ============================================================================

stl_reader::StlMesh<float, unsigned int> TraceImages::loadStlMesh(const std::string& filepath) {
	LOG_INFO("Loading STL file: {}", filepath);
	stl_reader::StlMesh<float, unsigned int> stlMesh(filepath);
	LOG_INFO("Loaded mesh with {} triangles", stlMesh.num_tris());
	return stlMesh;
}

// ============================================================================
// VOXEL GRID GENERATION
// ============================================================================

VoxelGrid TraceImages::generateVoxelGridFromStlMesh(const stl_reader::StlMesh<float, unsigned int>& stlMesh, int nx, int ny, int nz) {
	LOG_INFO("Generating voxel grid with dimensions {}x{}x{}", nx, ny, nz);
	// Use the refactored voxelization function from SceneCreator
	return BuildVoxelGridFromStlMesh(stlMesh, nx, ny, nz);
}

// ============================================================================
// IMAGE SAVING
// ============================================================================

void TraceImages::SaveImage(const std::string& filename,
	const std::vector<cg_datastructures::Vec3>& pixels,
	int width, int height) {
	std::filesystem::path filePath(filename);
	std::filesystem::path dirPath = filePath.parent_path();

	try {
		// Create output directory if needed
		if (!dirPath.empty() && !std::filesystem::exists(dirPath)) {
			std::filesystem::create_directories(dirPath);
		}

		// Get file extension (convert to lowercase for comparison)
		std::string extension = filePath.extension().string();
		std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

		bool success = false;

		// Save based on file extension
		if (extension == ".jpg" || extension == ".jpeg") {
			success = saveToJPG(filename, pixels, width, height, 90);
		}
		else if (extension == ".png") {
			success = saveToPNG(filename, pixels, width, height);
		}
		else if (extension == ".ppm") {
			success = saveToPPM(filename, pixels, width, height);
		}
		else {
			LOG_WARN("Unknown extension '{}', defaulting to PPM format", extension);
			success = saveToPPM(filename, pixels, width, height);
		}

		if (success) {
			LOG_INFO("Image saved: {}", filename);
		} else {
			LOG_ERROR("Failed to save: {}", filename);
		}
	}
	catch (const std::exception& e) {
		LOG_ERROR("Save error: {} - falling back to output.jpg", e.what());
		if (saveToJPG("output.jpg", pixels, width, height, 90)) {
			LOG_INFO("Saved to fallback: {}", std::filesystem::absolute("output.jpg").string());
		}
	}
}

// ============================================================================
// VOXEL GRID LOADING/GENERATION
// ============================================================================

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
		// Load STL mesh and generate voxel grid separately
		LOG_INFO("Processing STL file: {}", filepath);
		stl_reader::StlMesh<float, unsigned int> stlMesh = loadStlMesh(filepath);
		return generateVoxelGridFromStlMesh(stlMesh, nx, ny, nz);
	}
	else {
		LOG_ERROR("Unsupported file format: {}. Expected .stl or .voxgrid", extension);
		throw std::runtime_error("Unsupported file format: " + extension);
	}
}

// ============================================================================
// VOXEL GRID SERIALIZATION
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
	file.write(reinterpret_cast<const char*>(&center), sizeof(center));

	// Write voxels as 3D array
	for (int x = 0; x < nx; ++x) {
		for (int y = 0; y < ny; ++y) {
			for (int z = 0; z < nz; ++z) {
				file.write(reinterpret_cast<const char*>(&voxels[x][y][z]), sizeof(cg_datastructures::Voxel));
			}
		}
	}

	// Write triangle indices
	/*int tri_index_count = triangle_indices.size();
	file.write(reinterpret_cast<const char*>(&tri_index_count), sizeof(tri_index_count));
	file.write(reinterpret_cast<const char*>(triangle_indices.data()), tri_index_count * sizeof(unsigned int));*/

	// Write triangles
	int triangle_count = triangles.size();
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
		throw std::runtime_error("Failed to open file: " + filename);
	}

	// Read dimensions
	int nx, ny, nz;
	file.read(reinterpret_cast<char*>(&nx), sizeof(nx));
	file.read(reinterpret_cast<char*>(&ny), sizeof(ny));
	file.read(reinterpret_cast<char*>(&nz), sizeof(nz));

	// Read bounds
	cg_datastructures::Vec3 minBound, maxBound, voxelSize, center;
	file.read(reinterpret_cast<char*>(&minBound), sizeof(minBound));
	file.read(reinterpret_cast<char*>(&maxBound), sizeof(maxBound));
	file.read(reinterpret_cast<char*>(&voxelSize), sizeof(voxelSize));
	file.read(reinterpret_cast<char*>(&center), sizeof(center));

	// Read voxels as 3D array
	std::vector<std::vector<std::vector<cg_datastructures::Voxel>>> voxels(nx);
	for (int x = 0; x < nx; ++x) {
		voxels[x].resize(ny);
		for (int y = 0; y < ny; ++y) {
			voxels[x][y].resize(nz);
			for (int z = 0; z < nz; ++z) {
				file.read(reinterpret_cast<char*>(&voxels[x][y][z]), sizeof(cg_datastructures::Voxel));
			}
		}
	}

	// Read triangles
	int triangle_count;
	file.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
	std::vector<cg_datastructures::Triangle> triangles(triangle_count);
	file.read(reinterpret_cast<char*>(triangles.data()), triangle_count * sizeof(cg_datastructures::Triangle));

	file.close();
	LOG_INFO("Voxel grid loaded successfully: {}x{}x{} voxels, {} triangles", nx, ny, nz, triangle_count);

	// Note: triangle_indices is not saved/loaded, so we create an empty vector
	std::vector<unsigned int> triangle_indices;

	return VoxelGrid(
		std::move(voxels),
		std::move(triangle_indices),
		nx, ny, nz,
		minBound, maxBound, voxelSize,
		std::move(triangles)
	);
}

// ============================================================================
// RAY TRACER FACTORY
// ============================================================================

// Factory method to create appropriate ray tracer
std::unique_ptr<IRayTracer> TraceImages::createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method) {
	switch (method) {
		case RayTracingMethod::VOXEL_DDA:
			// VOXEL_DDA uses Octree ray tracing
			return std::make_unique<OctreeLikeRayTracer>(voxelGrid);

		case RayTracingMethod::ART:
			return std::make_unique<OctreeLikeRayTracer>(voxelGrid);

		case RayTracingMethod::OCTREE:
			return std::make_unique<OctreeLikeRayTracer>(voxelGrid);

		// Future methods can be added here:
		// case RayTracingMethod::BVH:
		//     return std::make_unique<BVHRayTracer>(voxelGrid);

		default:
			LOG_WARN("Unknown ray tracing method, defaulting to Octree");
			return std::make_unique<OctreeLikeRayTracer>(voxelGrid);
	}
}