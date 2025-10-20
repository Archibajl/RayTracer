#include "TraceImages.h"
#include "Logger.h"
#include <filesystem>
#include "SceneCreator.h"
#include "VoxelDDARayTracer.h"
#include "ImageSaver.h"
#include "MeshGenerator.h"
#include "VoxelDDARayTracer.cpp"
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
		default: methodName = "Unknown"; break;
	}
	LOG_INFO("Ray tracing method: {}", methodName);
	LOG_INFO("========================================");

	try {
		// Build voxel grid
		VoxelGrid voxelGrid = generateVoxelGridFromFile(gridFileLocation, 50, 50, 50);
		// Generate images using specified method
		genateImageFromGrid(voxelGrid, outputFileName, method);

		LOG_INFO("Successfully generated image: {}", outputFileName);
	}
	catch (const std::exception& e) {
		LOG_ERROR("========================================");
		LOG_ERROR("FAILED to generate image for: {}", gridFileLocation);
		LOG_ERROR("Error: {}", e.what());
		LOG_ERROR("========================================");
		// Don't re-throw, just continue to next image
	}

	////Teapot model Image Generation
	////// Build voxel grids
	//VoxelGrid teapotGrid = generateVoxelGridFromFile("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\teapot.stl", 50, 50, 50);
	//// Generate images
	//genateImageFromGrid(teapotGrid, "teapot.png");
	//teapotGrid.~VoxelGrid();

	////Teapot model Image Generation
	////// Build voxel grids
	//VoxelGrid teapotGrid = generateVoxelGridFromFile("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\teapot.stl", 50, 50, 50);
	//// Generate images
	//genateImageFromGrid(teapotGrid, "teapot.png");
	//teapotGrid.~VoxelGrid();

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

	auto pixels = raytracer->render(camera, width, height);
	LOG_INFO("Rendering complete!");
	LOG_INFO("Image rendered with {} pixels", pixels.size());

	// Output image
	SaveImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\GeneratedFiles\\" + filename, pixels, width, height);
}

VoxelGrid TraceImages::generateVoxelGridFromFile(const std::string filepath, int nx, int ny, int nz) {
	
	// Load mesh from STL file
	MeshGenerator meshGenerator = MeshGenerator();
	StlMeshCuda mesh = meshGenerator.generateMeshes(filepath);
	LOG_INFO("Loaded mesh with {} triangles", mesh.num_tris);
	// Build voxel grid from mesh
	VoxelGrid voxelGrid = generateVoxelGridFromMesh(mesh, nx, ny, nz);

	LOG_INFO("Voxel grid created: {} voxels", nx * ny * nz);
	return voxelGrid;
}

VoxelGrid TraceImages::generateVoxelGridFromMesh(StlMeshCuda mesh, int nx, int ny, int nz) {
		// Build voxel grid from mesh
		VoxelGrid voxelGrid = BuildVoxelGridFromStlMeshCuda(mesh, nx, ny, nz);

		LOG_INFO("Voxel size: ({}, {}, {})",
			voxelGrid.voxelSize.x,
			voxelGrid.voxelSize.y,
			voxelGrid.voxelSize.z);
		LOG_INFO("Grid bounds: min=({}, {}, {}), max=({}, {}, {})",
			voxelGrid.minBound.x, voxelGrid.minBound.y, voxelGrid.minBound.z,
			voxelGrid.maxBound.x, voxelGrid.maxBound.y, voxelGrid.maxBound.z);

		// Count occupied voxels
		int occupiedCount = 0;
		for (const auto& voxel : voxelGrid.voxels) {
			if (voxel.occupied) occupiedCount++;
		}
		LOG_INFO("Occupied voxels: {} / {}", occupiedCount, voxelGrid.voxels.size());

		return voxelGrid;
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

// Factory method to create appropriate ray tracer
std::unique_ptr<IRayTracer> TraceImages::createRayTracer(VoxelGrid& voxelGrid, RayTracingMethod method) {
	switch (method) {
		case RayTracingMethod::VOXEL_DDA:
			return std::make_unique<VoxelDDARayTracer>(voxelGrid);

		// Future methods can be added here:
		// case RayTracingMethod::BVH:
		//     return std::make_unique<BVHRayTracer>(voxelGrid);

		default:
			LOG_WARN("Unknown ray tracing method, defaulting to Voxel Grid DDA");
			return std::make_unique<VoxelDDARayTracer>(voxelGrid);
	}
}