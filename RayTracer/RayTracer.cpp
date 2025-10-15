// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "MeshGenerator.cpp"
#include "SceneCreator.cpp"
#include "Logger.h"
#include <filesystem>
#include "SceneCreator.h"
using namespace std;


int main() {
	// Initialize logger
	Logger::init();

	LOG_INFO("Current working directory: {}", std::filesystem::current_path().string());

	MeshGenerator meshGenerator = MeshGenerator();

	StlMeshCuda mesh = meshGenerator.generateMeshes("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Utah_Teapot\\tea.stl");

	LOG_INFO("Loaded mesh with {} triangles", mesh.num_tris);

	VoxelGrid voxelGrid = BuildVoxelGridFromStlMeshCuda(mesh, 50, 50, 50);

	LOG_INFO("Voxel grid created: {} voxels", 50 * 50 * 50);
	LOG_INFO("Voxel size: ({}, {}, {})",
		voxelGrid.voxelSize.x,
		voxelGrid.voxelSize.y,
		voxelGrid.voxelSize.z);

}

