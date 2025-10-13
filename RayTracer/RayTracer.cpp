// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "STLReader.cpp";
#include "MeshGenerator.cpp"
#include <filesystem>
#include <iostream>
using namespace std;


int main() {
	StlReader reader = StlReader();

	//int value = reader.read_stl("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Generic-scans\\Airless_2.stl");
	MeshGenerator meshGenerator = MeshGenerator();
	StlMesh mesh = meshGenerator.generateMeshes("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Utah_Teapot\\tea.stl");

	cout << std::filesystem::current_path() << std::endl;
	//cout << mesh.tri_normal;
	cout << mesh.num_tris;
	//cout << value;
}

