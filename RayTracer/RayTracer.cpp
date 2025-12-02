// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "Logger.h"
#include <filesystem>
#include "TraceImages.h"


int main() {
	Logger::init();
	Logger::setLevel(spdlog::level::debug);

	LOG_INFO("========================================");
	LOG_INFO("Ray Tracer Starting");
	LOG_INFO("========================================");

	try {
		TraceImages tracer;

		// Base directory for models
		const std::string modelDir = "C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models";

		// Model configurations: {input_path, output_name}
		std::vector<std::pair<std::string, std::string>> models = {
			{modelDir + "\\Generic-scans\\Resistor.stl", "resistor.jpg"},
			{modelDir + "\\Generic-scans\\Airless_2.stl", "airless-ball.jpg"},
			{modelDir + "\\Utah_Teapot\\utahHR.stl", "utahTeapot.jpg"},
			{modelDir + "\\lucy_scans\\lucy\\lucy.stl", "lucy.jpg"}
		};

		// Process all models
		for (const auto& [inputPath, outputName] : models) {
			tracer.TraceImage(inputPath, outputName, RayTracingMethod::OCTREE);
		}

		LOG_INFO("========================================");
		LOG_INFO("All processing complete!");
		LOG_INFO("========================================");
		return 0;
	}
	catch (const std::exception& e) {
		LOG_CRITICAL("FATAL ERROR: {}", e.what());
		return 1;
	}
}



