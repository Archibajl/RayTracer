// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "Logger.h"
#include <filesystem>
#include "TraceImages.h"


int main() {
	// Initialize logger
	Logger::init();
	Logger::setLevel(spdlog::level::debug);

	LOG_INFO("========================================");
	LOG_INFO("Ray Tracer Starting");
	LOG_INFO("Current working directory: {}", std::filesystem::current_path().string());
	LOG_INFO("========================================");

	try {
		TraceImages tracer = TraceImages();

		// Process multiple STL files
		//Resistor Model
		tracer.TraceImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Generic-scans\\Resistor.stl", "resistor.png");
		//Airless Ball Model
		tracer.TraceImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Generic-scans\\Airless_2.stl", "airless-ball.png");
		//Utah Teapot High Res
		tracer.TraceImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Utah_Teapot\\utahHR.stl", "utahTeapot.png");
		//Lucy Model -- High Res
		tracer.TraceImage("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\lucy_scans\\lucy\\lucy.stl", "lucy.png");

		LOG_INFO("========================================");
		LOG_INFO("All processing complete!");
		LOG_INFO("========================================");
		return 0;
	}
	catch (const std::exception& e) {
		LOG_CRITICAL("========================================");
		LOG_CRITICAL("FATAL ERROR: {}", e.what());
		LOG_CRITICAL("========================================");
		return 1;
	}
}



