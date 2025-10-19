#pragma once
#include "SceneCreator.h"
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include "Logger.h"

namespace cg_datastructures {

// Save image to PPM format (simple, no dependencies)
inline bool saveToPPM(const std::string& filename,
                      const std::vector<Vec3>& pixels,
                      int width, int height) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    // Write PPM header
    file << "P6\n" << width << " " << height << "\n255\n";

    // Write pixels
    for (const auto& pixel : pixels) {
        uint8_t r = static_cast<uint8_t>(std::clamp(pixel.x * 255.0f, 0.0f, 255.0f));
        uint8_t g = static_cast<uint8_t>(std::clamp(pixel.y * 255.0f, 0.0f, 255.0f));
        uint8_t b = static_cast<uint8_t>(std::clamp(pixel.z * 255.0f, 0.0f, 255.0f));
        file.write(reinterpret_cast<const char*>(&r), 1);
        file.write(reinterpret_cast<const char*>(&g), 1);
        file.write(reinterpret_cast<const char*>(&b), 1);
    }

    return true;
}
//void PrintPixels(const std::vector<cg_datastructures::Vec3>& pixels, int width, int height) {
//    for (int y = 0; y < height; ++y) {
//        for (int x = 0; x < width; ++x) {
//            const auto& pixel = pixels[y * width + x];
//			if (pixel.x != 0 || pixel.y != 0 ) //|| pixel.z != 0)
//                LOG_INFO("({}, {}, {}) ", static_cast<int>(pixel.x), static_cast<int>(pixel.y), static_cast<int>(pixel.z));
//        }
//    }
//}

} // namespace cg_datastructures
