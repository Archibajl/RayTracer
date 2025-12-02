#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include "SceneCreator.h"
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include "Logger.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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

// Save image to JPG format using stb_image_write
inline bool saveToJPG(const std::string& filename,
                      const std::vector<Vec3>& pixels,
                      int width, int height,
                      int quality = 90) {
    // Convert Vec3 pixels to RGB bytes
    std::vector<uint8_t> imageData(width * height * 3);

    for (size_t i = 0; i < pixels.size(); ++i) {
        imageData[i * 3 + 0] = static_cast<uint8_t>(std::clamp(pixels[i].x * 255.0f, 0.0f, 255.0f));
        imageData[i * 3 + 1] = static_cast<uint8_t>(std::clamp(pixels[i].y * 255.0f, 0.0f, 255.0f));
        imageData[i * 3 + 2] = static_cast<uint8_t>(std::clamp(pixels[i].z * 255.0f, 0.0f, 255.0f));
    }

    // Write JPG file (quality: 1-100, higher = better quality but larger file)
    int result = stbi_write_jpg(filename.c_str(), width, height, 3, imageData.data(), quality);
    return result != 0;
}

// Save image to PNG format using stb_image_write
inline bool saveToPNG(const std::string& filename,
                      const std::vector<Vec3>& pixels,
                      int width, int height) {
    // Convert Vec3 pixels to RGB bytes
    std::vector<uint8_t> imageData(width * height * 3);

    for (size_t i = 0; i < pixels.size(); ++i) {
        imageData[i * 3 + 0] = static_cast<uint8_t>(std::clamp(pixels[i].x * 255.0f, 0.0f, 255.0f));
        imageData[i * 3 + 1] = static_cast<uint8_t>(std::clamp(pixels[i].y * 255.0f, 0.0f, 255.0f));
        imageData[i * 3 + 2] = static_cast<uint8_t>(std::clamp(pixels[i].z * 255.0f, 0.0f, 255.0f));
    }

    // Write PNG file (stride_in_bytes = 0 means tightly packed)
    int result = stbi_write_png(filename.c_str(), width, height, 3, imageData.data(), width * 3);
    return result != 0;
}


} // namespace cg_datastructures
