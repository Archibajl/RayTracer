#pragma once
#include "Voxels.h"
#include <cmath>

namespace vector_math
{
    using namespace cg_datastructures;
    // Helper functions for vector math
    inline Vec3 normalize(const Vec3& v) {
        float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len > 0.0f) {
            return { v.x / len, v.y / len, v.z / len };
        }
        return v;
    }

    inline Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    inline float dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline Vec3 subtract(const Vec3& a, const Vec3& b) {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    inline Vec3 add(const Vec3& a, const Vec3& b) {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    inline Vec3 multiply(const Vec3& v, float s) {
        return { v.x * s, v.y * s, v.z * s };
    }

    inline int getDistanceTraveled(const Vec3& a, const Vec3& b) {
        return static_cast<int>(std::sqrt(
            (b.x - a.x) * (b.x - a.x) +
            (b.y - a.y) * (b.y - a.y) +
            (b.z - a.z) * (b.z - a.z)
        ));
	}
};

