#pragma once

#include "RayTracerCommon.h"
#include "VectorMath.h"
#include <algorithm>
#include "TraceImages.h"

using namespace cg_datastructures;
using namespace vector_math;

class Shaders
{
public : struct Pixel {
		Vec3 color;
		float opacity;

		Pixel() : color{ 0.0f, 0.0f, 0.0f }, opacity(1.0f) {}
		Pixel(float r, float g, float b) : color{r, g, b }, opacity(1.0f) {}
	};

	public:
		static Pixel LambertianShader(const RayHit& hit, const float r, const float g, const float b, const cg_datastructures::Triangle& triangle);
};

