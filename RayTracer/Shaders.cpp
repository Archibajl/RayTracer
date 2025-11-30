#include "Shaders.h"


Shaders::Pixel Shaders::LambertianShader(const RayHit& hit, const float r, const float g, const float b, const cg_datastructures::Triangle& triangle) {
	Pixel pixel = Shaders::Pixel(r, g, b);

	// Simple Lambertian shading
	Vec3 P = hit.point;
	Vec3 L = normalize(subtract(hit.origin, P));
	Vec3 N = normalize(hit.normal);

	float dotNL = std::max(0.0f, dot(N, L));
	float kd = 1.0f; // Diffuse coefficient
	Vec3 lightColor = pixel.color;
	Vec3 diffuse = multiply(scale(kd, lightColor), dotNL);
	float inShadow = 0.0f; // No shadows for now
	Vec3 ambient = multiply(pixel.color, 0.1f); // Ambient term

	pixel.color = add(ambient, scale((1 - inShadow) , diffuse));

	return pixel;
}