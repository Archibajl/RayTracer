// RayTracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "STLReader.cpp";
#include <filesystem>
#include <iostream>
using namespace std;


int main() {
	StlReader reader = StlReader();

	//int value = reader.read_stl("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Generic-scans\\Airless_2.stl");
	stl_reader::StlMesh<float, unsigned int> mesh = reader.read_stl("C:\\Users\\j`\\OneDrive\\Documents\\MS-UCCS\\CS5800\\mesh models\\Utah_Teapot\\tea.stl");
	cout << std::filesystem::current_path() << std::endl;
	//cout << mesh.tri_normal;
	cout << mesh.num_tris();
	//cout << value;
}

//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <vector>
//#include <limits>
//#include <vulkan/vulkan.h>
//#include <iostream>
//
//// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
//// Debug program: F5 or Debug > Start Debugging menu
//
//// Tips for Getting Started: 
////   1. Use the Solution Explorer window to add/manage files
////   2. Use the Team Explorer window to connect to source control
////   3. Use the Output window to see build output and other messages
////   4. Use the Error List window to view errors
////   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
////   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
//
//
//
//struct Vec3 {
//    double x, y, z;
//    Vec3 operator+(const Vec3& v) const { return { x + v.x, y + v.y, z + v.z }; }
//    Vec3 operator-(const Vec3& v) const { return { x - v.x, y - v.y, z - v.z }; }
//    Vec3 operator*(double s) const { return { x * s, y * s, z * s }; }
//    Vec3 operator/(double s) const { return { x / s, y / s, z / s }; }
//    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
//    Vec3 normalize() const { return *this / std::sqrt(dot(*this)); }
//};
//
//struct Ray {
//    Vec3 origin, direction;
//};
//
//struct Sphere {
//    Vec3 center;
//    double radius;
//    Vec3 color;
//
//    bool intersect(const Ray& ray, double& t) const {
//        Vec3 oc = ray.origin - center;
//        double b = 2.0 * oc.dot(ray.direction);
//        double c = oc.dot(oc) - radius * radius;
//        double discriminant = b * b - 4 * c;
//        if (discriminant < 0) return false;
//        t = (-b - std::sqrt(discriminant)) / 2.0;
//        return t > 0;
//    }
//};
//
//Vec3 trace(const Ray& ray, const std::vector<Sphere>& spheres) {
//    double t_min = std::numeric_limits<double>::max();
//    const Sphere* hit_sphere = nullptr;
//
//    for (const auto& sphere : spheres) {
//        double t;
//        if (sphere.intersect(ray, t) && t < t_min) {
//            t_min = t;
//            hit_sphere = &sphere;
//        }
//    }
//
//    if (hit_sphere) {
//        Vec3 hit_point = ray.origin + ray.direction * t_min;
//        Vec3 normal = (hit_point - hit_sphere->center).normalize();
//        double intensity = std::max(0.0, normal.dot({ 0, 0, -1 }));
//        return hit_sphere->color * intensity;
//    }
//
//    return { 0.2, 0.7, 0.8 }; // background color
//}
//
//
//
//int main() {
//    VkInstance instance;
//    VkApplicationInfo appInfo{};
//    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
//    appInfo.pApplicationName = "RayTracer";
//    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
//    appInfo.pEngineName = "NoEngine";
//    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
//    appInfo.apiVersion = VK_API_VERSION_1_2;
//
//    VkInstanceCreateInfo createInfo{};
//    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
//    createInfo.pApplicationInfo = &appInfo;
//
//    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
//        std::cerr << "Failed to create Vulkan instance\n";
//        return -1;
//    }
//
//    std::cout << "Vulkan initialized\n";
//    vkDestroyInstance(instance, nullptr);
//    return 0;
//}
//
//
//int main1() {
//    const int width = 800, height = 600;
//    std::ofstream out("render.ppm");
//    out << "P3\n" << width << " " << height << "\n255\n";
//
//    std::vector<Sphere> scene = {
//        {{0, 0, -5}, 1, {255, 0, 0}},
//        {{2, 0, -6}, 1, {0, 255, 0}},
//        {{-2, 0, -6}, 1, {0, 0, 255}}
//    };
//
//    for (int y = 0; y < height; ++y) {
//        for (int x = 0; x < width; ++x) {
//            double fx = (2.0 * x / width - 1.0) * (width / (double)height);
//            double fy = 1.0 - 2.0 * y / height;
//            Ray ray = { {0, 0, 0}, {fx, fy, -1}.normalize() };
//            Vec3 color = trace(ray, scene);
//            out << (int)color.x << " " << (int)color.y << " " << (int)color.z << "\n";
//        }
//    }
//
//    out.close();
//    std::cout << "Rendered to render.ppm\n";
//    return 0;
//}