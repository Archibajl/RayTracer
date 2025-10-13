#include "MeshGenerator.h"

class MeshGenerator
{

    public: StlMesh generateMeshes(const std::string filePath) {
        stl_reader::StlMesh<float, unsigned int> stlObject = readStl(filePath);

        std::cout << stlObject.num_tris();

		//printMeshInfoStlMesh(stlObject);

        StlMesh mesh = convertMesh(stlObject);

		//std::cout << "Triangles : " + mesh.num_tris;
		//std::cout << "Number of vertecees: " + mesh.num_vrts;

		//printMeshInfoStlMesh(mesh);

		return mesh;
    }


    private : stl_reader::StlMesh<float, unsigned int> readStl(std::string filename) {
        //std::cout << std::filesystem::current_path() << std::endl;

        try {
            stl_reader::StlMesh<float, unsigned int> mesh(filename);

            std::cout << mesh.num_tris();

            return mesh;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return NULL;
        }
    }

    private : StlMesh convertMesh(stl_reader::StlMesh<float, unsigned int> mesh) {

        // Allocate device memory
        float* d_coords;
        float* d_normals;
        unsigned int* d_tris;

        size_t num_coords = mesh.num_vrts() * 3;
        size_t num_normals = mesh.num_tris() * 3;
        size_t num_tris = mesh.num_tris() * 3;

        // Fix: Cast the address to (void**) for cudaMalloc
        cudaMalloc((void**)&d_coords, num_coords * sizeof(float));
        cudaMalloc((void**)&d_normals, num_normals * sizeof(float));
        cudaMalloc((void**)&d_tris, num_tris * sizeof(unsigned int));

        cudaMemcpy(d_coords, mesh.raw_coords(), num_coords * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_normals, mesh.raw_normals(), num_normals * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_tris, mesh.raw_tris(), num_tris * sizeof(unsigned int), cudaMemcpyHostToDevice);

        StlMesh h_mesh_dev;
        h_mesh_dev.coords = d_coords;
        h_mesh_dev.normals = d_normals;
        h_mesh_dev.tris = d_tris;
        h_mesh_dev.num_vrts = mesh.num_vrts();
        h_mesh_dev.num_tris = mesh.num_tris();

        StlMesh* d_mesh_dev;
        // Fix the cudaMalloc call by casting the address to (void**)
        cudaMalloc((void**)&d_mesh_dev, sizeof(StlMesh));
        cudaMemcpy(d_mesh_dev, &h_mesh_dev, sizeof(StlMesh), cudaMemcpyHostToDevice);

		//cleanup(d_coords, d_normals, d_tris);

        return h_mesh_dev;
    }

 //   private : void cleanup(float* d_coords, float* d_normals, unsigned int* d_tris) {
 //       // Free device memory
 //       cudaFree(d_coords);
 //       cudaFree(d_normals);
 //       cudaFree(d_tris);
	//}

    private: void printMeshInfoStlMesh(const stl_reader::StlMesh<float, unsigned int>& mesh) {
    std::cout << "Number of vertices: " << mesh.num_vrts() << std::endl;
    std::cout << "Number of triangles: " << mesh.num_tris() << std::endl;

        // Iterate through all triangles
        for (size_t i = 0; i < mesh.num_tris(); ++i) {
            std::cout << "Triangle " << i << ": ";
            for (size_t j = 0; j < 3; ++j) {
                const float* c = mesh.tri_corner_coords(i, j);
                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
            }
            const float* n = mesh.tri_normal(i);
            std::cout << "Normal: (" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
        }
    }


    private: void printMeshInfoStlMesh(const StlMesh& mesh) {
        printf( "Number of vertices: " + mesh.num_vrts );
        printf( "Number of triangles: " + mesh.num_tris );
    
        // Iterate through all triangles
        for (size_t i = 0; i < mesh.num_tris; ++i) {
            printf( "Triangle %zu: ", i);
            for (size_t j = 0; j < 3; ++j) {
                const float* c = mesh.tri_corner_coords(i, j);
                printf("(%f, %f, %f) ", c[0], c[1], c[2]);
            }
            const float* n = mesh.tri_normal(i);
            printf( "Normal: (%f, %f, %f)\n", n[0], n[1], n[2]);
        }
    }

};