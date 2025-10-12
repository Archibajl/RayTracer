#include "STLReader.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
using namespace std;

class StlReader {

    // Uses open stl stl Header to read in an .stl file type
public:
    stl_reader::StlMesh<float, unsigned int> read_stl(std::string filename)
    {
        try {
            stl_reader::StlMesh<float, unsigned int> mesh(filename);

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
            return mesh;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return NULL;
        }

        return 0;
    }
};