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

            return mesh;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return NULL;
        }

        return 0;
    }
};