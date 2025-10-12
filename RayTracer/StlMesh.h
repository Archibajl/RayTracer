// Struct for use on the device (CUDA)
struct StlMesh
{
    const float* coords;    // [num_vrts * 3]
    const float* normals;   // [num_tris * 3]
    const unsigned int* tris; // [num_tris * 3]
    size_t num_vrts;
    size_t num_tris;
};