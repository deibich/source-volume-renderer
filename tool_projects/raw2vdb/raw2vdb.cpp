#include <iostream>
#include <string>

#include <openvdb/tools/Dense.h>
#include <openvdb/io/Stream.h>

typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;

using namespace openvdb;

int main(int ac, char **av)
{
    // check arguments
    std::string inFileName = av[1];
    std::string format = av[2]; // e.g. "float", "uint8", "uint16", "uint32", "uint64", "int8", "int16", "int32", "int64", "double"
    // resolution of your data
    size_t RES_X = atoi(av[3]);
    size_t RES_Y = atoi(av[4]);
    size_t RES_Z = atoi(av[5]);
    std::string outFileName = av[6];

    std::cout << "Reading " << inFileName << std::endl;
    std::cout << "Format: " << format << std::endl;
    std::cout << "Resolution: " << RES_X << " " << RES_Y << " " << RES_Z << std::endl;
    
    size_t numVoxels = RES_X * RES_Y * RES_Z;
    
    std::vector<float> data;
    data.resize(numVoxels);
    FILE* fp = fopen(inFileName.c_str(), "rb");
    std::cout << "Read " << fread(data.data(), sizeof(float), numVoxels, fp) << " Elements" << std::endl;
    fclose(fp);

    math::CoordBBox bbox(Coord(0), Coord(RES_X-1, RES_Y-1, RES_Z-1)); // resolution of your data basically
	tools::Dense<float, tools::LayoutXYZ> dense_grid(bbox, data.data()); // LayoutXYZ is for cases where x is fastest loop varying index

		
    FloatGrid::Ptr grid = FloatGrid::create(0.0f);

    tools::copyFromDense(dense_grid, *grid, 1e-6f); // 1e-3f is the tolerance for the voxel values

    // set grid class
    grid->setGridClass(GRID_FOG_VOLUME);
    
    // write vdb grid
    std::cout << "Writing " << outFileName << std::endl;
    
    // Create a VDB file object.
    openvdb::io::File file(outFileName);
    
    // Add the grid pointer to a container.
    openvdb::GridPtrVec grids;
    grids.push_back(grid);

    // Write out the contents of the container.
    file.write(grids);
    file.close();
    return EXIT_SUCCESS;
}