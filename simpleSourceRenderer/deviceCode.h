#pragma once

#include <owl/owl.h>
#include <owl/common/math/vec.h>
#include <owl/common/math/box.h>


/* variables for the triangle mesh geometry */
struct TrianglesGeomData
{
    /*! base color we use for the entire mesh */
    owl::vec3f color;
    /*! array/buffer of vertex indices */
    owl::vec3i* index;
    /*! array/buffer of vertex positions */
    owl::vec3f* vertex;
};

/* variables for the ray generation program */
struct RayGenData
{
    uint32_t* fbPtr;
    owl::vec2i  fbSize;
    OptixTraversableHandle world;
    struct {
	    owl::vec3f pos;
	    owl::vec3f dir_00;
	    owl::vec3f dir_du;
	    owl::vec3f dir_dv;
    } camera;
};

struct MissProgData
{
	
};


// AABB with offset into buffer which contains data
struct SourceRegion
{
    owl::box3i sourceBox{ {0,0,0}, {0,0,0} };
    uint64_t bufferOffset{0};
};

struct VolumeGridData {
    float1 *gridData;
    SourceRegion *sourceRegions;
    int sourceCount;
    owl::vec3i gridDims;
    owl::vec2f minmax;
};
