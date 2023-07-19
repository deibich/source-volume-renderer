// ======================================================================== //
// Copyright 2019-2020 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

// This program sets up a single geometric object, a mesh for a cube, and
// its acceleration structure, then ray traces it.

// public owl node-graph API
#include "owl/owl.h"
// our device-side data structures
#include "deviceCode.h"
// viewer base class, for window and user interaction
#include <fstream>

#include "tinyxml2.h"
#include <map>
#include "owlViewer/OWLViewer.h"
#include "fitsio.h"
#include <time.h>
using namespace owl;
#define LOG(message)                                            \
  std::cout << OWL_TERMINAL_BLUE;                               \
  std::cout << "#owl.sample(main): " << message << std::endl;   \
  std::cout << OWL_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << OWL_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#owl.sample(main): " << message << std::endl;   \
  std::cout << OWL_TERMINAL_DEFAULT;

extern "C" char deviceCode_ptx[];

// const vec2i fbSize(800,600);
const vec3f init_lookFrom(0.5f , 0.5f, -2.f);
const vec3f init_lookAt(0.5f, 0.5f, 0.f);
const vec3f init_lookUp(0.f, 1.f, 0.f);
const float init_cosFovy = 0.66f;// 0.3442489415681175f;//  0.66f;

struct Viewer : public owl::viewer::OWLViewer
{
    typedef owl::viewer::OWLViewer inherited;

    Viewer();

    /*! gets called whenever the viewer needs us to re-render out widget */
    void render() override;

    /*! window notifies us that we got resized. We HAVE to override
        this to know our actual render dimensions, and get pointer
        to the device frame buffer that the viewer cated for us */
    void resize(const vec2i& newSize) override;


    void key(char key, const vec2i& where) override;

    /*! this function gets called whenever any camera manipulator
      updates the camera. gets called AFTER all values have been updated */
    void cameraChanged() override;

    bool sbtDirty = true;
    OWLRayGen rayGen{ 0 };
    OWLContext context{ 0 };
    box3f volumeBox;
};

/*! window notifies us that we got resized */
void Viewer::resize(const vec2i& newSize)
{
    OWLViewer::resize(newSize);
    cameraChanged();
}

/*! window notifies us that the camera has changed */
void Viewer::cameraChanged()
{
    const vec3f lookFrom = camera.getFrom();
    const vec3f lookAt = camera.getAt();
    const vec3f lookUp = camera.getUp();
    const float cosFovy = camera.getCosFovy();
	
    std::cout << lookFrom << std::endl;
    // ----------- compute variable values  ------------------
    vec3f camera_pos = lookFrom;
    vec3f camera_d00
        = normalize(lookAt - lookFrom);
    float aspect = fbSize.x / float(fbSize.y);
    vec3f camera_ddu
        = cosFovy * aspect * normalize(cross(camera_d00, lookUp));
    vec3f camera_ddv
        = cosFovy * normalize(cross(camera_ddu, camera_d00));
    camera_d00 -= 0.5f * camera_ddu;
    camera_d00 -= 0.5f * camera_ddv;

    // ----------- set variables  ----------------------------
    owlRayGenSet1ul(rayGen, "fbPtr", (uint64_t)fbPointer);
    // owlRayGenSetBuffer(rayGen,"fbPtr",        frameBuffer);
    owlRayGenSet2i(rayGen, "fbSize", (const owl2i&)fbSize);
    owlRayGenSet3f(rayGen, "camera.pos", (const owl3f&)camera_pos);
    owlRayGenSet3f(rayGen, "camera.dir_00", (const owl3f&)camera_d00);
    owlRayGenSet3f(rayGen, "camera.dir_du", (const owl3f&)camera_ddu);
    owlRayGenSet3f(rayGen, "camera.dir_dv", (const owl3f&)camera_ddv);
    sbtDirty = true;
}


struct BoxCoordinateIndices
{
    std::map<std::string, int> idxMap = {
        {"x_min", -1},
        {"x_max", -1},
        {"y_min", -1},
        {"y_max", -1},
        {"z_min", -1},
        {"z_max", -1}
    };
};

struct SourceBox
{
    std::map<std::string, int> idxMap = {
        {"x_min", -1},
        {"x_max", -1},
        {"y_min", -1},
        {"y_max", -1},
        {"z_min", -1},
        {"z_max", -1}
    };

    box3i toBox()
    {
        return {
            {
                 idxMap["x_min"],
                idxMap["y_min"],
                idxMap["z_min"]
            },
            {
                 idxMap["x_max"] + 1,
                idxMap["y_max"]+ 1,
                idxMap["z_max"]+ 1
            }
        };
    }
};


/**
 * \brief 
 * \param filePath Path to VOTable xml file
 * \param sourceBoxes reference to existing vector of sourceRegions
 * \return Number of voxels for all sourceRegions
 */
size_t extractSourceRegionsFromXML(const std::string &filePath, std::vector<SourceRegion> & sourceBoxes)
{
    size_t voxels = 0;
    tinyxml2::XMLDocument doc;
    doc.LoadFile(filePath.c_str());

    auto tableElement = doc.FirstChildElement("VOTABLE")->FirstChildElement("RESOURCE")->FirstChildElement("TABLE");
    auto tableChild = tableElement->FirstChild();

    BoxCoordinateIndices bci;
    int tableElementsIdx = -1;
    for (auto currTableChild = tableChild; currTableChild != nullptr; currTableChild = currTableChild->NextSibling())
    {
        tableElementsIdx++;
        auto tableChildElement = currTableChild->ToElement();
        if (tableChildElement == nullptr)
        {
            break;
        }
        auto nameAttribute = tableChildElement->Attribute("name");
        if (nameAttribute)
        {
            for (auto& mapEle : bci.idxMap)
            {
                if (mapEle.first == nameAttribute)
                {
                    mapEle.second = tableElementsIdx;
                    break;
                }
            }
        }
    }

    auto dataEntry = tableElement->LastChildElement("DATA")->FirstChildElement("TABLEDATA")->FirstChild();


    // Foreach TR-element (source)

    for (; dataEntry != nullptr; dataEntry = dataEntry->NextSibling())
    {
        SourceBox b;
        int idx = 0;
        auto idxToDo = bci.idxMap.size();
        for (auto tdEntry = dataEntry->FirstChild(); tdEntry != nullptr && idxToDo > 0; tdEntry = tdEntry->NextSibling())
        {
            for (auto& mapEle : bci.idxMap)
            {
                if (mapEle.second == idx)
                {
                    b.idxMap[mapEle.first] = tdEntry->ToElement()->IntText();
                    idxToDo--;
                }
            }
            idx++;
        }
        SourceRegion sr;
        sr.sourceBox = b.toBox();
        voxels += sr.sourceBox.volume();
        sourceBoxes.push_back(sr);
    }
    return voxels;
}

Viewer::Viewer() : OWLViewer("Standalone", vec2i(1200, 800), true, false)
{
    // ##################################################################
    // Read data and store to a vec
    // ##################################################################
    
    // This file contains 342x342x448 float elements
    
    

    std::string fileName_originalFits = "D:/data/work/b3d/n4565/n4565_lincube_big.fits";
    const std::string fileName_data = "D:/data/work/b3d/n4565/n4565_lincube_big_downsampled.raw";
    std::string fileName_catXML = "D:/data/work/b3d/n4565/sofia_output/outname_cat.xml";


    //fileName_originalFits = "D:/data/work/b3d/ska/40gb/sky_ldev_v2.fits";
    //fileName_catXML = "D:/data/work/b3d/ska/40gb/output/outname_cat.xml";



    std::vector<SourceRegion> sourceBoxes;
    /*sourceBoxes.push_back(
        {
            {{0,0,0},{1024,1024,448}},
            0
        }
    );*/
    // auto voxelCount = sourceBoxes.at(0).sourceBox.volume(); // extractSourceRegionsFromXML(fileName_catXML, sourceBoxes);
    auto voxelCount = extractSourceRegionsFromXML(fileName_catXML, sourceBoxes);

    fitsfile *fitsFile;
    int fitsError;
    ffopen(&fitsFile, fileName_originalFits.c_str(), READONLY, &fitsError);
    assert(fitsError == 0);

	int axisCount;
    int imgType;
    long axis[3];
    fits_get_img_param(fitsFile, 3, &imgType, &axisCount, &axis[0], &fitsError);
    assert(fitsError == 0);
	assert(axisCount == 3);
    assert(imgType == FLOAT_IMG);

	const box3i gridBox({ 0,0,0 }, { axis[0], axis[1], axis[2]});

    volumeBox =
    {
        vec3f(gridBox.lower),
        vec3f(gridBox.upper)
    };

    std::vector<float> data;
    data.resize(voxelCount);
    long firstPx[3];
    long lastPx[3];
    long inc[3] = {1,1,1};
    size_t nextDataIdx = 0;

    float nan = NAN;
    for (auto & sourceBox : sourceBoxes)
    {
        sourceBox.bufferOffset = nextDataIdx;
        firstPx[0] = sourceBox.sourceBox.lower.x + 1;
        firstPx[1] = sourceBox.sourceBox.lower.y + 1;
        firstPx[2] = sourceBox.sourceBox.lower.z + 1;

        lastPx[0] = sourceBox.sourceBox.upper.x;
        lastPx[1] = sourceBox.sourceBox.upper.y;
        lastPx[2] = sourceBox.sourceBox.upper.z;

        fits_read_subset(fitsFile, TFLOAT, firstPx, lastPx, inc, &nan, data.data() + nextDataIdx, 0, &fitsError);
        assert(fitsError == 0);
        nextDataIdx += sourceBox.sourceBox.volume();
    }
    size_t nans = 0;
    ffclos(fitsFile, &fitsError);
    for (int i = 0; i < data.size(); ++i)
    {
        isnan(data[i]) ? nans++ : 1;
    }   
    // create a context on the first device
    context = owlContextCreate(nullptr, 1);
    owlContextSetNumAttributeValues(context, 3);
    OWLModule module = owlModuleCreate(context, deviceCode_ptx);

    // ##################################################################
    // set up all the *GEOMS* we want to run that code on
    // ##################################################################
    
    OWLVarDecl volumeGridDataVars[] = {
    {"gridData", OWL_BUFPTR, OWL_OFFSETOF(VolumeGridData, gridData)},
    {"sourceRegions", OWL_BUFPTR, OWL_OFFSETOF(VolumeGridData, sourceRegions)},
    {"sourceCount", OWL_INT, OWL_OFFSETOF(VolumeGridData, sourceCount)},
    {"gridDims", OWL_INT3, OWL_OFFSETOF(VolumeGridData, gridDims)},
    {"minmax", OWL_FLOAT2, OWL_OFFSETOF(VolumeGridData, minmax)},
    { /* sentinel to mark end of list */ }
    };

    // Custom geometry type
    OWLGeomType voxGeomType = owlGeomTypeCreate(context, OWL_GEOMETRY_USER, sizeof(VolumeGridData), volumeGridDataVars, -1);

    // Set programs for custom geometry
    owlGeomTypeSetBoundsProg(voxGeomType, module, "VolumeGrid");

    
    owlGeomTypeSetIntersectProg(voxGeomType, 0, module, "VolumeGrid");
    owlGeomTypeSetClosestHit(voxGeomType, 0, module, "VolumeGrid");
    owlGeomTypeSetAnyHit(voxGeomType, 0, module, "VolumeGrid");

    // compile bounds program
    owlBuildPrograms(context);

    // Create gpu buffer and upload initial data
    const OWLBuffer owlDataBuffer = owlDeviceBufferCreate(context, OWL_FLOAT, data.size(), data.data());
    const OWLBuffer owlSourcesDataBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(SourceRegion), sourceBoxes.size(), sourceBoxes.data());
    
    // Create actual geometry
    OWLGeom voxGeom = owlGeomCreate(context, voxGeomType);
    
    const owl2f gridDataMinMax = 
    {
        *std::min_element(data.begin(), data.end()),
        *std::max_element(data.begin(), data.end())
    };

    const auto gridSize = gridBox.size();
    owlGeomSetPrimCount(voxGeom, sourceBoxes.size());
    owlGeomSetBuffer(voxGeom, "gridData", owlDataBuffer);
    owlGeomSetBuffer(voxGeom, "sourceRegions", owlSourcesDataBuffer);
    owlGeomSet1i(voxGeom, "sourceCount", sourceBoxes.size());
    owlGeomSet3i(voxGeom, "gridDims", { gridSize.x, gridSize.y, gridSize.z});
    owlGeomSet2f(voxGeom, "minmax", gridDataMinMax);

    OWLGroup userGeomGroup = owlUserGeomGroupCreate(context, 1, &voxGeom);
    owlGroupBuildAccel(userGeomGroup);
    OWLGroup world = owlInstanceGroupCreate(context, 1, &userGeomGroup);
    owlGroupBuildAccel(world);

    // ##################################################################
    // set miss and raygen program required for SBT
    // ##################################################################

    // -------------------------------------------------------
    // set up miss prog
    // -------------------------------------------------------
    OWLVarDecl missProgVars[]
        = {
        { "color0", OWL_FLOAT4, OWL_OFFSETOF(MissProgData,color0)},
        { "color1", OWL_FLOAT4, OWL_OFFSETOF(MissProgData,color1)},
        { /* sentinel to mark end of list */ }
    };
    // ----------- create object  ----------------------------
    OWLMissProg missProg
        = owlMissProgCreate(context, module, "miss", sizeof(MissProgData),
            missProgVars, -1);

    // ----------- set variables  ----------------------------
    owlMissProgSet4f(missProg, "color0", owl4f{ .8f,0.8f,0.8f, 1.0f });
    owlMissProgSet4f(missProg, "color1", owl4f{ .8f, .8f, .8f, 1.0f });
    owlMissProgSet4f(missProg, "color0", owl4f{ .0f,0.0f,0.0f, 1.0f });
    owlMissProgSet4f(missProg, "color1", owl4f{ .0f, .0f, .0f, 1.0f });

    // -------------------------------------------------------
    // set up ray gen program
    // -------------------------------------------------------
    OWLVarDecl rayGenVars[] = {
      { "fbPtr",         OWL_RAW_POINTER, OWL_OFFSETOF(RayGenData,fbPtr)},
      // { "fbPtr",         OWL_BUFPTR, OWL_OFFSETOF(RayGenData,fbPtr)},
      { "fbSize",        OWL_INT2,   OWL_OFFSETOF(RayGenData,fbSize)},
      { "world",         OWL_GROUP,  OWL_OFFSETOF(RayGenData,world)},
      { "camera.pos",    OWL_FLOAT3, OWL_OFFSETOF(RayGenData,camera.pos)},
      { "camera.dir_00", OWL_FLOAT3, OWL_OFFSETOF(RayGenData,camera.dir_00)},
      { "camera.dir_du", OWL_FLOAT3, OWL_OFFSETOF(RayGenData,camera.dir_du)},
      { "camera.dir_dv", OWL_FLOAT3, OWL_OFFSETOF(RayGenData,camera.dir_dv)},
      { /* sentinel to mark end of list */ }
    };

    // ----------- create object  ----------------------------
    rayGen
        = owlRayGenCreate(context, module, "simpleRayGen",
            sizeof(RayGenData),
            rayGenVars, -1);
    /* camera and frame buffer get set in resiez() and cameraChanged() */
    owlRayGenSetGroup(rayGen, "world", world);

    // ##################################################################
    // build *SBT* required to trace the groups
    // ##################################################################

    owlBuildPrograms(context);
    owlBuildPipeline(context);
    owlBuildSBT(context);

    
}

void Viewer::render()
{
    static double t_last = getCurrentTime();
    static double t_first = t_last;

    if (sbtDirty) {
      owlBuildSBT(context);
      sbtDirty = false;
    }
    owlRayGenLaunch2D(rayGen, fbSize.x, fbSize.y);
    
    double t_now = getCurrentTime();
    static double avg_t = t_now - t_last;
    // if (t_last >= 0)
    avg_t = 0.8 * avg_t + 0.2 * (t_now - t_last);

    char title[1000];
    sprintf(title, "%.2f FPS", 1.f / avg_t);
    inherited::setTitle(title);

    t_last = t_now;
}


void Viewer::key(char key, const vec2i& where)
{
	if (key == '!')
	{
        std::cout << "scrnshot" << "\n";
        screenShot("screenshot.png");
	}
}



int main(int ac, char** av)
{
    LOG("owl::ng example '" << av[0] << "' starting up");

    Viewer viewer;
    viewer.camera.setOrientation(init_lookFrom,
        init_lookAt,
        init_lookUp,
        owl::viewer::toDegrees(acosf(init_cosFovy)));
    //viewer.enableInspectMode(viewer::OWLViewer::RotateMode::POI, {{0,0,0},{5,5,5}});
    viewer.enableFlyMode();
    // ##################################################################
    // now that everything is ready: launch it ....
    // ##################################################################
    viewer.showAndRun();
}
