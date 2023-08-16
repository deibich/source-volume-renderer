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
#include "owlViewer/OWLViewer.h"

#include "tinyxml2.h"
#include "fitsio.h"
#include <iostream>

#include <map>

#define LOG(message)                                            \
  std::cout << OWL_TERMINAL_BLUE;                               \
  std::cout << "#owl.sample(main): " << message << std::endl;   \
  std::cout << OWL_TERMINAL_DEFAULT;
#define LOG_OK(message)                                         \
  std::cout << OWL_TERMINAL_LIGHT_BLUE;                         \
  std::cout << "#owl.sample(main): " << message << std::endl;   \
  std::cout << OWL_TERMINAL_DEFAULT;

extern "C" char deviceCode_ptx[];

const int NUM_VERTICES = 8;
vec3f vertices[NUM_VERTICES] =
  {
    { -1.f,-1.f,-1.f },
    { +1.f,-1.f,-1.f },
    { -1.f,+1.f,-1.f },
    { +1.f,+1.f,-1.f },
    { -1.f,-1.f,+1.f },
    { +1.f,-1.f,+1.f },
    { -1.f,+1.f,+1.f },
    { +1.f,+1.f,+1.f }
  };

const int NUM_INDICES = 12;
vec3i indices[NUM_INDICES] =
  {
    { 0,1,3 }, { 2,3,0 },
    { 5,7,6 }, { 5,6,4 },
    { 0,4,5 }, { 0,5,1 },
    { 2,3,7 }, { 2,7,6 },
    { 1,5,7 }, { 1,7,3 },
    { 4,0,2 }, { 4,2,6 }
  };

// const vec2i fbSize(800,600);
const vec3f init_lookFrom(0,0,-2.0f);
const vec3f init_lookAt(0.f,0.f,0.f);
const vec3f init_lookUp(0.f,1.f,0.f);
const float init_cosFovy = 0.66f;


OWLParams  lp      { 0 };
struct CmdlineInput {
    std::string volumeFilePath {};
    std::string catalogueFilePath {};
} cmdlineInput;

struct SourceRegionLocation {
  std::map<std::string, int> boxMinMax = {
    {"x_min", -1},
    {"x_max", -1},
    {"y_min", -1},
    {"y_max", -1},
    {"z_min", -1},
    {"z_max", -1}
  };

  box3i toBox3i() {
    return {{boxMinMax["x_min"],boxMinMax["y_min"],boxMinMax["z_min"]}, {boxMinMax["x_max"],boxMinMax["y_max"],boxMinMax["z_max"]}};
  }

  box3f toBox3f() {
    const auto x_min = static_cast<float>(boxMinMax["x_min"]);
    const auto x_max = static_cast<float>(boxMinMax["x_max"]);
    const auto y_min = static_cast<float>(boxMinMax["y_min"]);
    const auto y_max = static_cast<float>(boxMinMax["y_max"]);
    const auto z_min = static_cast<float>(boxMinMax["z_min"]);
    const auto z_max = static_cast<float>(boxMinMax["z_max"]);
    return {{x_min, y_min, z_min}, {x_max,y_max,z_max}};
  }
};

struct Viewer : public owl::viewer::OWLViewer
{
  Viewer();

  /*! gets called whenever the viewer needs us to re-render out widget */
  void render() override;

      /*! window notifies us that we got resized. We HAVE to override
          this to know our actual render dimensions, and get pointer
          to the device frame buffer that the viewer cated for us */
  void resize(const vec2i &newSize) override;

  /*! this function gets called whenever any camera manipulator
    updates the camera. gets called AFTER all values have been updated */
  void cameraChanged() override;

  bool sbtDirty = true;
  OWLRayGen rayGen   { 0 };
  OWLContext context { 0 };
  OWLGroup world;
  OWLGroup triangleWorld;
  OWLGroup _aabbWorld;
  double t0 = getCurrentTime();
};

/*! window notifies us that we got resized */
void Viewer::resize(const vec2i &newSize)
{
  OWLViewer::resize(newSize);
  owlRayGenSet1ul   (rayGen,"fbPtr",        (uint64_t)fbPointer);
  owlRayGenSet2i    (rayGen,"fbSize",       (const owl2i&)fbSize);
  sbtDirty = true;
  cameraChanged();
}

/*! window notifies us that the camera has changed */
void Viewer::cameraChanged()
{
  const vec3f lookFrom = camera.getFrom();
  const vec3f lookAt = camera.getAt();
  const vec3f lookUp = camera.getUp();
  const float cosFovy = camera.getCosFovy();
  // ----------- compute variable values  ------------------
  vec3f camera_pos = lookFrom;
  vec3f camera_d00
    = normalize(lookAt-lookFrom);
  float aspect = fbSize.x / float(fbSize.y);
  vec3f camera_ddu
    = cosFovy * aspect * normalize(cross(camera_d00,lookUp));
  vec3f camera_ddv
    = cosFovy * normalize(cross(camera_ddu,camera_d00));
  camera_d00 -= 0.5f * camera_ddu;
  camera_d00 -= 0.5f * camera_ddv;
  
  CameraData cam {
    camera_pos, camera_d00, camera_ddu, camera_ddv
  };
  
  owlParamsSetRaw(lp, "camera", &cam);
}

size_t extractSourceRegionsFromCatalogueXML(const std::string &filePath, std::vector<SourceRegion> &sourceBoxes)
{
    size_t voxels = 0;
    tinyxml2::XMLDocument doc;
    doc.LoadFile(filePath.c_str());

    /*
      <VOTABLE>
        <RESOURCE>
          ...
          ...
          <TABLE>
            <FIELD name="fieldname1" datatype="datatype" .... />
            <FIELD name="fieldname2" datatype="datatype" .... />
            ...
            <FIELD name="fieldnameX" datatype="datatype" .... />
            <DATA>
              <TABLEDATA>
                <TR>
                  <TD> value for fieldname1 </TD>
                  <TD> value for fieldname2 </TD>
                  ...
                  <TD> value for fieldnameX </TD>
                </TR>
                <TR>
                  ...
                </TR>
                  ...
                <TR>
                </TR>
              </TABLEDATA>
            </DATA>

          </TABLE>
        </RESOURCE>
      </VOTABLE>
    */
    auto tableElement = doc.FirstChildElement("VOTABLE")->FirstChildElement("RESOURCE")->FirstChildElement("TABLE");
    auto tableChildNameEntrys = tableElement->FirstChild();
   
    auto dataEntry = tableElement->LastChildElement("DATA")->FirstChildElement("TABLEDATA")->FirstChild();
    
    for (; dataEntry != nullptr; dataEntry = dataEntry->NextSibling())
    {
      SourceRegionLocation srl{};
      auto tdEntry = dataEntry->FirstChild();
      for (auto currTableChildNameEntry = tableChildNameEntrys; currTableChildNameEntry != nullptr; currTableChildNameEntry = currTableChildNameEntry->NextSibling())
      {
          auto tableChildElement = currTableChildNameEntry->ToElement();
          
          if (tableChildElement == nullptr || !tableChildElement->NoChildren())
          {
              break;
          }
          std::string attributeName = tableChildElement->Attribute("name");
          if(!attributeName.empty() &&  srl.boxMinMax.contains(attributeName))
          {
            auto entryValue = tdEntry->ToElement()->IntText();
            srl.boxMinMax[attributeName] = entryValue;
          }
          tdEntry = tdEntry->NextSibling();
      }
        SourceRegion sr;
        sr.gridSourceBox = srl.toBox3i();
        sr.bufferOffset = voxels;
        voxels += sr.gridSourceBox.volume();
        sr.sourceBoxNormalized = srl.toBox3f();
        sourceBoxes.push_back(sr);
    }

    return voxels;
}

vec3i loadDataForSources(const std::string &filePath, std::vector<SourceRegion> &sourceBoxes, std::vector<float> &dataBuffer)
{
  long firstPx[3];
  long lastPx[3];
  long inc[3] = {1,1,1};
  size_t nextDataIdx = 0;
  
  fitsfile *fitsFile;
  int fitsError;
  ffopen(&fitsFile, filePath.c_str(), READONLY, &fitsError);
  assert(fitsError == 0);

  int axisCount;
  int imgType;
  long axis[3];
  fits_get_img_param(fitsFile, 3, &imgType, &axisCount, &axis[0], &fitsError);
  assert(fitsError == 0);
	assert(axisCount == 3);
  assert(imgType == FLOAT_IMG);

  
	const box3i dataCubeVolume({ 0,0,0 }, { axis[0]-1, axis[1]-1, axis[2]-1});
  
  float nan = NAN;

  for (auto & sourceBox : sourceBoxes)
  {
    sourceBox.bufferOffset = nextDataIdx;
    firstPx[0] = sourceBox.gridSourceBox.lower.x + 1;
    firstPx[1] = sourceBox.gridSourceBox.lower.y + 1;
    firstPx[2] = sourceBox.gridSourceBox.lower.z + 1;

    lastPx[0] = sourceBox.gridSourceBox.upper.x;
    lastPx[1] = sourceBox.gridSourceBox.upper.y;
    lastPx[2] = sourceBox.gridSourceBox.upper.z;

    fits_read_subset(fitsFile, TFLOAT, firstPx, lastPx, inc, &nan, dataBuffer.data() + nextDataIdx, 0, &fitsError);
    assert(fitsError == 0);
    nextDataIdx += sourceBox.gridSourceBox.volume();
  }
    
  ffclos(fitsFile, &fitsError);
  return {axis[0], axis[1], axis[2]};
}



Viewer::Viewer()
{

  // load data
  std::vector<SourceRegion> sourceRegions;
  std::vector<float> dataBuffer;
  
  const auto voxelCount = extractSourceRegionsFromCatalogueXML(cmdlineInput.catalogueFilePath, sourceRegions);
  dataBuffer.resize(voxelCount);
  vec3i volumeSize = loadDataForSources(cmdlineInput.volumeFilePath, sourceRegions, dataBuffer);

  // create a context on the first device:
  context = owlContextCreate(nullptr,1);
  OWLModule module = owlModuleCreate(context,deviceCode_ptx);

  // ##################################################################
  // set up all the *GEOMETRY* graph we want to render
  // ##################################################################
  
  // -------------------------------------------------------
  // declare geometry type
  // -------------------------------------------------------
  OWLVarDecl trianglesGeomVars[] = {
    { "index",  OWL_BUFPTR, OWL_OFFSETOF(TrianglesGeomData,index)},
    { "vertex", OWL_BUFPTR, OWL_OFFSETOF(TrianglesGeomData,vertex)},
    { "color",  OWL_FLOAT3, OWL_OFFSETOF(TrianglesGeomData,color)}
  };
  OWLGeomType trianglesGeomType
    = owlGeomTypeCreate(context,
                        OWL_TRIANGLES,
                        sizeof(TrianglesGeomData),
                        trianglesGeomVars,3);
  owlGeomTypeSetClosestHit(trianglesGeomType,0,
                           module,"TriangleMesh");

  // ##################################################################
  // set up all the *GEOMS* we want to run that code on
  // ##################################################################

  LOG("building geometries ...");

  // ------------------------------------------------------------------
  // triangle mesh
  // ------------------------------------------------------------------
  OWLBuffer vertexBuffer
    = owlDeviceBufferCreate(context,OWL_FLOAT3,NUM_VERTICES,vertices);
  OWLBuffer indexBuffer
    = owlDeviceBufferCreate(context,OWL_INT3,NUM_INDICES,indices);
  // OWLBuffer frameBuffer
  //   = owlHostPinnedBufferCreate(context,OWL_INT,fbSize.x*fbSize.y);

  OWLGeom trianglesGeom
    = owlGeomCreate(context,trianglesGeomType);

  owlTrianglesSetVertices(trianglesGeom,vertexBuffer,
                          NUM_VERTICES,sizeof(vec3f),0);
  owlTrianglesSetIndices(trianglesGeom,indexBuffer,
                         NUM_INDICES,sizeof(vec3i),0);

  owlGeomSetBuffer(trianglesGeom,"vertex",vertexBuffer);
  owlGeomSetBuffer(trianglesGeom,"index",indexBuffer);
  owlGeomSet3f(trianglesGeom,"color",owl3f{0,1,0});

  // ------------------------------------------------------------------
  // the group/accel for that mesh
  // ------------------------------------------------------------------
  OWLGroup trianglesGroups[2];
  trianglesGroups[0]
    = owlTrianglesGeomGroupCreate(context,1,&trianglesGeom);
  owlGroupBuildAccel(trianglesGroups[0]);

  trianglesGroups[1]
    = owlTrianglesGeomGroupCreate(context,1,&trianglesGeom);
  owlGroupBuildAccel(trianglesGroups[1]);


  triangleWorld = owlInstanceGroupCreate(context,2,&trianglesGroups[0], nullptr, nullptr, OWL_MATRIX_FORMAT_OWL, OPTIX_BUILD_FLAG_ALLOW_UPDATE);
  owlGroupBuildAccel(triangleWorld);

  

  // ##################################################################
  // set miss and raygen program required for SBT
  // ##################################################################

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  OWLVarDecl missProgVars[]
    = {
    { "color0", OWL_FLOAT3, OWL_OFFSETOF(MissProgData,color0)},
    { "color1", OWL_FLOAT3, OWL_OFFSETOF(MissProgData,color1)},
    { /* sentinel to mark end of list */ }
  };
  // ----------- create object  ----------------------------
  OWLMissProg missProg
    = owlMissProgCreate(context,module,"miss",sizeof(MissProgData),
                        missProgVars,-1);

  // ----------- set variables  ----------------------------
  owlMissProgSet3f(missProg,"color0",owl3f{.8f,0.f,0.f});
  owlMissProgSet3f(missProg,"color1",owl3f{.8f,.8f,.8f});


  OWLVarDecl launchParamsVarsWithStruct[] = {
    { "camera", OWL_USER_TYPE(CameraData), OWL_OFFSETOF(LaunchParams, camera)},
    {nullptr/* sentinel to mark end of list */}
  };

  lp = owlParamsCreate(context,sizeof(LaunchParams),launchParamsVarsWithStruct,-1);

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------
  OWLVarDecl rayGenVars[] = {
    { "fbPtr",         OWL_RAW_POINTER, OWL_OFFSETOF(RayGenData,fbPtr)},
    // { "fbPtr",         OWL_BUFPTR, OWL_OFFSETOF(RayGenData,fbPtr)},
    { "fbSize",        OWL_INT2,   OWL_OFFSETOF(RayGenData,fbSize)},
    { "world",         OWL_GROUP,  OWL_OFFSETOF(RayGenData,world)},
    { /* sentinel to mark end of list */ }
  };


  // Create AABB 
    
  OWLVarDecl aabbGeomsVar[] = {
    {"gridData", OWL_BUFPTR, OWL_OFFSETOF(DatacubeSources, gridData)},
    {"sourceRegions", OWL_BUFPTR, OWL_OFFSETOF(DatacubeSources, sourceRegions)},
    {"sourceCount", OWL_INT, OWL_OFFSETOF(DatacubeSources, sourceCount)},
    {"gridDims", OWL_INT3, OWL_OFFSETOF(DatacubeSources, gridDims)},
    {"minmax", OWL_FLOAT2, OWL_OFFSETOF(DatacubeSources, minmax)},
    { /* sentinel to mark end of list */ }
  };

  OWLGeomType aabbGeomType = owlGeomTypeCreate(context, OWL_GEOM_USER, sizeof(DatacubeSources), aabbGeomsVar, -1);

  // Programs for user geom must be set explicit
  owlGeomTypeSetBoundsProg(aabbGeomType, module, "AABBGeom");
  owlGeomTypeSetIntersectProg(aabbGeomType, 0, module, "AABBGeom");
  owlGeomTypeSetClosestHit(aabbGeomType, 0, module, "AABBGeom");
  //owlGeomTypeSetAnyHit(aabbGeomType, 0, module, "AABBGeom");
  owlBuildPrograms(context);


  for(auto &region : sourceRegions)
  {
    auto lower = region.sourceBoxNormalized.lower / vec3f(volumeSize);
    auto upper = region.sourceBoxNormalized.upper / vec3f(volumeSize);
    // lower -= vec3f{0.5f};
    //upper -= vec3f{0.5f};
    region.sourceBoxNormalized = {lower, upper};
  }

  OWLBuffer owlSourcesDataBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(SourceRegion), sourceRegions.size(), sourceRegions.data());
  const OWLBuffer owlDataBuffer = owlDeviceBufferCreate(context, OWL_FLOAT, dataBuffer.size(), dataBuffer.data());
  const owl2f gridDataMinMax = 
  {
    *std::min_element(dataBuffer.begin(), dataBuffer.end()),
    *std::max_element(dataBuffer.begin(), dataBuffer.end())
  };

  OWLGeom aabbGeom = owlGeomCreate(context, aabbGeomType);

  owlGeomSetBuffer(aabbGeom, "gridData", owlDataBuffer);
  owlGeomSetBuffer(aabbGeom, "sourceRegions", owlSourcesDataBuffer);
  owlGeomSet1i(aabbGeom, "sourceCount", sourceRegions.size());
  owlGeomSet3i(aabbGeom, "gridDims", {volumeSize.x, volumeSize.y, volumeSize.z});
  owlGeomSet2f(aabbGeom, "minmax", gridDataMinMax);


  owlGeomSetPrimCount(aabbGeom, sourceRegions.size());

  OWLGroup aabbGroups[1];
  aabbGroups[0] = owlUserGeomGroupCreate(context, 1, &aabbGeom);
  owlGroupBuildAccel(aabbGroups[0]);
  
  _aabbWorld = owlInstanceGroupCreate(context, 1, &aabbGroups[0], nullptr, nullptr, OWL_MATRIX_FORMAT_OWL, OPTIX_BUILD_FLAG_ALLOW_UPDATE);
  

  owlGroupBuildAccel(_aabbWorld);
  world = _aabbWorld;

  //world = triangleWorld;
  // ----------- create object  ----------------------------
  rayGen
    = owlRayGenCreate(context,module,"simpleRayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);
  /* camera and frame buffer get set in resiez() and cameraChanged() */
  owlRayGenSetGroup (rayGen,"world",        world);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################

  owlBuildPrograms(context);
  owlBuildPipeline(context);
  owlBuildSBT(context);
}


void Viewer::render()
{

  auto deltaTime = getCurrentTime() - t0;

  if (sbtDirty) {
    owlBuildSBT(context);
    sbtDirty = false;
  }
  owlLaunch2D(rayGen,fbSize.x,fbSize.y,lp);
  cudaDeviceSynchronize();
}


int main(int argc, char **argv)
{
  LOG("owl::ng example '" << argv[0] << "' starting up");

  for (int i=1;i<argc;i++) {
        const std::string arg = argv[i];
        if (arg == "-vol")
        {
            cmdlineInput.volumeFilePath = argv[++i];
        }
        else if (arg == "-cat")
        {
            cmdlineInput.catalogueFilePath = argv[++i];
        } 
        else
        {
            std::cout << "unknown cmdline arg '" << arg << "'" << std::endl;
        }
    }
    
    assert(!cmdlineInput.volumeFilePath.empty(), "No Volume file given");
    assert(!cmdlineInput.catalogueFilePath.empty(), "No catalogue file given");
    


  Viewer viewer;
  viewer.camera.setOrientation(init_lookFrom,
                               init_lookAt,
                               init_lookUp,
                               owl::viewer::toDegrees(acosf(init_cosFovy)));
  viewer.enableFlyMode();
  // viewer.enableInspectMode(owl::box3f(vec3f(-1.f),vec3f(+1.f)));

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  viewer.showAndRun();
}
