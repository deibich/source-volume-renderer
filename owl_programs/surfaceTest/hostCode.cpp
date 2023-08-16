// ======================================================================== //
// Copyright 2019 Ingo Wald                                                 //
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
// external helper stuff for image output
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#ifdef WIN32
#include <windows.h>

#endif
#include <vector>
#include <cuda_gl_interop.h>
#include "glm/glm.hpp"
#include "glm/matrix.hpp"
#include "owl/common/math/AffineSpace.h"
#include "glm/ext.hpp"

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

const char *outFileName = "s01-simpleTriangles.png";
const char *outFileName1 = "s01-simpleTriangles1.png";
const vec2i fbSize(800,600);
const vec3f lookFrom(-4.f,-3.f,-2.f);
const vec3f lookAt(0.f,0.f,0.f);
const vec3f lookUp(0.f,1.f,0.f);
const float cosFovy = 0.66f;

OWLParams  lp      { 0 };

GLFWwindow* offscreen_context;
GLuint*   fbTexture  {0};
cudaGraphicsResource_t * cuDisplayTexture { 0 };

cudaSurfaceObject_t *cuSurfaceObj;

int main(int ac, char **av)
{
  fbTexture = new GLuint[2];
  cuSurfaceObj = new cudaSurfaceObject_t[2];
  cuDisplayTexture = new cudaGraphicsResource_t[2];
  LOG("owl::ng example '" << av[0] << "' starting up");

  // create a context on the first device:
  OWLContext context = owlContextCreate(nullptr,1);
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
  OWLBuffer frameBuffer
    = owlHostPinnedBufferCreate(context,OWL_INT,fbSize.x*fbSize.y);
  
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
  OWLGroup trianglesGroup
    = owlTrianglesGeomGroupCreate(context,1,&trianglesGeom);
  owlGroupBuildAccel(trianglesGroup);

  OWLGroup world
    = owlInstanceGroupCreate(context,1,&trianglesGroup,nullptr, nullptr, OWL_MATRIX_FORMAT_OWL, OPTIX_BUILD_FLAG_ALLOW_UPDATE);

  owlGroupBuildAccel(world);

  

  // ##################################################################
  // set miss and raygen program required for SBT
  // ##################################################################

  // -------------------------------------------------------
  // set up miss prog
  // -------------------------------------------------------
  OWLVarDecl missProgVars[]
    = {
    { "color0", OWL_FLOAT3, OWL_OFFSETOF(MissProgData, color0)},
    { "color1", OWL_FLOAT3, OWL_OFFSETOF(MissProgData, color1)},
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
    { "surfIdx", OWL_INT, OWL_OFFSETOF(LaunchParams, surfIdx)},
    { "camera", OWL_USER_TYPE(CameraData), OWL_OFFSETOF(LaunchParams, camera)},
    {nullptr/* sentinel to mark end of list */}
  };

  OWLVarDecl launchParamsVars[] = {
    { "surfIdx", OWL_INT, OWL_OFFSETOF(LaunchParams, surfIdx)},
    { "camera.pos",    OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.pos)},
    { "camera.dir_00", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_00)},
    { "camera.dir_du", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_du)},
    { "camera.dir_dv", OWL_FLOAT3, OWL_OFFSETOF(LaunchParams,camera.dir_dv)},
    { nullptr/* sentinel to mark end of list */ }
  };

  lp = owlParamsCreate(context,sizeof(LaunchParams),launchParamsVarsWithStruct,-1);

  

  // -------------------------------------------------------
  // set up ray gen program
  // -------------------------------------------------------

  glfwInit();
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  offscreen_context = glfwCreateWindow(fbSize.x, fbSize.y, "", NULL, NULL);
  
  glfwMakeContextCurrent(offscreen_context);
  glGenTextures(2, fbTexture);
  for(int i = 0; i < 2; i++)
  {

    glBindTexture(GL_TEXTURE_2D, fbTexture[i]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, fbSize.x, fbSize.y, 0, GL_RGBA,
                            GL_UNSIGNED_BYTE, nullptr);

    int ret = cudaGraphicsGLRegisterImage(&cuDisplayTexture[i], fbTexture[i], GL_TEXTURE_2D, 0);
    ret = cudaGraphicsMapResources(1, &cuDisplayTexture[i]);
    cudaArray_t arrayPtr;
    ret = cudaGraphicsSubResourceGetMappedArray(&arrayPtr, cuDisplayTexture[i], 0, 0);

    cudaResourceDesc resDesc{};
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeArray;
    resDesc.res.array.array = arrayPtr;
    ret = cudaCreateSurfaceObject(&cuSurfaceObj[i], &resDesc);
   
  }

  OWLBuffer surfaceBuffer = owlDeviceBufferCreate(context, OWL_USER_TYPE(cudaSurfaceObject_t), 2, &cuSurfaceObj[0]);

  OWLVarDecl rayGenVars[] = {
    {"surf", OWL_BUFPTR, OWL_OFFSETOF(RayGenData, surf)},
    // { "surf",          OWL_USER_TYPE(cudaSurfaceObject_t*), OWL_OFFSETOF(RayGenData, surf)},
    { "fbSize",        OWL_INT2,   OWL_OFFSETOF(RayGenData,fbSize)},
    { "world",         OWL_GROUP,  OWL_OFFSETOF(RayGenData,world)},
    { /* sentinel to mark end of list */ }
  };

  // ----------- create object  ----------------------------
  OWLRayGen rayGen
    = owlRayGenCreate(context,module,"simpleRayGen",
                      sizeof(RayGenData),
                      rayGenVars,-1);

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

  // ----------- set variables  ----------------------------
  owlRayGenSetBuffer(rayGen, "surf", surfaceBuffer);
  // owlRayGenSetRaw   (rayGen, "surf", &cuSurfaceObj);
  owlRayGenSet2i    (rayGen,"fbSize",       (const owl2i&)fbSize);
  owlRayGenSetGroup (rayGen,"world",        world);

  CameraData cam {
    camera_pos, camera_d00, camera_ddu, camera_ddv
  };
  owlParamsSet1i(lp, "surfIdx", 0);
  owlParamsSetRaw(lp, "camera", &cam);

  // owlParamsSet3f(lp,"camera.pos",   (const owl3f&)camera_pos);
  // owlParamsSet3f(lp,"camera.dir_00",(const owl3f&)camera_d00);
  // owlParamsSet3f(lp,"camera.dir_du",(const owl3f&)camera_ddu);
  // owlParamsSet3f(lp,"camera.dir_dv",(const owl3f&)camera_ddv);

  // ##################################################################
  // build *SBT* required to trace the groups
  // ##################################################################
  owlBuildPrograms(context);
  owlBuildPipeline(context);
  owlBuildSBT(context);

  const linear3f rot = owl::common::linear3f::rotate(vec3f{.0f,1.f,.0f}, 35.f * (3.141593f/180.f)); 
  
  affine3f transf{rot, vec3f{.2f,.0f,.5f}};
  owlInstanceGroupSetTransform(world, 0, (const float*)&transf);
  
  owlGroupRefitAccel(world);
  auto trans = affine3f::translate(vec3f{1,2,3});
  auto rot1 = affine3f::rotate({1.f,.0f,.0f}, 1.0f);
  auto scale = affine3f::scale(vec3f{1.f,1.f,1.f});

  // ##################################################################
  // now that everything is ready: launch it ....
  // ##################################################################
  LOG("launching ...");
  owlAsyncLaunch2D(rayGen,fbSize.x,fbSize.y, lp);
  owlParamsSet1i(lp, "surfIdx", 1);
  cam.pos.x += 0.4f;
  owlParamsSetRaw(lp, "camera", &cam);
  owlAsyncLaunch2D(rayGen, fbSize.x, fbSize.y, lp);
  owlLaunchSync(lp);
  cudaDeviceSynchronize();

  LOG("done with launch, writing picture ...");
  // for host pinned mem it doesn't matter which device we query...
  const uint32_t *fb
    = (const uint32_t*)owlBufferGetPointer(frameBuffer,0);
  assert(fb);

  std::vector<uint32_t> fbData{};
  fbData.resize(fbSize.x*fbSize.y);

  glBindTexture(GL_TEXTURE_2D, fbTexture[0]);
  glEnable(GL_TEXTURE_2D);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, fbData.data());
  stbi_write_png(outFileName, fbSize.x, fbSize.y, 4, fbData.data(), fbSize.x*sizeof(uint32_t));

  
  glBindTexture(GL_TEXTURE_2D, fbTexture[1]);
  glEnable(GL_TEXTURE_2D);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, fbData.data());
  stbi_write_png(outFileName1, fbSize.x, fbSize.y, 4, fbData.data(), fbSize.x*sizeof(uint32_t));
  
  // stbi_write_png(outFileName,fbSize.x,fbSize.y,4, fb,fbSize.x*sizeof(uint32_t));
  LOG_OK("written rendered frame buffer to file "<<outFileName);
  // ##################################################################
  // and finally, clean up
  // ##################################################################

  LOG("destroying devicegroup ...");
  owlContextDestroy(context);

  LOG_OK("seems all went OK; app is done, this should be the last output ...");
  
  // cudaDestroySurfaceObject(cuSurfaceObj);
  // cudaGraphicsUnregisterResource(cuDisplayTexture);
  glfwDestroyWindow(offscreen_context);
  glfwTerminate();
}
