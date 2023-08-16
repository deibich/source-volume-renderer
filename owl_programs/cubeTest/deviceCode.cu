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

#include "deviceCode.h"
#include <optix_device.h>
#include "owl/common/math/vec.h"

extern "C" __constant__ LaunchParams optixLaunchParams;

__constant__ float EPS = .0001f;

#define PRINT_VEC3F(vec) \
  printf("x: %.20f, y: %.20f, y: %.20f\n", vec.x, vec.y, vec.z);

#define PRINT_VEC3I(vec) \
  printf("x: %d, y: %d, y: %d\n", vec.x, vec.y, vec.z);

namespace
{
  inline __device__ vec3i sign(vec3f v)
  {
    return vec3i
    {
      v.x < 0 ? -1 : v.x > 0 ? 1: 0,
      v.y < 0 ? -1 : v.y > 0 ? 1 : 0,
      v.z < 0 ? -1 : v.z > 0 ? 1 : 0
    };
  }

  inline __device__ vec3b lte(vec3f a, vec3f b)
    {

        { return { a.x <= b.x, a.y <= b.y, a.z <= b.z }; }
    }

    inline __device__ vec3i ltei(vec3f a, vec3f b)
    { return { a.x <= b.x ? 1 : 0, a.y <= b.y ? 1 : 0, a.z <= b.z ? 1 : 0 }; }

  __device__ vec3f mix(vec3f x, vec3f y, float s)
{
    return x + s*(y - x);
}
inline __device__
	vec3f spectral_jet(float x)
	{
	    vec3f c;
	    if (x < 0.25)
	        c = vec3f(0.0, 4.0 * x, 1.0);
	    else if (x < 0.5)
	        c = vec3f(0.0, 1.0, 1.0 + 4.0 * (0.25 - x));
	    else if (x < 0.75)
	        c = vec3f(4.0 * (x - 0.5), 1.0, 0.0);
	    else
	        c = vec3f(1.0, 1.0 + 4.0 * (0.75 - x), 0.0);
	    return clamp(c, vec3f(0.0), vec3f(1.0));
	}
}

OPTIX_RAYGEN_PROGRAM(simpleRayGen)
()
{
  const RayGenData &self = owl::getProgramData<RayGenData>();
  const vec2i pixelID = owl::getLaunchIndex();
  
  const vec2f screen = (vec2f(pixelID) + vec2f(.5f)) / vec2f(self.fbSize);
  owl::Ray ray;
  ray.origin = optixLaunchParams.camera.pos;
  
  ray.direction = normalize(optixLaunchParams.camera.dir_00 + screen.u * optixLaunchParams.camera.dir_du + screen.v * optixLaunchParams.camera.dir_dv);

  PerRayData perRayData{};
  perRayData.fbSize = self.fbSize;
  perRayData.hitCount = 0;
  perRayData.maxVal = -CUDART_INF_F;
  owl::traceRay(/*accel to trace against*/ self.world,
                /*the ray to trace*/ ray,
                /*prd*/ perRayData);

  const int fbOfs = pixelID.x + self.fbSize.x * pixelID.y;
  self.fbPtr[fbOfs] = owl::make_rgba(perRayData.color);
}

OPTIX_CLOSEST_HIT_PROGRAM(TriangleMesh)
()
{
  PerRayData &prd = owl::getPRD<PerRayData>();

  const TrianglesGeomData &self = owl::getProgramData<TrianglesGeomData>();

  // compute normal:
  const int primID = optixGetPrimitiveIndex();
  const vec3i index = self.index[primID];
  const vec3f &A = self.vertex[index.x];
  const vec3f &B = self.vertex[index.y];
  const vec3f &C = self.vertex[index.z];
  const vec3f Ng = normalize(cross(B - A, C - A));

  const vec3f rayDir = optixGetWorldRayDirection();
  prd.color = (.2f + .8f * fabs(dot(rayDir, Ng))) * self.color;
}

OPTIX_MISS_PROGRAM(miss)
()
{

  const vec2i pixelID = owl::getLaunchIndex();

  const MissProgData &self = owl::getProgramData<MissProgData>();

  PerRayData &prd = owl::getPRD<PerRayData>();
  int pattern = (pixelID.x / 8) ^ (pixelID.y / 8);

if(prd.hitCount > EPS)
	{
	}
    else
    {
        prd.color = {.8f, .8f, .8f};
    }
}

OPTIX_BOUNDS_PROGRAM(AABBGeom)
(const void *geomData, box3f &primBounds, const int primID)
{
  const DatacubeSources &self = *(const DatacubeSources *)geomData;
  auto lower = self.sourceRegions[primID].sourceBoxNormalized.lower;
  auto upper = self.sourceRegions[primID].sourceBoxNormalized.upper;

  PRINT_VEC3F(lower)
  PRINT_VEC3F(upper)
  primBounds = self.sourceRegions[primID].sourceBoxNormalized;
}

OPTIX_INTERSECT_PROGRAM(AABBGeom)
()
{
  const auto primID = optixGetPrimitiveIndex();
  const auto &datacubeSources = owl::getProgramData<DatacubeSources>();

  const vec3f rayOriginObj = optixGetObjectRayOrigin();
  vec3f rayDirectionObj = optixGetObjectRayDirection();
  const auto rayLengthInv = 1 / length(rayDirectionObj);
  rayDirectionObj = rayDirectionObj * rayLengthInv;

  auto &prd = owl::getPRD<PerRayData>();
  auto t_lower = (datacubeSources.sourceRegions[primID].sourceBoxNormalized.lower - rayOriginObj) / rayDirectionObj;
  auto t_upper = (datacubeSources.sourceRegions[primID].sourceBoxNormalized.upper - rayOriginObj) / rayDirectionObj;

  const auto tMin = min(t_lower, t_upper);
  const auto tMax = max(t_lower, t_upper);

  const auto tNear = max(max(tMin.x, tMin.y), tMin.z);
  const auto tFar = min(min(tMax.x, tMax.y), tMax.z);
  
  if (tNear < tFar && tFar > 0)
  {
    optixReportIntersection(tNear * rayLengthInv, 0);
  }
  /*
  if (tFar >= tNear && !(tNear < 0 && tFar < 0))
  {
    float tFarReport = max(tNear, tFar);
    float tNearReport = max(0.0f, min(tNear, tFar));
    // Store both points for CH and AH program
    optixReportIntersection(tFar, 0, __float_as_int(tNear));
  }
  */
}

OPTIX_CLOSEST_HIT_PROGRAM(AABBGeom)
()
{
  auto &prd = owl::getPRD<PerRayData>();
  const vec3f rayOrigin = optixGetWorldRayOrigin();
  const vec3f rayDir = optixGetWorldRayDirection();
  
  vec3f hitPosition = vec3f{rayOrigin + optixGetRayTmax() * rayDir};
  prd.color = optixTransformPointFromWorldToObjectSpace(hitPosition);
  prd.color += vec3f{0.5f};
  // prd.color = vec3f{optixGetPrimitiveIndex()} / 300;
}


OPTIX_ANY_HIT_PROGRAM(AABBGeom)
()
{
  return;
  const DatacubeSources& self = owl::getProgramData<DatacubeSources>();
  PerRayData& prd = owl::getPRD<PerRayData>();

  float tFar = optixGetRayTmax();
  float tNear = __int_as_float(optixGetAttribute_0());
  auto primId = optixGetPrimitiveIndex();
  
  tNear = max(0.0f, min(tNear, tFar));
  vec3f rayDirection = optixGetWorldRayDirection();
  vec3f rayOrigin = optixGetWorldRayOrigin();

  vec3f worldPosEntry = rayOrigin + rayDirection * (tNear + EPS);
  vec3f worldPosExit = rayOrigin + rayDirection * (tFar + EPS);

  
  // Current position to position in grid coordinates
  vec3f worldGridPosEntryF = worldPosEntry * vec3f{ self.gridDims };
  vec3f worldGridPosExitF = worldPosExit * vec3f{ self.gridDims };
  
  // Change ray to local coordinates 0 to size of the box
  const vec3f localGridPosEntryF = worldGridPosEntryF - vec3f{ self.sourceRegions[primId].gridSourceBox.lower };
  const vec3f localGridPosExitF = worldGridPosExitF - vec3f{ self.sourceRegions[primId].gridSourceBox.lower };
  const vec3f localRayDirection = normalize(localGridPosExitF - localGridPosEntryF);


  // Box to test current position
  const box3i oaabb = box3i{ {0,0,0}, self.sourceRegions[primId].gridSourceBox.size() };

  // The traversal algorithm consists of two phases : initialization and incremental traversal
  // The initialization phase begins by identifying the voxel in which the ray origin, ->u, is found.
  vec3i localGridPosI = vec3i{ localGridPosEntryF };
  
  // In addition, the variables stepX and stepY are initialized to either 1 or -1 indicating whether X and Y are incremented or decremented as the ray crosses voxel boundaries(this is determined by the sign of the x and y components of ->v).
  vec3i step = sign(localRayDirection);
  vec3f stepF = vec3f{ step };

  // TDeltaX indicates how far along the ray we must move
  // (in units of t) for the horizontal component of such a movement to equal the width of a voxel.Similarly,
  // we store in tDeltaY the amount of movement along the ray which has a vertical component equal to the
  // height of a voxel.
  vec3f tDelta = abs(vec3f(length(localRayDirection)) / localRayDirection);

  // Next, we determine the value of t at which the ray crosses the first vertical
  // voxel boundary and store it in variable tMaxX.We perform a similar computation
  // in y and store the result in tMaxY.
  // The minimum of these two values will indicate how much we can travel along the
  // ray and still remain in the current voxel.
  vec3f tMax = (stepF * (vec3f(localGridPosI) - localGridPosEntryF) + (stepF * 0.5f) + 0.5f) * tDelta;

  // Increment for hitcount heatmap
  int itCount = ceilf(dot(vec3f{ self.sourceRegions[primId].gridSourceBox.size() }, vec3f{ 1.0f, 1.0f, 1.0f }));
  float inc = 1.0f / itCount;

  float maxVal = 0;
  float currVal = 0;
  auto bufferOffset = self.sourceRegions[primId].bufferOffset;

  // The incremental phase ("branchless" optiized) from: https://www.shadertoy.com/view/4dX3zl
  vec3i mask;
  for (int i = 0; i < itCount; i++) {
    if (oaabb.contains(localGridPosI))
    {
      currVal = self.gridData[bufferOffset + localGridPosI.x + localGridPosI.y * oaabb.upper.x + localGridPosI.z * oaabb.upper.x * oaabb.upper.y].x;
      maxVal = max(currVal, maxVal);
      prd.hitCount += inc;
    }
    else
    {
      break;
    }
    mask = ltei(tMax, min(tMax.yzx(), vec3f{tMax.z, tMax.x, tMax.y}));
    tMax += vec3f(mask) * tDelta;
    localGridPosI += mask * step;
  }
  prd.maxVal = max(prd.maxVal, maxVal);
  vec3f bgColor = { .8f, .8f, .8f };
  bgColor = { .8f, .8f, .8f };
    
  vec3f mainColor = spectral_jet(prd.maxVal / self.minmax.y);
  
  vec3f contentColor = mix(bgColor, mainColor, prd.maxVal / self.minmax.y);
  prd.color = contentColor ;
  optixIgnoreIntersection();
}