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
#include <vector_functions.h>
#include <cuda_runtime.h>
#include <optix.h>
#include "owl/owl_device.h"
#include "owl/common/math/vec.h"
#include "owl/common/math/box.h"
#include <owl/common/math/LinearSpace.h>
#include "owl/owl.h"
#include "owl/helper/cuda.h"
#include <owl/owl.h>
#include <owl/common/math/AffineSpace.h>
#include <owl/common/math/random.h>
using namespace owl;

struct PerRayData
{
    vec3f invDir;
    vec3i sign;

    float hitCount;
    float maxVal;
    float maxT;
    vec4f color;
};

constexpr float EPS = 1.e-4f;
__constant__ int GRID = 8;

namespace
{
	template <typename T>
	__device__ T smoothstep(T h0, T h1, T x)
	{
	    T t = clamp((x - h0) / (h1 - h0), T{ 0.0f }, T{ 1.0f });
	    return t * t * (T{ 3.0f } - 2.0f * t);
	}

	template<typename T>
	inline __both__ T length(const vec_t<T, 2>& v)
	{
	    return owl::common::polymorphic::sqrt(dot(v, v));
	}

    inline __device__ vec3b lte(vec3f a, vec3f b)
    {

        { return { a.x <= b.x, a.y <= b.y, a.z <= b.z }; }
    }

    inline __device__ vec3i ltei(vec3f a, vec3f b)
    { return { a.x <= b.x ? 1 : 0, a.y <= b.y ? 1 : 0, a.z <= b.z ? 1 : 0 }; }

	inline __device__ vec3i sign(vec3f v)
	{
	    return vec3i
	    {
	        v.x < 0 ? -1 : v.x > 0 ? 1: 0,
	        v.y < 0 ? -1 : v.y > 0 ? 1 : 0,
	        v.z < 0 ? -1 : v.z > 0 ? 1 : 0
	    };
	}

	// --- MATLAB Jet Colour Scheme ----------------------------------------
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
	// --- MATLAB Jet Colour Scheme ----------------------------------------

    __both__
        float fracf(float x)
    {
        return x - floorf(x);
    }

    __both__
        vec3f fracf(vec3f v)
    {
        return
        {
            v.x - floorf(v.x),
            v.y - floorf(v.y),
            v.z - floorf(v.z),
        };
    }

    __device__ vec3f mix(vec3f x, vec3f y, float s)
    {
        return x + s*(y - x);
    }

    __device__ float aabb_outline(vec3f pos, box3i box)
    {
        vec3f bcen = 0.5f * (vec3f{ box.lower } + vec3f{ box.upper });
        vec3f brad = 0.5f * (vec3f{ box.upper } - vec3f{ box.lower });
        vec3f e = smoothstep(brad - 1.f, brad - 0.01f, abs(pos - bcen));
        return 1.0f - (1.0f - e.x * e.y) * (1.0f - e.y * e.z) * (1.0f - e.z * e.x);
    }
}

OPTIX_RAYGEN_PROGRAM(simpleRayGen)()
{
    const RayGenData& self = owl::getProgramData<RayGenData>();
    const vec2i pixelID = owl::getLaunchIndex();

    const vec2f screen = (vec2f(pixelID) + vec2f(.5f)) / vec2f(self.fbSize);

    owl::Ray ray;
    ray.origin
        = self.camera.pos;
    ray.direction
        = normalize(self.camera.dir_00
            + screen.u * self.camera.dir_du
            + screen.v * self.camera.dir_dv);
    PerRayData prd;
    prd.invDir = 1.0f / ray.direction;
    prd.hitCount = 0;
    prd.maxVal = -1.f;

    owl::traceRay(/*accel to trace against*/self.world,
        /*the ray to trace*/ray,
        /*prd*/prd);

    // Ray traversal finished
    const int framebufferOffset = pixelID.x + self.fbSize.x * pixelID.y;
    self.fbPtr[framebufferOffset] = owl::make_rgba(prd.color);
}   

OPTIX_BOUNDS_PROGRAM(VolumeGrid)(const void* geomData, box3f& primBounds, const int primID)
{
    const VolumeGridData& self = *(const VolumeGridData*)geomData;
    vec3f lower = { 0,0,0 };
    primBounds = { vec3f{self.sourceRegions[primID].sourceBox.lower } / vec3f{self.gridDims}, vec3f{self.sourceRegions[primID].sourceBox.upper + vec3i{1,1,1}} / vec3f{self.gridDims} };
}

OPTIX_INTERSECT_PROGRAM(VolumeGrid)()
{
    const int primID = optixGetPrimitiveIndex();
    const VolumeGridData& self = owl::getProgramData<VolumeGridData>();

    vec3f boxMin = vec3f{ self.sourceRegions[primID].sourceBox.lower } / vec3f(self.gridDims);
    vec3f boxMax = vec3f{ self.sourceRegions[primID].sourceBox.upper + vec3i{1,1,1} } / vec3f(self.gridDims);

    PerRayData& prd = owl::getPRD<PerRayData>();
    vec3f rayOrig = optixGetWorldRayOrigin();

    vec3f t1 = (boxMin - rayOrig) * prd.invDir;
    vec3f t2 = (boxMax - rayOrig) * prd.invDir;

    vec3f tMin = min(t1, t2);
    vec3f tMax = max(t1, t2);

    float tNear = max(max(tMin.x, tMin.y), tMin.z);
    float tFar = min(min(tMax.x, tMax.y), tMax.z);


    if (tFar >= tNear && !(tNear < 0 && tFar < 0))
    {
        float tFarReport = max(tNear, tFar);
        float tNearReport = max(0.0f, min(tNear, tFar));
        // Store both points for CH and AH program
        optixReportIntersection(tFar, 0, __float_as_int(tNear));
    }
}


OPTIX_ANY_HIT_PROGRAM(VolumeGrid)()
{
    const VolumeGridData& self = owl::getProgramData<VolumeGridData>();
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
    const vec3f localGridPosEntryF = worldGridPosEntryF - vec3f{ self.sourceRegions[primId].sourceBox.lower };
    const vec3f localGridPosExitF = worldGridPosExitF - vec3f{ self.sourceRegions[primId].sourceBox.lower };
    const vec3f localRayDirection = normalize(localGridPosExitF - localGridPosEntryF);

    // Box to test current position
    const box3i oaabb = box3i{ {0,0,0}, self.sourceRegions[primId].sourceBox.size() };

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
    int itCount = ceilf(dot(vec3f{ self.sourceRegions[primId].sourceBox.size() }, vec3f{ 1.0f, 1.0f, 1.0f }));
    float inc = 1.0f / itCount;

    float maxVal = 0;
    float currVal = 0;
    auto bufferOffset = self.sourceRegions[primId].bufferOffset;

    // The incremental phase ("branchless" optimized) from: https://www.shadertoy.com/view/4dX3zl
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
        mask = ltei(tMax, min(tMax.yzx(), vec3f(tMax.z, tMax.x, tMax.y)));
        tMax += vec3f(mask) * tDelta;
        localGridPosI += mask * step;
    }
    prd.maxVal = max(prd.maxVal, maxVal);

    vec3f mainColor = spectral_jet(prd.maxVal / self.minmax.y);
    vec3f bgColor = { .8f, .8f, .8f };
    bgColor = { .0f, .0f, .0f };
    vec3f borderColor = { 0,0,0 };
    float borderAlpha = aabb_outline(localGridPosEntryF, oaabb);

    vec3f contentColor = mix(bgColor, mainColor, prd.maxVal / self.minmax.y);
    //prd.color = { contentColor,1 };
    // prd.color = { mix(contentColor , borderColor,borderAlpha) , 1 };
    //prd.color = { mainColor,1 };
    // prd.color = { spectral_jet(prd.hitCount),1 };
    prd.color = { contentColor, 1.0f };
    optixIgnoreIntersection();
}

OPTIX_CLOSEST_HIT_PROGRAM(VolumeGrid)()
{
    
}

OPTIX_MISS_PROGRAM(miss)()
{
    const vec2i pixelID = owl::getLaunchIndex();
    PerRayData &prd = owl::getPRD<PerRayData>();
    //const MissProgData &self = owl::getProgramData<MissProgData>();

    if(prd.hitCount < EPS)
    {
        prd.color = vec4f(.0f);
    }
}
