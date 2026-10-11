// Texture coverage is evaluated inside Embree traversal, without ray restarts.
#include "embree4/rtcore.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>

struct Terrain {
  RTCDevice device = nullptr;
  RTCScene scene = nullptr;
  const float* uv;
  const int32_t* material;
  const uint8_t* alpha;
  const int64_t* offsets;
  const int64_t* widths;
  const int64_t* heights;
  const double* cutoffs;
  ~Terrain() {
    if (scene) rtcReleaseScene(scene);
    if (device) rtcReleaseDevice(device);
  }
};

static int wrap(int index, int size) { return (index % size + size) % size; }

static void coverage(const RTCFilterFunctionNArguments* args) {
  const auto* t = static_cast<const Terrain*>(args->geometryUserPtr);
  for (unsigned i = 0; i < args->N; ++i) {
    if (!args->valid[i]) continue;
    const unsigned face = RTCHitN_primID(args->hit, args->N, i);
    const int mid = t->material[face];
    if (t->cutoffs[mid] == 0) continue;
    const float u = RTCHitN_u(args->hit, args->N, i);
    const float v = RTCHitN_v(args->hit, args->N, i);
    const float* uv = t->uv + face*6;
    const double s = (1.0-u-v)*uv[0] + u*uv[2] + v*uv[4];
    const double r = (1.0-u-v)*uv[1] + u*uv[3] + v*uv[5];
    const int w = t->widths[mid], h = t->heights[mid];
    const double x = (s-std::floor(s))*w-.5;
    const double y = (1-(r-std::floor(r)))*h-.5;
    const int x0 = std::floor(x), y0 = std::floor(y);
    const double fx = x-x0, fy = y-y0;
    const uint8_t* a = t->alpha + t->offsets[mid];
    const double value = (1-fx)*(1-fy)*a[wrap(y0,h)*w+wrap(x0,w)]
      + fx*(1-fy)*a[wrap(y0,h)*w+wrap(x0+1,w)]
      + (1-fx)*fy*a[wrap(y0+1,h)*w+wrap(x0,w)]
      + fx*fy*a[wrap(y0+1,h)*w+wrap(x0+1,w)];
    if (value < t->cutoffs[mid]*255) args->valid[i] = 0;
  }
}

extern "C" void* terrain_create(const float* triangles, size_t count,
    const float* uv, const int32_t* materials, const uint8_t* alpha,
    const int64_t* offsets, const int64_t* widths, const int64_t* heights,
    const double* cutoffs) {
  auto* t = new (std::nothrow) Terrain;
  if (!t) return nullptr;
  t->uv=uv; t->material=materials; t->alpha=alpha; t->offsets=offsets;
  t->widths=widths; t->heights=heights; t->cutoffs=cutoffs;
  t->device = rtcNewDevice("threads=1");
  if (!t->device) { delete t; return nullptr; }
  t->scene = rtcNewScene(t->device);
  if (!t->scene) { delete t; return nullptr; }
  rtcSetSceneFlags(t->scene, RTC_SCENE_FLAG_ROBUST);
  RTCGeometry geometry = rtcNewGeometry(t->device, RTC_GEOMETRY_TYPE_TRIANGLE);
  if (!geometry) { delete t; return nullptr; }
  auto* vertices = static_cast<float*>(rtcSetNewGeometryBuffer(geometry,
      RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), count*3));
  auto* faces = static_cast<uint32_t*>(rtcSetNewGeometryBuffer(geometry,
      RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(uint32_t), count));
  if (!vertices || !faces) { rtcReleaseGeometry(geometry); delete t; return nullptr; }
  std::memcpy(vertices, triangles, count*9*sizeof(float));
  for (size_t i=0; i<count*3; ++i) faces[i]=i;
  rtcSetGeometryUserData(geometry, t);
  rtcSetGeometryIntersectFilterFunction(geometry, coverage);
  rtcCommitGeometry(geometry);
  rtcAttachGeometry(t->scene, geometry);
  rtcReleaseGeometry(geometry);
  rtcCommitScene(t->scene);
  if (rtcGetDeviceError(t->device) != RTC_ERROR_NONE) { delete t; return nullptr; }
  return t;
}

extern "C" void terrain_trace(void* handle, const float* origins,
    const float* directions, size_t count, float max_range, float* distances,
    int32_t* faces) {
  const auto* t = static_cast<const Terrain*>(handle);
  for (size_t i=0; i<count; ++i) {
    RTCRayHit hit{};
    hit.ray.org_x=origins[i*3]; hit.ray.org_y=origins[i*3+1]; hit.ray.org_z=origins[i*3+2];
    hit.ray.dir_x=directions[i*3]; hit.ray.dir_y=directions[i*3+1]; hit.ray.dir_z=directions[i*3+2];
    hit.ray.tnear=0; hit.ray.tfar=max_range; hit.ray.mask=~0u;
    hit.hit.geomID=RTC_INVALID_GEOMETRY_ID;
    for (auto& id : hit.hit.instID) id=RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(t->scene, &hit, nullptr);
    const bool valid = hit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
    distances[i] = valid ? hit.ray.tfar : std::numeric_limits<float>::infinity();
    faces[i] = valid ? static_cast<int32_t>(hit.hit.primID) : -1;
  }
}

extern "C" void terrain_destroy(void* handle) { delete static_cast<Terrain*>(handle); }
