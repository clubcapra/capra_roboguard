// In-process SDK2 adapter. Rust owns telemetry and LVXR forwarding.
// Callback packets retain the original 20-byte LVXR header and payload.
// start/stop are serialized; SDK uninit joins workers before context is freed.
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>

namespace {

using PacketCallback = void (*)(const uint8_t*, size_t, void*);
PacketCallback g_callback = nullptr;
void* g_context = nullptr;
std::mutex g_lifecycle;
bool g_started = false;

constexpr uint8_t MAGIC[4] = {'L', 'V', 'X', 'R'};
constexpr size_t HDR = 20;
constexpr uint16_t MAX_PTS = 256; // Mid-360 high-data is ~96 pts/packet; headroom.

// The SDK handle is the lidar IP in network byte order, so the high host-order
// byte is the IP's last octet (used for the configured stable identity).
inline uint8_t lidar_id_from_handle(uint32_t handle) {
  return static_cast<uint8_t>((handle >> 24) & 0xFF);
}

uint64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

void put_u16(uint8_t* b, uint16_t v) {
  b[0] = v & 0xFF;
  b[1] = (v >> 8) & 0xFF;
}
void put_u64(uint8_t* b, uint64_t v) {
  for (int i = 0; i < 8; i++) b[i] = (v >> (8 * i)) & 0xFF;
}
void fill_hdr(uint8_t* b, uint8_t type, uint8_t lid, uint8_t dtype, uint16_t count,
              uint64_t ts) {
  memcpy(b, MAGIC, 4);
  b[4] = 1;
  b[5] = type;
  b[6] = lid;
  b[7] = dtype;
  put_u16(b + 8, count);
  put_u16(b + 10, 0);
  put_u64(b + 12, ts);
}

void PointCloudCallback(uint32_t handle, const uint8_t, LivoxLidarEthernetPacket* data, void*) {
  if (!data || data->data_type != kLivoxLidarCartesianCoordinateHighData) return;
  const uint16_t n = data->dot_num;
  if (n == 0 || data->length < offsetof(LivoxLidarEthernetPacket, data) +
                                 size_t(n) * sizeof(LivoxLidarCartesianHighRawPoint)) return;
  auto* pts = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
  uint16_t cnt = n < MAX_PTS ? n : MAX_PTS;

  uint8_t buf[HDR + MAX_PTS * 12];
  fill_hdr(buf, /*points*/ 1, lidar_id_from_handle(handle), /*high*/ 1, cnt, now_ns());
  uint8_t* p = buf + HDR;
  for (uint16_t i = 0; i < cnt; i++) {
    float x = pts[i].x * 0.001f, y = pts[i].y * 0.001f, z = pts[i].z * 0.001f;
    memcpy(p, &x, 4);
    memcpy(p + 4, &y, 4);
    memcpy(p + 8, &z, 4);
    p += 12;
  }
  g_callback(buf, HDR + cnt * 12u, g_context);
}

void ImuDataCallback(uint32_t handle, const uint8_t, LivoxLidarEthernetPacket* data, void*) {
  if (!data || data->data_type != kLivoxLidarImuData) return;
  if (data->length < offsetof(LivoxLidarEthernetPacket, data) + sizeof(LivoxLidarImuRawPoint)) return;
  auto* imu = reinterpret_cast<LivoxLidarImuRawPoint*>(data->data);
  uint8_t buf[HDR + 24];
  fill_hdr(buf, /*imu*/ 2, lidar_id_from_handle(handle), /*imu*/ 0, 1, now_ns());
  float vals[6] = {imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->acc_x, imu->acc_y, imu->acc_z};
  memcpy(buf + HDR, vals, sizeof(vals));
  g_callback(buf, HDR + sizeof(vals), g_context);
}

void WorkModeCb(livox_status, uint32_t, LivoxLidarAsyncControlResponse*, void*) {}

void InfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info, void*) {
  if (!info) return;
  printf("[HALo/livox] lidar up: handle=%u id=%u sn=%s\n", handle,
         lidar_id_from_handle(handle), info->sn);
  // Start sampling. Do NOT touch ESC mode (the SDK warns it needs a power cycle).
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCb, nullptr);
}

}  // namespace


extern "C" int halo_livox_start(const char* config, PacketCallback callback, void* context) {
  std::lock_guard<std::mutex> lock(g_lifecycle);
  if (g_started) return 2;
  if (!config || !callback || !context) return 3;
  g_callback = callback;
  g_context = context;
  if (!LivoxLidarSdkInit(config)) {
    LivoxLidarSdkUninit();
    g_callback = nullptr;
    g_context = nullptr;
    return 1;
  }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, nullptr);
  if (!LivoxLidarSdkStart()) {
    LivoxLidarSdkUninit();
    g_callback = nullptr;
    g_context = nullptr;
    return 1;
  }
  g_started = true;
  return 0;
}

extern "C" void halo_livox_stop() {
  std::lock_guard<std::mutex> lock(g_lifecycle);
  if (!g_started) return;
  LivoxLidarSdkUninit();
  g_started = false;
  g_callback = nullptr;
  g_context = nullptr;
}
