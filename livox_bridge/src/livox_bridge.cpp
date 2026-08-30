// livox_bridge — thin Livox-SDK2 sidecar for the Capra Roboguard.
//
// Brings up the two real Mid-360 lidars (they're raw on the LAN, not behind
// rove_sensor_api) and re-emits each unit's point cloud + IMU as simple "LVXR"
// UDP datagrams to the rove_control_bridge. Points are in the LIDAR SENSOR frame,
// in METRES; the bridge applies the URDF mount extrinsics + Livox-IMU pole-shake
// compensation, then runs the hazard reflex / cost map.
//
// Keeping this a thin C++ shim (the mature SDK does the handshake + decode) and
// all the geometry/filtering in the Rust bridge is deliberate: the math stays
// testable + tunable without rebuilding C++. See livox_bridge/README.md.
//
// Wire format (little-endian), one datagram per SDK callback:
//   off 0  : magic   "LVXR"
//   off 4  : u8      version (1)
//   off 5  : u8      msg_type (1 = points, 2 = imu)
//   off 6  : u8      lidar_id (last IP octet: 40 = bottom, 41 = top)
//   off 7  : u8      data_type (1 = cartesian-high, 0 = imu)
//   off 8  : u16     count (points, or 1 for imu)
//   off 10 : u16     reserved
//   off 12 : u64     timestamp_ns (host arrival, CLOCK_REALTIME)
//   off 20 : payload  points: count*(f32 x,y,z) m | imu: f32 gyro_xyz, acc_xyz (acc in g)

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <thread>

namespace {

int g_sock = -1;
sockaddr_in g_pts_addr{};
sockaddr_in g_imu_addr{};
std::atomic<uint64_t> g_pts_pkts{0};
std::atomic<uint64_t> g_imu_pkts{0};

constexpr uint8_t MAGIC[4] = {'L', 'V', 'X', 'R'};
constexpr size_t HDR = 20;
constexpr uint16_t MAX_PTS = 256; // Mid-360 high-data is ~96 pts/packet; headroom.

// The SDK handle is the lidar IP in network byte order, so the high host-order
// byte is the IP's last octet (40 = bottom, 41 = top).
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
  if (n == 0) return;
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
  sendto(g_sock, buf, HDR + cnt * 12u, 0, (sockaddr*)&g_pts_addr, sizeof(g_pts_addr));
  g_pts_pkts++;
}

void ImuDataCallback(uint32_t handle, const uint8_t, LivoxLidarEthernetPacket* data, void*) {
  if (!data || data->data_type != kLivoxLidarImuData) return;
  auto* imu = reinterpret_cast<LivoxLidarImuRawPoint*>(data->data);
  uint8_t buf[HDR + 24];
  fill_hdr(buf, /*imu*/ 2, lidar_id_from_handle(handle), /*imu*/ 0, 1, now_ns());
  float vals[6] = {imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->acc_x, imu->acc_y, imu->acc_z};
  memcpy(buf + HDR, vals, sizeof(vals));
  sendto(g_sock, buf, HDR + sizeof(vals), 0, (sockaddr*)&g_imu_addr, sizeof(g_imu_addr));
  g_imu_pkts++;
}

void WorkModeCb(livox_status, uint32_t, LivoxLidarAsyncControlResponse*, void*) {}

void InfoChangeCallback(uint32_t handle, const LivoxLidarInfo* info, void*) {
  if (!info) return;
  printf("[livox_bridge] lidar up: handle=%u id=%u sn=%s\n", handle,
         lidar_id_from_handle(handle), info->sn);
  // Start sampling. Do NOT touch ESC mode (the SDK warns it needs a power cycle).
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCb, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  if (argc < 2) {
    printf("usage: livox_bridge <mid360_config.json> [dest_ip=127.0.0.1] [base_port=7020]\n");
    return 1;
  }
  const char* cfg = argv[1];
  const char* dest = argc > 2 ? argv[2] : "127.0.0.1";
  uint16_t base = argc > 3 ? static_cast<uint16_t>(atoi(argv[3])) : 7020;

  g_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_sock < 0) {
    perror("socket");
    return 1;
  }
  g_pts_addr.sin_family = AF_INET;
  g_pts_addr.sin_port = htons(base);
  g_imu_addr.sin_family = AF_INET;
  g_imu_addr.sin_port = htons(base + 1);
  if (inet_pton(AF_INET, dest, &g_pts_addr.sin_addr) != 1 ||
      inet_pton(AF_INET, dest, &g_imu_addr.sin_addr) != 1) {
    printf("bad dest ip: %s\n", dest);
    return 1;
  }

  if (!LivoxLidarSdkInit(cfg)) {
    printf("Livox SDK init failed (config %s)\n", cfg);
    LivoxLidarSdkUninit();
    return 1;
  }
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(InfoChangeCallback, nullptr);
  if (!LivoxLidarSdkStart()) {
    printf("Livox SDK start failed\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  printf("[livox_bridge] streaming Mid-360 -> %s points:%u imu:%u (LVXR)\n", dest, base,
         base + 1);
  uint64_t last_p = 0, last_i = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    uint64_t p = g_pts_pkts.load(), i = g_imu_pkts.load();
    printf("[livox_bridge] points +%llu (%llu) imu +%llu (%llu)\n",
           static_cast<unsigned long long>(p - last_p), static_cast<unsigned long long>(p),
           static_cast<unsigned long long>(i - last_i), static_cast<unsigned long long>(i));
    last_p = p;
    last_i = i;
  }
  LivoxLidarSdkUninit();
  return 0;
}
