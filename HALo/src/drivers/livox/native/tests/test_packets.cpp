// Exercise real SDK structs -> LVXR bytes without initializing the SDK.
#include "../livox_driver.cpp"
#include <cassert>
#include <cmath>
#include <vector>

std::vector<std::vector<uint8_t>> received;
void capture(const uint8_t* bytes, size_t size, void*) {
  received.emplace_back(bytes, bytes + size);
}
float value(const std::vector<uint8_t>& bytes, size_t offset) {
  float result;
  memcpy(&result, bytes.data() + offset, 4);
  return result;
}

int main() {
  g_callback = capture;
  // SDK handles contain IPv4 addresses in network order on the host.
  constexpr uint32_t handle = 0x2802a8c0; // 192.168.2.40
  std::vector<uint8_t> buffer(offsetof(LivoxLidarEthernetPacket, data) +
                            sizeof(LivoxLidarCartesianHighRawPoint));
  auto* packet = reinterpret_cast<LivoxLidarEthernetPacket*>(buffer.data());
  packet->length = buffer.size();
  packet->dot_num = 1;
  packet->data_type = kLivoxLidarCartesianCoordinateHighData;
  auto* point = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
  point->x = 1000; point->y = -2000; point->z = 3000;
  PointCloudCallback(handle, 0, packet, nullptr);
  assert(received.size() == 1);
  const auto& cloud = received.back();
  assert(cloud.size() == 32 && memcmp(cloud.data(), "LVXR", 4) == 0);
  assert(cloud[4] == 1 && cloud[5] == 1 && cloud[6] == 40 && cloud[7] == 1);
  assert(cloud[8] == 1 && cloud[9] == 0);
  assert(std::fabs(value(cloud, 20) - 1.0f) < 1e-6f);
  assert(std::fabs(value(cloud, 24) + 2.0f) < 1e-6f);
  assert(std::fabs(value(cloud, 28) - 3.0f) < 1e-6f);
  packet->length -= 1;
  PointCloudCallback(handle, 0, packet, nullptr);
  assert(received.size() == 1); // truncated SDK packet rejected

  buffer.resize(offsetof(LivoxLidarEthernetPacket, data) + sizeof(LivoxLidarImuRawPoint));
  packet = reinterpret_cast<LivoxLidarEthernetPacket*>(buffer.data());
  packet->length = buffer.size();
  packet->data_type = kLivoxLidarImuData;
  auto* imu = reinterpret_cast<LivoxLidarImuRawPoint*>(packet->data);
  imu->gyro_x = 0.1f; imu->gyro_y = 0.2f; imu->gyro_z = 0.3f;
  imu->acc_x = 0.0f; imu->acc_y = 0.0f; imu->acc_z = 1.0f;
  ImuDataCallback(handle, 0, packet, nullptr);
  assert(received.size() == 2);
  const auto& inertial = received.back();
  assert(inertial.size() == 44 && inertial[5] == 2 && inertial[6] == 40);
  assert(value(inertial, 20) == 0.1f && value(inertial, 40) == 1.0f);
  packet->length -= 1;
  ImuDataCallback(handle, 0, packet, nullptr);
  assert(received.size() == 2);
}
