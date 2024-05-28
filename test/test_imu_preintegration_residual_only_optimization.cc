#include <iostream>
#include <vector>

#include "Eigen/Dense"

#include "core/imu_preintegrator.h"

void GenerateSimulationData(std::vector<Pose>* pose_list,
                            std::vector<Eigen::Vector3d>* linear_vel_list,
                            std::vector<ImuData>* imu_data_list) {
  constexpr double kTimeInterval{0.005};  // 200 Hz
  constexpr double kTimeEnd{30.0};        // 200 Hz

  const double radius = 4.0;
  const double fluctuation_z = 2.0;

  const double c = M_PI * 2.0 / kTimeEnd;

  double time = 0.0;
  while (time <= kTimeEnd) {
    const double s = time / kTimeEnd * M_PI * 2.0;
    Eigen::Vector3d p;
    p.x() = radius * std::cos(c * time);
    p.y() = radius * std::sin(c * time);
    p.z() = fluctuation_z * (1.0 - std::cos(c * time));
    Eigen::Vector3d v;
    v.x() = -radius * c * std::sin(c * time);
    v.y() = radius * c * std::cos(c * time);
    v.z() = fluctuation_z * c * std::sin(c * time);
    Eigen::Vector3d a;
    a.x() = -radius * c * c * std::sin(c * time);
    a.y() = -radius * c * c * std::sin(c * time);
    a.z() = fluctuation_z * c * c * std::sin(c * time);

    // Make rotation matrix
    Eigen::Vector3d ex{std::cos(c * time), std::sin(c * time), 0.0};
  }
}

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;

int main() { return 0; }