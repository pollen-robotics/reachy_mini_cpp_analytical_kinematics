#include "kinematics.h"

namespace reachy_mini_kinematics {

Kinematics::Kinematics(double motor_arm_length, double rod_length)
    : motor_arm_length(motor_arm_length), rod_length(rod_length) {}

void Kinematics::add_branch(Eigen::Vector3d branch_platform,
                            Eigen::Affine3d T_world_motor) {
  branches.push_back({branch_platform, T_world_motor});
}

} // namespace reachy_mini_kinematics