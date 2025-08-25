#include "kinematics.h"

namespace reachy_mini_kinematics {

Kinematics::Kinematics(double motor_arm_length, double rod_length)
    : motor_arm_length(motor_arm_length), rod_length(rod_length) {}

void Kinematics::add_branch(Eigen::Vector3d branch_platform,
                            Eigen::Affine3d T_world_motor, double solution) {
  branches.push_back({branch_platform, T_world_motor, solution});
}

Eigen::VectorXd
Kinematics::inverse_kinematics(Eigen::Affine3d T_world_platform) {
  Eigen::VectorXd joint_angles(branches.size());

  double rs = motor_arm_length;
  double rp = rod_length;

  for (int k = 0; k < branches.size(); k++) {
    Branch &branch = branches[k];

    Eigen::Vector3d branch_motor = branch.T_world_motor.inverse() *
                                   T_world_platform * branch.branch_platform;
    double px = branch_motor.x();
    double py = branch_motor.y();
    double pz = branch_motor.z();

    joint_angles[k] =
        2 *
        atan2(
            (2 * py * rs +
             branch.solution *
                 sqrt(
                     -(pow(px, 4)) - 2 * pow(px, 2) * pow(py, 2) -
                     2 * pow(px, 2) * pow(pz, 2) + 2 * pow(px, 2) * pow(rp, 2) +
                     2 * pow(px, 2) * pow(rs, 2) - pow(py, 4) -
                     2 * pow(py, 2) * pow(pz, 2) + 2 * pow(py, 2) * pow(rp, 2) +
                     2 * pow(py, 2) * pow(rs, 2) - pow(pz, 4) +
                     2 * pow(pz, 2) * pow(rp, 2) - 2 * pow(pz, 2) * pow(rs, 2) -
                     pow(rp, 4) + 2 * pow(rp, 2) * pow(rs, 2) - pow(rs, 4))),
            (pow(px, 2) + 2 * px * rs + pow(py, 2) + pow(pz, 2) - pow(rp, 2) +
             pow(rs, 2)));
  }

  return joint_angles;
}
} // namespace reachy_mini_kinematics