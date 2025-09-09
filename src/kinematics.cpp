#include "kinematics.h"
#include <iostream>

namespace reachy_mini_kinematics {

double Kinematics::wrap_angle(double angle) {
  return angle - (2 * M_PI) * std::floor((angle + M_PI) * (1. / (2 * M_PI)));
}

Kinematics::Kinematics(double motor_arm_length, double rod_length)
    : motor_arm_length(motor_arm_length), rod_length(rod_length) {
  T_world_platform = Eigen::Affine3d::Identity();
}

void Kinematics::add_branch(Eigen::Vector3d branch_platform,
                            Eigen::Affine3d T_world_motor, double solution) {

  // Building a 3x6 jacobian relating platform velocity to branch anchor point
  // linear velocity Linear velocity is kept as identity and angular velocity is
  // using Varignon's formula w x p, which Is anti-symmetric -p x w and used in
  // matrix form [-p]
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, 6);
  jacobian.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

  Eigen::Vector3d p = -branch_platform;
  jacobian.block(0, 3, 3, 3) << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(),
      p.x(), 0;

  branches.push_back({branch_platform, T_world_motor, solution, jacobian});
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

    joint_angles[k] = wrap_angle(joint_angles[k]);
  }

  return joint_angles;
}

void Kinematics::reset_forward_kinematics(Eigen::Affine3d T_world_platform_) {
  this->T_world_platform = T_world_platform_;
}

Eigen::Affine3d Kinematics::forward_kinematics(Eigen::VectorXd joint_angles) {
  if (branches.size() != 6) {
    throw std::runtime_error("Forward kinematics requires exactly 6 branches");
  }

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd errors = Eigen::VectorXd::Zero(6);
  std::vector<Eigen::Vector3d> arms_motor;

  for (int k = 0; k < branches.size(); k++) {
    Branch &branch = branches[k];

    // Computing the position of motor arm in the motor frame
    Eigen::Vector3d arm_motor =
        motor_arm_length *
        Eigen::Vector3d(cos(joint_angles[k]), sin(joint_angles[k]), 0);
    arms_motor.push_back(arm_motor);

    // Expressing the tip of motor arm in the platform frame
    Eigen::Vector3d arm_platform =
        T_world_platform.inverse() * branch.T_world_motor * arm_motor;

    // Computing the current distance
    double current_distance = (arm_platform - branch.branch_platform).norm();

    // Computing the arm-to-branch vector in platform frame
    Eigen::Vector3d armBranch_platform = branch.branch_platform - arm_platform;

    // Computing the jacobian of the distance
    J.block(k, 0, 1, 6) = armBranch_platform.transpose() * branch.jacobian;
    errors(k) = rod_length - current_distance;
  }

  // If the error is sufficiently high, performs a line-search along the
  // direction given by the jacobian inverse
  if (errors.norm() > 1e-6) {
    Eigen::VectorXd V = J.inverse() * errors;

    for (int i = 0; i < line_search_maximum_iterations; i++) {
      Eigen::Affine3d T = Eigen::Affine3d::Identity();
      T.translation() = V.head(3);

      double norm = V.tail(3).norm();
      if (fabs(norm) > 1e-6) {
        T.linear() =
            Eigen::AngleAxisd(norm, V.tail(3).normalized()).toRotationMatrix();
      }
      Eigen::Affine3d T_world_platform2 = T_world_platform * T;

      Eigen::VectorXd new_errors(6);
      for (int k = 0; k < branches.size(); k++) {
        Branch &branch = branches[k];

        Eigen::Vector3d arm_platform =
            T_world_platform2.inverse() * branch.T_world_motor * arms_motor[k];
        double current_distance =
            (arm_platform - branch.branch_platform).norm();

        new_errors(k) = rod_length - current_distance;
      }

      if (new_errors.norm() < errors.norm()) {
        T_world_platform = T_world_platform2;
        break;
      } else {
        V = V * 0.5;
      }
    }
  }

  return T_world_platform;
}
} // namespace reachy_mini_kinematics