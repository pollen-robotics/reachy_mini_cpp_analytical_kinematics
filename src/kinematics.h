#pragma once

#include <Eigen/Dense>
#include <vector>

namespace reachy_mini_kinematics {
struct Kinematics {
  struct Branch {
    // Branch position expressed in the platform frame
    Eigen::Vector3d branch_platform;

    // Motor pose in the world
    Eigen::Affine3d T_world_motor;

    // Solution sign (1.0 or -1.0)
    double solution = 1.0;

    // Internal 3x6 jacobian matrix used for forward kinematics
    Eigen::MatrixXd jacobian;
  };

  // Kinematic branches
  std::vector<Branch> branches;

  /**
   * @brief Create a kinematics object
   * @param motor_arm_length length of the motor arm
   * @param rod_length length of the rods
   */
  Kinematics(double motor_arm_length, double rod_length);

  /**
   * @brief Adds a branch to the kinematics
   * @param branch_platform position of the branch in the platform frame
   * @param T_world_motor transformation matrix from motor to world frame
   */
  void add_branch(Eigen::Vector3d branch_platform,
                  Eigen::Affine3d T_world_motor, double solution = 1.0);

  /**
   * @brief Computes the inverse kinematics for the given platform pose
   * @param T_world_platform transformation matrix from platform to world frame
   * @return joint angles that achieve the desired platform pose
   */
  Eigen::VectorXd inverse_kinematics(Eigen::Affine3d T_world_platform);

  /**
   * @brief Resets the forward kinematics estimation
   * @param T_world_platform
   */
  void reset_forward_kinematics(Eigen::Affine3d T_world_platform);

  /**
   * @brief Update the platform pose estimation based on joint angles
   * @param joint_angles vector of joint angles
   */
  Eigen::Affine3d forward_kinematics(Eigen::VectorXd joint_angles);

  // Kinematics dimensions
  double motor_arm_length;
  double rod_length;

  // Current platform estimation for forward kinematics
  Eigen::Affine3d T_world_platform;

  static double wrap_angle(double angle);

  // Maximum iterations for line search in forward kinematics
  int line_search_maximum_iterations = 16;
};

} // namespace reachy_mini_kinematics