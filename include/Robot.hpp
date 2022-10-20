/**
 * @file robot.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

#include <array>
#include <memory>
#include <tuple>
#include <vector>

#include "./Ackermann.hpp"
#include "./PidController.hpp"
namespace bot {
class Robot {
 private:
  /// @brief The maximum angle the driven wheels can turn by
  const double maxWheelAngle{0.0};
  /// @brief The maximum velocity of the robot
  const double maxVelocity{0.0};
  /// @brief The physical dimensions that correspond to the wheel base measure
  /// in meters
  const double wheelBase{0.01};
  /// @brief The physical dimension that corresponds to the distance between the
  /// wheels measured in meters
  const double trackWidth{0.0};
  /// @brief The current heading the COM of the robot
  double headingCurrent{0.0};
  /// @brief The current velocity of the COM of the robot
  double velocityCurrent{0.0};
  /// @brief Steering module(Ackermann steering) using to calculate the wheel
  /// angle and velocity
  std::shared_ptr<steering::Ackermann> steering;
  /// @brief Velocity controller to update the velocity of the Robot
  std::shared_ptr<pd::PidController> velocityController;

 public:
  /**
   * @brief Construct a new Robot object
   *
   * @param maxAngle Init maxWheelAngle
   * @param maxVel Init maxVelocity
   * @param wBase Init wheelBase
   * @param tWidth Init trackWidth
   */
  Robot(double maxAngle, double maxVel, double wBase, double tWidth);
  /**
   * @brief Destroy the Robot object
   *
   */
  ~Robot();
  /// @brief Construct a new std::vector<double>move Robot object
  /// @param targetVel The velocity that is to be reached by the robot
  /// @param targetHead The heading that is to be obtained by the robot
  /// @return std::vector<std::array<double, 2>> The heading and velocities of
  /// each wheel
  std::vector<std::array<double, 2>> moveRobot(double targetVel,
                                               double targetHead);
};
}  // namespace bot
#endif  // INCLUDE_ROBOT_HPP_
