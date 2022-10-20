/**
 * @file ackermann.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_ACKERMANN_HPP_
#define INCLUDE_ACKERMANN_HPP_
#include <array>
#include <memory>

#include "./PidController.hpp"

namespace steering {
class Ackermann {
 private:
  /// @brief The distance between the axels measured in meters
  const double wheelBase{0.0};
  /// @brief The distance between the wheels measured along the axel in meters
  const double trackWidth{0.0};
  /// @brief Pid controller used to drive the wheel to the desired angle
  const pd::PidController leftWheelController;
  /// @brief Pid controller used to drive the wheel to the desired angle
  const pd::PidController rightWheelController;
  /// @brief Radius of curvature of the turn
  double curvatureRadius{0.0};
  /// @brief The angle made by the wheel wrt the vertical
  std::array<double, 2> wheelAngles{0.0, 0.0};
  /// @brief The velocities attained by the wheel
  std::array<double, 2> wheelVelocity{0.00, 0.00};

 public:
  /// @brief Default Initializer
  Ackermann();
  /**
   * @brief Construct a new Ackermann object
   *
   * @param lwheel Controller that will be used to calculate the heading of the
   * left wheel
   * @param rwheel Controller that will be used to calculate the heading of the
   * right wheel
   */
  Ackermann(double track_w, double w_base,
            std::shared_ptr<pd::PidController> lwheel,
            std::shared_ptr<pd::PidController> rwheel);
  /**
   * @brief Destroy the Ackermann object
   *
   */
  ~Ackermann();
  /// @brief Used to calculate the velocities of each driven while having
  /// calculated their angles
  /// @param robotVelocity The velocity of the COM of the robot
  /// @return std::array<double, 2> Containing the velocities of the left and
  /// right wheel
  std::array<double, 2> computeVelocity(double robotVelocity);

  /// @brief Computes the angles of the wheel to achieve the required turn angle
  /// @param currHead The current heading of the robot
  /// @param targetHead The required heading of the robot
  /// @return std::array<double, 2> The computed headings of each wheel
  std::array<double, 2> computeSteerAngle(double currHead, double targetHead);
};
}  // namespace steering
#endif  // INCLUDE_ACKERMANN_HPP_
