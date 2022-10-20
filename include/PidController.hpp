/**
 * @file controller.hpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_PIDCONTROLLER_HPP_
#define INCLUDE_PIDCONTROLLER_HPP_
#include <memory>
#include <tuple>
namespace pd {
class PidController {
 private:
  /// @brief Proportional Gain
  const double kp{0.0};
  /// @brief Derivative Gain
  const double kd{0.0};
  /// @brief Time Step
  const double dt{0.01};
  /// @brief Previous error
  double prev_error{0.0};
  /// @brief Error for current time step
  double error{0.0};
  /// @brief Current output calculate using compute()
  double output{0.0};

 public:
  /// @brief Copy Constructor
  /// @param cpy A controller to be copied
  explicit PidController(std::shared_ptr<PidController> cpy);
  /// @brief Initialize the
  /// @param p Proportional Gain
  /// @param d Derivative Gain
  /// @param t Time step for controller calculations
  PidController(double p, double d, double t);
  ~PidController();
  /// @brief Compute the PID
  /// @param target Reference value = Value to reach
  /// @param curr Current value
  /// @return Controller Output
  double compute(double target, double curr);
  /// @brief Returns the computed error of the current output
  /// @param tar target
  /// @param curr curr
  /// @return Return the Error currently calculated
  double getError(double tar, double curr);
};
}  // namespace pd
#endif  // INCLUDE_PIDCONTROLLER_HPP_
