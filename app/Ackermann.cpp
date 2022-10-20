/**
 * @file ackermann.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "../include/Ackermann.hpp"

#include <array>

steering::Ackermann::Ackermann(double track_w, double w_base,
                               std::shared_ptr<pd::PidController> lwheel,
                               std::shared_ptr<pd::PidController> rwheel)
    : wheelBase(w_base),
      trackWidth(track_w),
      leftWheelController(lwheel),
      rightWheelController(rwheel) {}
steering::Ackermann::~Ackermann() {}

std::array<double, 2> steering::Ackermann::computeVelocity(
    double robotVelocity) {
  return std::array<double, 2>{0.0, 0.0};
}

std::array<double, 2> steering::Ackermann::computeSteerAngle(
    double currHead, double targetHead) {
  return std::array<double, 2>{0.0, 0.0};
}
