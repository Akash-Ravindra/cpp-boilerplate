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

#include <math.h>

#include <array>
#include <cmath>
#include <iostream>

#include "../include/PidController.hpp"

steering::Ackermann::Ackermann(double track_w, double w_base,
                               std::shared_ptr<pd::PidController> lwheel,
                               std::shared_ptr<pd::PidController> rwheel,
                               std::shared_ptr<pd::PidController> velocityContr)
    : wheelBase(w_base),
      trackWidth(track_w),
      leftWheelController(lwheel),
      rightWheelController(rwheel),
      velocityController(velocityContr) {}
steering::Ackermann::~Ackermann() {}

std::array<double, 2> steering::Ackermann::computeVelocity(double tarVelocity) {
  velocity += velocityController.compute(
      tarVelocity, velocity);  // Update robot velocity value
  return std::array<double, 2>{
      velocity * sin(wheelAngles[0]) / wheelBase,
      velocity * sin(wheelAngles[1]) / wheelBase};  // calculate wheel velocity
}

std::array<double, 2> steering::Ackermann::computeSteerAngle(
    double currHead, double targetHead) {
  double delta = targetHead - currHead;  // Calculate change in heading
  std::array<double, 2> targetwheelAngles{0, 0};
  // Initialize Target Wheel Angles
  if (delta == 0) {
    return wheelAngles;  // No change in wheel Angles
  } else {
    curvatureRadius = wheelBase / tan(delta);  // Calculate Radius of Curvature
    // Calculate Wheel angles. Positive delta signifies right turn
    if (delta < 0) {  // Left Turn
      targetwheelAngles[0] =
          atan2(wheelBase, (curvatureRadius - (0.5 * trackWidth)));
      targetwheelAngles[1] =
          atan2(wheelBase, (curvatureRadius + (0.5 * trackWidth)));
    } else {  // Right Turn
      targetwheelAngles[0] =
          atan2(wheelBase, (curvatureRadius + (0.5 * trackWidth)));
      targetwheelAngles[1] =
          atan2(wheelBase, (curvatureRadius - (0.5 * trackWidth)));
    }
    // Left Wheel angle control
    targetwheelAngles[0] =
        wheelAngles[0] +
        leftWheelController.compute(targetwheelAngles[0] + wheelAngles[0],
                                    wheelAngles[0]);
    // Right Wheel angle control
    targetwheelAngles[1] =
        wheelAngles[1] +
        rightWheelController.compute(targetwheelAngles[1] + wheelAngles[1],
                                     wheelAngles[1]);
    for (int i = 0; i < 2; i++) {  // Limiting wheel angles to 45 degrees
      if (targetwheelAngles[i] > 0.785398) {
        targetwheelAngles[i] = 0.785398;
      } else if (targetwheelAngles[i] < -0.785398) {
        targetwheelAngles[i] = -0.785398;
      }
    }
    wheelAngles = targetwheelAngles;  // Update Wheel Angles
  }
  return targetwheelAngles;
}
