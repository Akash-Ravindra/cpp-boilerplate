/**
 * @file robot.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "../include/Robot.hpp"

#include <array>
#include <memory>
#include <vector>

#include "../include/Ackermann.hpp"
#include "../include/PidController.hpp"

bot::Robot::Robot(double maxAngle, double maxVel, double wBase, double tWidth)
    : maxWheelAngle{maxAngle},
      maxVelocity{maxVel},
      wheelBase{wBase},
      trackWidth{tWidth} {
  std::shared_ptr<pd::PidController> lWheel(
      std::make_shared<pd::PidController>(0.0, 0.0, 0.01));
  std::shared_ptr<pd::PidController> rWheel(
      std::make_shared<pd::PidController>(0.0, 0.0, 0.01));
  steering = std::make_shared<steering::Ackermann>(trackWidth, wheelBase,
                                                   lWheel, rWheel);
  velocityController = std::make_shared<pd::PidController>(0.0, 0.0, 0.01);
}
bot::Robot::~Robot() {}

std::vector<std::array<double, 2>> bot::Robot::moveRobot(double targetVel,
                                                         double targetHead) {
  steering->computeSteerAngle(headingCurrent, targetHead);
  double tarVel = velocityController->compute(targetVel, velocityCurrent);
  steering->computeVelocity(tarVel);
  return std::vector<std::array<double, 2>>{{0.0, 0.0}, {0.0, 0.0}};
}
