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
#include <cmath>
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
      std::make_shared<pd::PidController>(3.5, 0.0, 0.01));
  std::shared_ptr<pd::PidController> rWheel(
      std::make_shared<pd::PidController>(3.5, 0.0, 0.01));
  std::shared_ptr<pd::PidController> velocityContr(
      std::make_shared<pd::PidController>(14, 0.001, 0.01));

  steering = std::make_shared<steering::Ackermann>(
      trackWidth, wheelBase, lWheel, rWheel, velocityContr);
}
bot::Robot::~Robot() {}

std::vector<std::array<double, 2>> bot::Robot::moveRobot(double targetVel,
                                                         double targetHead) {
  double velThreshold{0.001};
  double angleThreshold{0.0872665};
  int iterationThreshold{50};
  int iterCount{0};
  std::array<double, 2> opWheelAngle{0, 0};
  std::array<double, 2> opWheelVelocity{0, 0};
  std::vector<std::array<double, 2>> outputValues{
      {velocityCurrent, headingCurrent}};

  while ((fabs(targetVel - velocityCurrent) > velThreshold) &&
         (iterCount < iterationThreshold)) {
    // Calculate new wheel Angles
    opWheelAngle = steering->computeSteerAngle(headingCurrent, targetHead);
    // Calculate new wheel Velocity
    opWheelVelocity = steering->computeVelocity(targetVel);
    // Calculate new robot Velocity
    velocityCurrent = opWheelVelocity[0] * wheelBase / sin(opWheelAngle[0]);
    // Calculate new robot Heading
    headingCurrent += ((velocityCurrent / wheelBase) *
                       atan2(2 * tan(opWheelAngle[0]) * tan(opWheelAngle[1]),
                             (opWheelAngle[0]) + tan(opWheelAngle[1]))) *
                      0.01;
    // Saving new velocity and heading in a vector
    outputValues.push_back(std::array<double, 2>{
        velocityCurrent, std::fmod(headingCurrent, 3.14159)});
    iterCount++;
  }
  return outputValues;
}
