/**
 * @file main.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iostream>

#include "../include/Robot.hpp"

int main() {
  bot::Robot robo(45.0, 10.0, 2.5, 1.8);
  std::vector<std::array<double, 2>> outputValues = robo.moveRobot(5.0, 0.52);
  for (int i = 0; i < outputValues.size(); i++) {
    std::cout << "Iteration " << i << ": Velocity: " << outputValues.at(i)[0]
              << ", Heading:" << outputValues.at(i)[1] << std::endl;
  }
  return 0;
}
