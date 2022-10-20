/**
 * @file test.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <gtest/gtest.h>
#include <math.h>
#include <vector>

#include "../include/PidController.hpp"
#include "../include/Ackermann.hpp"
#include "../include/Robot.hpp"

/// Test to check the Functionality of the Pid Compute method
TEST(PidControllerTest, test_compute) {
  pd::PidController control(2.0, 0.0, 0.01);
  EXPECT_NEAR(control.compute(5.0, 7.0), 4.0, 0.0001);
}
TEST(PidControllerTest, test_getError) {
  pd::PidController control(2.0, 0, 0.01);
  EXPECT_EQ(control.getError(2,2), 0.0);
}
/// Test to check the Functionality of the Robot move to  method
TEST(RobotTest, test_getRobot) {
  bot::Robot robot(45.0,10.0,2.5,1.8);
  EXPECT_EQ(robot.moveRobot(0,10)[0][0], 10.0);
}