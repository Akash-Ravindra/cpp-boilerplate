/**
 * @file controller.cpp
 * @author Akash Ravindra (aravind2@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-10-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "../include/PidController.hpp"

#include <memory>

pd::PidController::PidController(double p, double d, double t)
    : kp{p}, kd{d}, dt{t} {}
pd::PidController::PidController(std::shared_ptr<PidController> cpy)
    : kp{cpy.get()->kp}, kd{cpy.get()->kd}, dt{cpy.get()->dt} {}
pd::PidController::~PidController() {}

double pd::PidController::compute(double tar, double curr) { return 0.0; }

double pd::PidController::getError(double tar, double curr) {
  return curr - tar;
}
