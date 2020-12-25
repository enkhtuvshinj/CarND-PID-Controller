#include "pid.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  d_error_ = 0;
  i_error_ = 0;
  p_error_ = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;
  
/*    if(cte == 0)  {
    i_error_ = 0;
  }
  if(std::abs(cte)>1)  {
    i_error_ = 0;
  } */
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (-((p_error_ * Kp_) + (i_error_ * Ki_) + (d_error_ * Kd_)));
}