#ifndef PID_H
#define PID_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp, double Kd, double Ki);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;
};

#endif  // PID_H