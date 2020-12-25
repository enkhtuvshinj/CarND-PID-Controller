#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <iostream>
#include <string>
#include <vector>
#include "robot.h"
#include "helper.h"

class Twiddle {
 public:
  Twiddle(double tol, double steering_drift);
  
  vector<double> ComputeCoefficients();
  Robot MakeRobot();
  double Run(Robot robot, vector<double> params);
  
 private:
  double tol_;
  double steering_drift_;
  };

#endif