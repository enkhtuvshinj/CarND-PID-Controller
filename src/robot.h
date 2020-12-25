#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

using std::vector;

class Robot {
 public:
  void Set(double x, double y, double orientation);
  void SetNoise(double steering_noise, double distance_noise);
  void SetSteeringDrift(double drift);
  void Move(double steering, double distance);
  
  double x_ = 0.0;
  double y_ = 0.0;
  double orientation_ = 0.0;
  double length_ = 20.0;
  double steering_noise_ = 0.0;
  double distance_noise_ = 0.0;
  double steering_drift_ = 0.0;
  
};

#endif