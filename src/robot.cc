#include "robot.h"

void Robot::Set(double x, double y, double orientation)  {
  // Sets a robot coordinate.
  x_ = x;
  y_ = y;
  double double_pi = (2.0 * M_PI);

  if(orientation > double_pi)  {
    int n = (int)(orientation / double_pi);
    orientation_ = orientation - n*double_pi;
  } else {
    orientation_ = orientation;
  }
}

void Robot::SetNoise(double steering_noise, double distance_noise) {
  // Sets the noise parameters.
  // makes it possible to change the noise parameters
  // this is often useful in particle filters
  steering_noise_ = steering_noise;
  distance_noise_ = distance_noise;
}

void Robot::SetSteeringDrift(double drift)  {
  // Sets the systematical steering drift parameter
  steering_drift_ = drift;
}

void Robot::Move(double steering, double distance) {
  // steering = front wheel steering angle, limited by max_steering_angle
  // distance = total distance driven, most be non-negative
  double tolerance = 0.001;
  double max_steering_angle = (M_PI / 4.0);     // no more than 1.0
  
  std::default_random_engine generator;
 
  if(steering > max_steering_angle) {
      steering = max_steering_angle;
  } else if(steering < -max_steering_angle) {
      steering = -max_steering_angle;
  }
  
  if(distance < 0.0)  {
    distance = 0.0;
  }

  // apply noise
  std::normal_distribution<double> steering_dist(steering, steering_noise_);  // no change to steering since steering_noise_ = 0
  double steering2 = steering_dist(generator);
  std::normal_distribution<double> distance_dist(distance, distance_noise_);  // no change to distance since distance_noise_ = 0
  double distance2 = distance_dist(generator);

  // apply steering drift
  steering2 += steering_drift_;

  // Execute motion
  double turn = std::tan(steering2) * distance2 / length_;      // Not more than maximum angle
  double double_pi = (2.0 * M_PI);

  if(std::abs(turn) < tolerance)  {
    // approximate by straight line motion
    x_ += distance2 * std::cos(orientation_);
    y_ += distance2 * std::sin(orientation_);
    if((orientation_ + turn) > double_pi)  {
      int n = (int)((orientation_ + turn) / double_pi);
      orientation_ = (orientation_ + turn) - n*double_pi;
    } else {
      orientation_ = (orientation_ + turn);
    }
  } else  {
    // approximate bicycle model for motion
    double radius = distance2 / turn;
    double cx = x_ - (std::sin(orientation_) * radius);
    double cy = y_ + (std::cos(orientation_) * radius);
    if((orientation_ + turn) > double_pi)  {
      int n = (int)((orientation_ + turn) / double_pi);
      orientation_ = (orientation_ + turn) - n*double_pi;
    } else {
      orientation_ = (orientation_ + turn);
    }
    x_ = cx + (std::sin(orientation_) * radius);
    y_ = cy - (std::cos(orientation_) * radius);
  }
  
//  return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation);
}