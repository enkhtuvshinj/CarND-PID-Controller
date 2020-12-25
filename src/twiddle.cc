#include "twiddle.h"

// Constructor to set initial value to tolerance and steering drift 
// steering_drift should be in degree
Twiddle::Twiddle(double tol, double steering_drift)  {
  tol_ = tol;
  steering_drift_ = steering_drift;
};

// Compute coefficients using Twiddle algorithm
vector<double> Twiddle::ComputeCoefficients()  {
  vector<double> p  = {0, 0, 0};  // Kp, Kd, Ki
  vector<double> dp = {1, 1, 1};
  double err;
  Robot robot = MakeRobot();
  double best_err = Run(robot, p);

  double sum = dp[0] + dp[1] + dp[2];
  
  while(sum > tol_) {
    for(int i=0; i<p.size(); i++)  {
      p[i] += dp[i];
      robot = MakeRobot();
      err = Run(robot, p);
      
      if(err < best_err)  {
        best_err = err;
        dp[i] *= 1.1;
      } else  {
        p[i] -= 2*dp[i];
        robot = MakeRobot();
        err = Run(robot, p);
        
        if(err < best_err)  {
          best_err = err;
          dp[i] *= 1.1;
        } else  {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
    
    sum = dp[0] + dp[1] + dp[2];
  }
  
  p.push_back(best_err);
  return p;
}


// Create robot object by setting initial values to coordinates and steering drift
Robot Twiddle::MakeRobot() {
  // Resets the robot back to the initial position and drift.
  // You'll want to call this after you call `run`.
  Robot robot;
  robot.Set(0, 1, 0);
  double drift = deg2rad(steering_drift_);
  robot.SetSteeringDrift(drift);
  
  return robot;
}

// NOTE: We use params instead of tau_p, tau_d, tau_i
double Twiddle::Run(Robot robot, vector<double> params) {
  int n=100;
  double distance=1.0;
  double steer;
  double err = 0.0;
  double prev_cte = robot.y_;
  double int_cte = 0.0;
  
  for(int i=0; i<(2*n); i++) {
    double cte = robot.y_;
    double diff_cte = (cte - prev_cte);
    int_cte += cte;
    prev_cte = cte;
    steer = -params[0]*cte - params[1]*diff_cte - params[2]*int_cte;
    robot.Move(steer, distance);

    if(i >= n)  {
      err += cte*cte;
    }
  }
  return (err/n);
}
