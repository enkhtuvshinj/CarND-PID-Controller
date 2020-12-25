#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <chrono> 
#include "json.hpp"
#include "pid.h"
#include "twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  /**
   * TODO: Initialize the pid variable.
   */
/*   
  std::chrono::time_point<std::chrono::system_clock> last_time; 
  last_time = std::chrono::system_clock::now();
*/

  // Compute hyperparameters using Twiddle
  double twiddle_tolerance = 0.2;
  double twiddle_steering_drift= 0;
  Twiddle twiddle(twiddle_tolerance, twiddle_steering_drift);
  vector<double> param = twiddle.ComputeCoefficients();
  std::cout << "Hyperparameters from Twiddle:" << std::endl;
  std::cout << "Best Error: " << param[3] << std::endl;
  std::cout << "Param_Kp: " << param[0] << " Param_Kd: " << param[1] << " Param_Ki: " << param[2] << std::endl << std::endl;
  
  
  // The tunned hyperparameters
  param = {0.090609, 2.70091, 0.00101};     // 0.154669, 3.09048, 0.00935  : 0.100009, 3.00008, 0.00935
  std::cout << "Final tunned hyperparameters:" << std::endl;
  std::cout << "Param_Kp: " << param[0] << " Param_Kd: " << param[1] << " Param_Ki: " << param[2] << std::endl;
  
  // Initialize throttle
  double throttle = 0.3;
  
  // Assign the optimized parameters to PID
  PID pid_steering;
  pid_steering.Init(param[0], param[1], param[2]);

  h.onMessage([&pid_steering, &throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) 
  {
/*     // Compute the elapsed time between webSocket calls
    std::chrono::time_point<std::chrono::system_clock> now_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = now_time - last_time; 
    last_time = now_time;
    std::cout << "Called after elapsed time: " << elapsed_seconds.count() << std::endl;
     */
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double current_speed = std::stod(j[1]["speed"].get<string>());
          double current_steering_angle = std::stod(j[1]["steering_angle"].get<string>());
          double steering_angle;
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          // 1. Steering angle control
          pid_steering.UpdateError(cte);
          steering_angle = pid_steering.TotalError();

          if (steering_angle > 1)  {
            steering_angle = 1;
          } else if (steering_angle < -1) {
            steering_angle = -1;
          } 
         
          
          // 2. Adjust throttle based on current steering angle and speed
          current_steering_angle = deg2rad(current_steering_angle);
          double max_speed = (120/std::exp(std::sqrt(current_steering_angle*5)));

          if(current_speed > max_speed) {
            throttle -= 0.005;
            throttle = (throttle < 0) ? 0.15:throttle;
          } else if (throttle <0.3)  {
            throttle += 0.0005;
          }
          
/*           double max_steering_angle = (5/std::exp(std::sqrt(current_speed/4)));
          max_steering_angle = (std::abs(max_steering_angle) > 1) ? 1: max_steering_angle;
          
          if(current_speed > 10)  {
            if(std::abs(steering_angle) > std::abs(max_steering_angle)) {
              steering_angle = (steering_angle > 0) ? max_steering_angle: -max_steering_angle;
            }
          }
 */          
          
          // DEBUG
          std::cout << " CTE: " << cte 
                    << " Steering: " << steering_angle 
                    << " CurrentAngle: " << current_steering_angle 
                    << " CurrentSpeed: " << current_speed 
                    << " Throttle: " << throttle<< std::endl;

          json msgJson;
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}