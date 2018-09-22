#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>



// for convenience
using json = nlohmann::json;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  
  uWS::Hub h;
  

  PID pid_steer, pid_throttle;
  // TODO: Initialize the pid_steer variable.
  double init_Kp_steer = -0.13;   // best manual tuned to: -0.15;
  double init_Ki_steer = -0.0002;   // best manual tuned to: -0.00003;
  double init_Kd_steer = -3.5;   // best manual tuned to: -2.5;
  
  double init_Kp_throttle = 0.87;
  double init_Ki_throttle = 0.00000;
  double init_Kd_throttle = 1.5;
  
  // debugging
  long long loop_count = 1;
  
  pid_steer.Init(init_Kp_steer, init_Ki_steer, init_Kd_steer);
  
  pid_throttle.Init(init_Kp_throttle, init_Ki_throttle, init_Kd_throttle);
  

  h.onMessage([&pid_steer, &pid_throttle, &loop_count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          //double dKp = 0.005;
          //double dKi = 0.001;
          //double dKd = 0.3;
          //double tol = 0.5;
          
          //pid.Twiddle(tol, dKp, dKi, dKd, cte);
          
          loop_count++; // to keep track of loop, might be helpful for logging purposes
          
          // Use PID to compute steering wheel angle
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          
          double throttle = 0.8 - abs(steer_value);
          std::cout << "Throttle ideal: " << throttle << std::endl;
          
          // Use PID to compute throttle
          pid_throttle.UpdateError(throttle);
          throttle_value = pid_throttle.TotalError();
          
          // DEBUG
          std::cout << "Throttle controlled: " << throttle_value << std::endl;
          
          if(loop_count<10) // Initially, just to start car strictly in forward direction
          {
            throttle_value = 0.3;
          }
          // Control throttle within threholds to avoid over acceleration in forward and reverse direction
          if(throttle_value > 0.6)
          {
            throttle_value = 0.6;
          }
          else if(throttle_value < -0.3)
          {
            throttle_value = -0.2;
          }
          
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //udacity provided value: 0.3
          msgJson["speed"] = speed;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
