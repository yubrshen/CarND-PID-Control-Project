#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddler.h"
#include <math.h>
#include <algorithm> // std::min

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double trimWithin(double x, double lowerBound, double upperBond) {
  if (x <= lowerBound) return lowerBound;
  if (upperBond <= x) return upperBond;
}

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
  std::string input = "";
  bool to_run_twiddle = false;

  std::cout << "Would you run the twiddle process? The program can run without it with pre-selected parameters.\nThe twiddle process will take hours to converge. If you want to run, please enter \'Y\'.\n";
  getline(std::cin, input);
  if (input == "Y") {
    to_run_twiddle = true;
    std::cout << "Twiddle process will start soon.\n";
      } else {
    std::cout << "No twiddle process to run.\n";
  }

  uWS::Hub h;

  PID pid;
  //Twiddler twiddler(0.2, 3.0, 0.004); also OK, not as 0.1, 3.0, 0.004
  //Twiddler twiddler(0.1, 3.0, 0.004); // good parameters, so far the best of manual picking, without speed conditioning.
  Twiddler twiddler(0.1, 3.34337, 0.004); // optimized by Twiddler from (0.1, 3.0, 0.004)
  //Twiddler twiddler(0.16, 4.35, 0.255264);
  // Here is the best found by Twiddler: 0.19905 3.69262 0.004 starting from (0.1, 3.0, 0.004)
  // another good set 0.105 3.5 0.004 at faster speed.
  // another good one 0.19905 3.76306 0.004
  //Twiddler twiddler(0.1, 0.25, 0.03); // good parameters
  double current_time = 0.0;
  double previous_time = clock();

  // TODO: Initialize the pid variable.
  pid.Init(twiddler.params[0], twiddler.params[1], twiddler.params[2]);

  h.onMessage([&pid, &twiddler, &current_time, &previous_time, &to_run_twiddle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value = 0;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          current_time = clock();
          double dt = (current_time - previous_time)/CLOCKS_PER_SEC; // elapsed time in seconds
          previous_time = current_time;

          pid.UpdateError(cte, dt);
          double pid_steer = trimWithin(-pid.TotalError(), -1, 1);
          steer_value = pid_steer;
          // if (0.0001 < speed) {
          //   steer_value = 5*pid_steer/speed; // adjust the steer conditional to the current speed
          // } else {
          //   steer_value = pid_steer;
          // }
          json msgJson;
          msgJson["steering_angle"] = steer_value;

          double throttle = 0.45;
          double throttle_steer = throttle;
          double f = abs(steer_value)*(10 - speed);
          // if (0.0001 < f)
          //   throttle_steer = 7/(10*f);
          throttle = throttle - abs(steer_value)*speed/20;

          // throttle = std::min(throttle, throttle_steer); // slow down with steep steering, and too fast speed
          // std::cout << "throttle: " << throttle << " throttle_steer: " << throttle_steer << " steer_value: " << steer_value << " speed: " << speed << std::endl;
          msgJson["throttle"] = throttle; // was 0.3

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // DEBUG
          // std::cout << "CTE: " << cte << " angle: " << angle << " Steering Value: " << steer_value << std::endl;
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          if (to_run_twiddle) {
            bool adjustment_needed = twiddler.process(cte);
            if (adjustment_needed) {
              pid.Init(twiddler.params[0], twiddler.params[1], twiddler.params[2]);
              std::cout << "cte: " << cte << std::endl;
              if (2 < abs(cte)) {      // reset at the excessive cte
                // send out the reset msg
                std::string msg = "42[\"reset\",{}]";
                std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
            }
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
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
