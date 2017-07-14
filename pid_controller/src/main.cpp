#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <math.h>
#include <iterator>     // std::ostream_iterator

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

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
  std::cout << "Reset Simulator" << std::endl;
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID steering_pid;
  double steering_Kp, steering_Ki, steering_Kd;
  // Hand Tuned Parameters
//  steering_Kp = 0.1;
//  steering_Ki = 0.0001;
//  steering_Kd = 4.0;
  // Twiddle Based Initial parameters
  /*
   * Steering PID: (0.098, 0.00100017, 20.5)
   * Steering DP: (3.83208e-05, 4.46062e-09, 0.122667)
   * 42["steer",{"steering_angle":-1.0,"throttle":0.3}]
   * CTE: 0.7687 Steering Value: -1
   */

  PID throttle_pid;
  double speed_Kp, speed_Ki, speed_Kd;
  speed_Kd = 5.0;
  speed_Kp = 0.01 * speed_Kd;
  speed_Ki = 0.01 * speed_Kp;

  double set_speed = 100; // 50 mph

  // TODO: Initialize the pid variable.
  throttle_pid.Init(speed_Kp, speed_Ki, speed_Kd);

  bool use_steering_twiddle = false;
  bool use_throttle_twiddle = false;
  Twiddle *pSteeringTwiddle = nullptr;
  Twiddle *pThrottleTwiddle = nullptr;
  if (use_steering_twiddle) {
    steering_Kd = 10.0;
    steering_Kp = 0.01 * steering_Kd;
    steering_Ki = 0.01 * steering_Kp;
    pSteeringTwiddle = new Twiddle("Steering", 0.02, 2500, steering_Kp, steering_Ki, steering_Kd);
    steering_pid.Init(steering_Kp, steering_Ki, steering_Kd);
  } else {
//    steering_Kp = 0.098;
//    steering_Ki = 0.00100017;
//    steering_Kd = 20.5;

    steering_Kp = 0.1;
    steering_Ki = 0.0001;
    steering_Kd = 4.0;
    steering_pid.Init(steering_Kp, steering_Ki, steering_Kd);
  }

  if (use_throttle_twiddle) {
    pThrottleTwiddle = new Twiddle("Throttle", 0.2, 2000, speed_Kp, speed_Ki, speed_Kd);
  }

  h.onMessage([&steering_pid, &throttle_pid, set_speed, pSteeringTwiddle, pThrottleTwiddle, use_steering_twiddle, use_throttle_twiddle]
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
          double steer_value;
          double throttle_value;
          TWIDDLE_STEP twiddle_step, throttle_step;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // Run Twiddle. Once Twiddle converges then we can start running the simulator
          if (use_steering_twiddle && pSteeringTwiddle->isActive()) {
            // std::cout << "Performing TWIDDLE step" << std::endl;
            twiddle_step = pSteeringTwiddle->step(cte);

            if (twiddle_step == TWIDDLE_STEP::RESET_SIMULATOR) {
              reset_simulator(ws);
              twiddle_step = pSteeringTwiddle->step(cte);
            }

            // Get the PID and use that
            PID *pPID = pSteeringTwiddle->getPID();
            pPID->UpdateError(cte);
            steer_value = pPID->TotalError();
            if (!pSteeringTwiddle->isActive()) {
              steering_pid = *pPID;
            }
          } else {
            // Steering PID
            steering_pid.UpdateError(cte);
            steer_value = steering_pid.TotalError();
            std::cout << "Steering PID: " << steering_pid.Kp << ", " << steering_pid.Ki << ", " << steering_pid.Kd << ")" << std::endl;
            std::cout << "Average Error: " << cte << std::endl;
          }

          steer_value = std::max(std::min(1.0, steer_value), -1.0);

          if (use_throttle_twiddle && pThrottleTwiddle->isActive()) {
            throttle_step = pThrottleTwiddle->step(fabs(cte));

            if (throttle_step == TWIDDLE_STEP::RESET_SIMULATOR) {
              reset_simulator(ws);
              throttle_step = pThrottleTwiddle->step(fabs(cte));
            }

            // Get the PID and use that
            PID *pPID = pThrottleTwiddle->getPID();
            pPID->UpdateError(fabs(cte));
            throttle_value = fabs(pPID->TotalError());
            if (!pThrottleTwiddle->isActive()) {
              throttle_pid = *pPID;
            }
          } else {
            // Steering PID
            throttle_pid.UpdateError(fabs(cte));
            throttle_value = fabs(throttle_pid.TotalError());
            std::cout << "Throttle PID: " << throttle_pid.Kp << ", " << throttle_pid.Ki << ", " << throttle_pid.Kd << ")" << std::endl;
            std::cout << "Average Error: " << cte << std::endl;
          }

          throttle_value = std::max(std::min(1.0, throttle_value), 0.0);
          if (!use_throttle_twiddle) {
            throttle_value = 0.3;
          }

//          // Speed PID
//          double speed_error = set_speed - speed;
//          speed_pid.UpdateError(speed_error);
//          double throttle = speed_pid.TotalError();
//          throttle = std::max(std::min(1.0, steer_value), -1.0);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // DEBUG
          if (use_steering_twiddle && pSteeringTwiddle->isActive() && twiddle_step != TWIDDLE_STEP::ACCUMULATE_ERROR) {
            std::cout << msg << std::endl;
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
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

  if (use_steering_twiddle)
    delete pSteeringTwiddle;
}
