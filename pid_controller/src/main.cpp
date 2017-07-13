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
//  steering_Kp = 0.1;
//  steering_Ki = 0.0001;
//  steering_Kd = 4.0;
  steering_Kd = 6.0;
  steering_Kp = 0.01 * steering_Kd;
  steering_Ki = 0.1 * steering_Kp;

  PID speed_pid;
  double speed_Kp, speed_Ki, speed_Kd;
  speed_Kp = 10.0;
  speed_Ki = 10.0;
  speed_Kd = 10.0;

  double set_speed = 50; // 50 mph

  // TODO: Initialize the pid variable.
  steering_pid.Init(steering_Kp, steering_Ki, steering_Kd);
  speed_pid.Init(speed_Kp, speed_Ki, speed_Kd);

  bool use_twiddle = true;
  Twiddle *pTwiddle = nullptr;
  if (use_twiddle)
    pTwiddle = new Twiddle(0.002, 2000, steering_Kp, steering_Ki, steering_Kd);

  h.onMessage([&steering_pid, &speed_pid, set_speed, pTwiddle, use_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          TWIDDLE_STEP twiddle_step;
          // std::cout << "CTE: " << cte << std::endl;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // Run Twiddle. Once Twiddle converges then we can start running the simulator
          if (use_twiddle && pTwiddle->isActive()) {
            // std::cout << "Performing TWIDDLE step" << std::endl;
            twiddle_step = pTwiddle->step(cte);
            std::vector<double> dp = pTwiddle->getDP();
            std::vector<double> p = pTwiddle->getP();

            if (twiddle_step != TWIDDLE_STEP::ACCUMULATE_ERROR) {
              std::cout << "P: ";
              std::copy(p.begin(), p.end(), std::ostream_iterator<double>(std::cout, ", "));
              std::cout << std::endl;

              std::cout << "DP: ";
              std::copy(dp.begin(), dp.end(), std::ostream_iterator<double>(std::cout, ", "));
              std::cout << std::endl;
            }

            if (twiddle_step == TWIDDLE_STEP::RESET_SIMULATOR) {
              reset_simulator(ws);
              twiddle_step = pTwiddle->step(cte);
              if (twiddle_step == TWIDDLE_STEP::ACCUMULATE_ERROR) {
                std::vector<double> _dp = pTwiddle->getDP();
                std::vector<double> _p = pTwiddle->getP();
                std::cout << "P: ";
                std::copy(_p.begin(), _p.end(), std::ostream_iterator<double>(std::cout, ", "));
                std::cout << std::endl;

                std::cout << "DP: ";
                std::copy(_dp.begin(), _dp.end(), std::ostream_iterator<double>(std::cout, ", "));
                std::cout << std::endl;
              }
            }

            // Get the PID and use that
            PID *pPID = pTwiddle->getPID();
            pPID->UpdateError(cte);
            steer_value = pPID->TotalError();
            if (!pTwiddle->isActive()) {
              steering_pid = *pPID;
            }
          } else {
            // Steering PID
            steering_pid.UpdateError(cte);
            steer_value = steering_pid.TotalError();
            std::cout << "Average Error: " << cte << std::endl;
          }

          steer_value = std::max(std::min(1.0, steer_value), -1.0);

//          // Speed PID
//          double speed_error = set_speed - speed;
//          speed_pid.UpdateError(speed_error);
//          double throttle = speed_pid.TotalError();
//          throttle = std::max(std::min(1.0, steer_value), -1.0);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // DEBUG
          if (use_twiddle && pTwiddle->isActive() && twiddle_step != TWIDDLE_STEP::ACCUMULATE_ERROR) {
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

  if (use_twiddle)
    delete pTwiddle;
}
