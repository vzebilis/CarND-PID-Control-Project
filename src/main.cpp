#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

  PID steer_pid;
  steer_pid.Init(0.15, 0.000, 4.0);

  h.onMessage([&steer_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
//          static int i = 0;
//          static long count = 0;
//          static double err = 0;
//          static double best_err = 1000000;
//          static double dK[NUM_COEF] = {0.1, 0.1, 0.1};
//          static double K[NUM_COEF] = {0.0, 0.000, 0.0};
//          if (TWIDDLE) {
//            if (count == 0) {
//              K[i] += dK[i];
//              steer_pid.Init(K[0], K[2], K[1]);
//              std::cout << "Kp: " << K[0] << " Kd: " << K[1] << " Ki: " << K[2] << std::endl;
//              std::cout << "dKp: " << dK[0] << " dKd: " << dK[1] << " dKi: " << dK[2] << std::endl;
//            }
//            ++count;
//            if (count == ITER) {
//              double cur_err = err / ITER;
//              if (cur_err < best_err) {
//                best_err = cur_err;
//                dK[i] *= 1.1;
//                count = 0;
//              } else {
//                K[i] -= 2 * dK[i];
//                steer_pid.Init(K[0], K[2], K[1]);
//                std::cout << "Kp: " << K[0] << " Kd: " << K[1] << " Ki: " << K[2] << std::endl;
//                std::cout << "dKp: " << dK[0] << " dKd: " << dK[1] << " dKi: " << dK[2] << std::endl;
//              }
//            }
//            if (count == 2 * ITER) {
//              double cur_err = err / ITER;
//              if (cur_err < best_err) {
//                best_err = cur_err;
//                dK[i] *= 1.1;
//              }
//              else {
//                K[i] += dK[i]; // back to initial value
//                dK[i] *= 0.9;
//                steer_pid.Init(K[0], K[2], K[1]);
//                std::cout << "Kp: " << K[0] << " Kd: " << K[1] << " Ki: " << K[2] << std::endl;
//                std::cout << "dKp: " << dK[0] << " dKd: " << dK[1] << " dKi: " << dK[2] << std::endl;
//              }
//              count = 0;
//            }
//            // Make sure to pick the next coefficient next time
//            ++i;
//            i %= NUM_COEF;
//            err = 0;
//          }
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
//          err += pow(cte, 2);
          if (speed > 30) steer_pid.Init(0.1, 0.002, 4.0);
          steer_pid.UpdateError(cte);
          double error = steer_pid.TotalError();
          error = fmin(error, 1.0);
          error = fmax(error, -1.0);

          double steer_value = error;
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          double avg_cte = steer_pid.AverageError();
          // DEBUG
          std::cout <<  "Avg-CTE: " << avg_cte << " CTE: " << cte << " Returned steering angle: " << angle << " Speed: " << speed <<
              " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = THROTTLE;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
