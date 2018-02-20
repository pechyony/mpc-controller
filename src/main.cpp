#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string> 
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

using namespace std;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char* argv[]) {
  uWS::Hub h;

  unsigned int N = 5;
  float dt = 0.11;
  float C_cte = 200;
  float C_epsi = 200;
  float C_v = 5;
  float C_delta = 100;
  float C_a = 100;
  float C_delta_diff = 200;
  float C_a_diff = 10;
  float C_slowdown = 500;

  if (argc == 11) {
    N = stoi(argv[1]);
    dt = stof(argv[2]);
    C_cte = stof(argv[3]);
    C_epsi = stof(argv[4]);
    C_v = stof(argv[5]);
    C_delta = stof(argv[6]);
    C_a = stof(argv[7]);
    C_delta_diff = stof(argv[8]);
    C_a_diff = stof(argv[9]);
    C_slowdown = stof(argv[10]);
  }

  // MPC is initialized here!
  MPC mpc(N, dt, C_cte, C_epsi, C_v, C_delta, C_a, C_delta_diff, C_a_diff, C_slowdown);
  int n = 1;
  float ave = 0;
  std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();

  h.onMessage([&mpc, &prev_time, &n, &ave]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // convert waypoints from global to car coordinate system
          // car coordinate system: (0,0) is at car's position, x axis points to psi direction, y axis 90 degrees to the left
          for (int i=0; i < ptsx.size(); i++) {
            // shift coordinates of waypoint 
            double ptsx_shifted = ptsx[i] - px;
            double ptsy_shifted = ptsy[i] - py;

            // rotate coordinates shifted coordinates of waypoint by -psi angle 
            ptsx[i] = ptsx_shifted * cos(-psi) - ptsy_shifted * sin(-psi);
            ptsy[i] = ptsx_shifted * sin(-psi) + ptsy_shifted * cos(-psi);
          }

          // convert waypoints to Eigen::VectorXd, since polyfit function accepts input in this format
          double *ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_car(ptrx, ptsx.size());
          double *ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_car(ptry, ptsy.size());

          // fit 3rd-degree polynomial to the the waypoints
          auto coeffs = polyfit(ptsx_car, ptsy_car, 2); //3

          // compute cross-track error (cte)
          double cte = polyeval(coeffs,0);   // this is an approximate computation

          // compute error in psi (epsi)
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // .. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i = 2; i < vars.size(); i++) { // the first two elements of vars vector are actuator values,
                                                  // next position coordinates start from the third element of vars
            if (i % 2 == 0)
                mpc_x_vals.push_back(vars[i]);
            else
                mpc_y_vals.push_back(vars[i]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (int i=1; i < ptsx.size(); i++) {
            next_x_vals.push_back(ptsx[i]);
            next_y_vals.push_back(polyeval(coeffs,ptsx[i]));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
