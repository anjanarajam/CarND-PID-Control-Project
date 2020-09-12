#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

static constexpr int MAX_SPEED = 100;
static constexpr double MAX_THROTTLE = 0.3;

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

  /* Get object for pid controller for speed */
  PID steer_pid;
  /* Get object for twiddle algorithm */
  Twiddle twiddle;

  /* Initialize the steering and throttle pid variable */
  steer_pid.Init(0.25, 0.001, 3.0);

  h.onMessage([&steer_pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
			// j[1] is the data JSON object
			double cte = std::stod(j[1]["cte"].get<string>());
			double speed = std::stod(j[1]["speed"].get<string>());
			double angle = std::stod(j[1]["steering_angle"].get<string>());
			double steer_value = 0.0;
			double throttle_value = MAX_THROTTLE;

			/* Calculate steering value here, */

			/* Find the individual P , I and D steering value errors wrt cross track error */
			steer_pid.UpdateError(cte);

			/* Find the total error(steering value). remember the steering value is [-1, 1] */
			steer_value = -(steer_pid.TotalError());
			if (steer_value < -1) {
				steer_value = -1;
			}
			else if (steer_value > 1) {
				steer_value = 1;
			}

			/* Print the values */
			std::cout << "CTE: " << cte << " Steering Value: " << steer_value
		    << "throttle_error:" << throttle_value << std::endl;

			/* Send the values to the server */
			json msgJson;
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = throttle_value;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			std::cout << msg << std::endl;
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  
			/* Twiddle for automated tuning of the parameters */
			twiddle.IncrementSteps();
			twiddle.AccumulateError(cte);
			twiddle.CalculateAverageError();

			if (twiddle.reached_count_) {
				/* Twiddle to fine tune the parameters in automation */
				twiddle.PerformTwiddle(steer_pid);

				steer_pid.Init(steer_pid.Kp_, steer_pid.Ki_, steer_pid.Kd_);
				twiddle.PrintValues();

				std::string msg("42[\"reset\", {}]");
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}

		}// end "telemetry" if

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

