#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  int lane = 1;
  double ref_vel = 0;//mph
  bool changing_lane = false;
  int next_lane = 1;
  double traveled_in_lane = 0.;
  h.onMessage([&lane, &ref_vel, &changing_lane, &next_lane, &traveled_in_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

		  int prev_size = previous_path_x.size();

		
		  if (prev_size > 0) {

			  car_s = end_path_s;
			  car_d = end_path_d;
		  }
		  if ((next_lane != lane) && (car_d < (2 + 4 * next_lane + .2)) && (car_d > (2 + 4 * next_lane - .2)))
		  {
			  lane = next_lane;
			  changing_lane = false;
			  traveled_in_lane = 0;

		  }

		  // Keep the lane and adjust the speed
		  bool too_close = false;
		  double gap_left_lane_front_car = 30.;
		  double gap_left_lane_rear_car = 30.;
		  double gap_right_lane_front_car = 30.;
		  double gap_right_lane_rear_car = 30.;
		  double gap_my_lane_front_car = 30.;
		  double left_lane_speed = 100.;
		  double right_lane_speed = 100.;
		  double left_lane_rear_speed = 0.;
		  double right_lane_rear_speed = 0.;

		  double my_lane_speed = 100.;


		  int k = 0;
		  if (lane == 0) {
			  gap_left_lane_front_car = 0.;
			  gap_left_lane_rear_car = 0.;
		  }

		  if (lane == 2) {
			  gap_right_lane_front_car =0.;
			  gap_right_lane_rear_car = 0.;
		  }

		  traveled_in_lane += .02*car_speed;

		  for (int i = 0; i < sensor_fusion.size(); i++) 
		  {

			  float d = sensor_fusion[i][6];

			  double vx = sensor_fusion[i][3];
			  double vy = sensor_fusion[i][4];
			  double check_speed = sqrt(vx*vx + vy * vy);
			  double check_car_s = sensor_fusion[i][5];

			  // check this lane

					  check_car_s += (double)prev_size*.02*check_speed;
					  
					  double gap = check_car_s - car_s;
					  
					  if (fabs(gap) < 30.)
					  {
						  // check my lane
						//  if ((check_car_s > car_s) && (d < (2 + 4 * lane + 1)) && (d > (2 + 4 * lane - 1))) 
						if ((check_car_s > car_s) && (d < (car_d + 1)) && (d > (car_d - 1)))

						  {

							  if (gap < gap_my_lane_front_car) 
							  {
								  gap_my_lane_front_car = gap;
								  my_lane_speed = check_speed;
							  }
							  //ref_vel = check_speed * 2.24;
							  k += 1;
							  // std::cout << "There is car infront of you bitch!! " << std::endl;
							  too_close = true;
						  }

						  if ((lane > 0) && (d < (2 + 4 * (lane - 1) + 1)) && (d > (2 + 4 * (lane - 1) - 1)))
						  {

							  if ((gap > 0) && (gap < gap_left_lane_front_car))
							  {
								  gap_left_lane_front_car = gap;
								  left_lane_speed = check_speed;
							  }

							  if ((gap < 0) && ((-1.*gap) < gap_left_lane_rear_car))
							  {
								  gap_left_lane_rear_car = -1.*gap;
								  left_lane_rear_speed = check_speed;
							  }
						  }

						  if ((lane < 2) && (d < (2 + 4 * (lane + 1) + 1)) && (d > (2 + 4 * (lane + 1) - 1)))
						  {

							  if ((gap > 0) && (gap < gap_right_lane_front_car))
							  {
								  gap_right_lane_front_car = gap;
								  right_lane_speed = check_speed;
							  }

							  if ((gap < 0) && ((-1.*gap) < gap_right_lane_rear_car))
							  {
								  gap_right_lane_rear_car = -1.*gap;
								  right_lane_rear_speed = check_speed;
							  }

						  }
					  }
		  }
			
		   

		   bool right_lane_clear = false;
		   bool left_lane_clear = false;
		   bool go_left = false;
		   bool go_right = false;


		   double min_gap_right_lane = fmin(gap_right_lane_rear_car, gap_right_lane_front_car);
		   double min_gap_left_lane = fmin(gap_left_lane_rear_car, gap_left_lane_front_car);

		   
		   double thershold_lane_change = 10.0;

		   double time_change_lane = (4. / car_speed) + 2.;
		   double time_rear_left_hit_me = gap_left_lane_rear_car / left_lane_rear_speed;
		   double time_rear_right_hit_me = gap_right_lane_rear_car / right_lane_rear_speed;

		  if (too_close ) {
				  ref_vel -= .224;
			  if ((changing_lane == false) && (ref_vel > 10.) && (traveled_in_lane> 250.))
			  {
				  if ((min_gap_right_lane > thershold_lane_change) && (time_rear_right_hit_me > time_change_lane)){
					  right_lane_clear = true;
				  }
				  if ((min_gap_left_lane > thershold_lane_change) && (time_rear_left_hit_me > time_change_lane)) {
					  left_lane_clear = true;
				  }

				  if (right_lane_clear && left_lane_clear) {

					  if (min_gap_right_lane < min_gap_left_lane) {
						  if (my_lane_speed < left_lane_speed) {
							  go_left = true;
						  }else if (my_lane_speed < right_lane_speed) {
							  go_right = true;
						  }
					  }
					  else {
						  if (my_lane_speed < right_lane_speed) {
							  go_right = true;
						  }
					  }

				  }
				  else {
					  if (left_lane_clear) {
						  if (my_lane_speed < left_lane_speed) {
							  go_left = true;
						  }
					  }
					  if (right_lane_clear) {
						  if (my_lane_speed < right_lane_speed) {
							  go_right = true;
						  }
					  }
				  }
				  if (go_left && (lane > 0)) {
					  next_lane = lane - 1;
					  changing_lane = true;

				  }
				  if (go_right && (lane < 2)) {
					  next_lane = lane + 1;
					  changing_lane = true;
				  }
			  }


		  }
		  else if (ref_vel<49.5){
			  ref_vel += .224;
		  } 

		  //std::cout << "min gap right  is " << min_gap_right_lane << std::endl;
		  // std::cout << "min gap left is " << min_gap_left_lane << std::endl;
		  if (go_right)
		  {
			  std::cout << "going right is clear. min gap right  is " << min_gap_right_lane << std::endl;
			  std::cout << "traveled_in_lane  is " << traveled_in_lane << std::endl;

		  }
		  if (go_left)
		  {
			  std::cout << "going left is clear. min gap left is " << min_gap_left_lane << std::endl;
			  std::cout << "traveled_in_lane  is " << traveled_in_lane << std::endl;

		  }


			  
		  vector<double> ptsx;
		  vector<double> ptsy;

		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);

		  if (prev_size < 2) {
			  double prev_car_x = car_x - cos(ref_yaw);
			  double prev_car_y = car_y - sin(ref_yaw);
			  ptsx.push_back(prev_car_x);
			  ptsy.push_back(prev_car_y);

			  ptsx.push_back(car_x);
			  ptsy.push_back(car_y);

		  }
		  else {
			  ref_x = previous_path_x[prev_size - 1];
			  ref_y = previous_path_y[prev_size - 1];
			  
			  double ref_x_prev = previous_path_x[prev_size - 2];
			  double ref_y_prev = previous_path_y[prev_size - 2];

			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
			  
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);

			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
		  }
	

	
		  vector<double> next_wp0 = getXY(car_s + 30., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s + 60., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s + 90., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  
		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);

		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);

		  for (int i = 0; i < ptsx.size(); i++) {

			  double shift_x = ptsx[i] - ref_x;
			  double shift_y = ptsy[i] - ref_y;

			  ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		      ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
		  }
	

		 // std::cout << "ptsx size is " << ptsx.size() << std::endl;
		 // std::cout << "ptsy size is " << ptsy.size() << std::endl;


		  tk::spline s;
		  s.set_points(ptsx, ptsy);



          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  vector<double> next_x_vals;
		  vector<double> next_y_vals;

		 // std::cout << "previous_path_x size is : " << previous_path_x.size() << std::endl;

		  for (int i = 0; i < previous_path_x.size(); ++i) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		 

		  double target_x = 30.;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

		  double x_add_on = 0;
		  double N = (target_dist / (0.02*ref_vel / 2.24));
		  
		//  std::cout << "current path size  is " << 50 - previous_path_x.size() << std::endl;

		  for (int i = 1; i < 50 - previous_path_x.size(); i++) {

			  double x_point = x_add_on + (target_x) / N;
			  double y_point = s(x_point);
			  x_add_on = x_point;

			  double shift_x = x_point;
			  double shift_y = y_point;

			  x_point = (shift_x * cos(ref_yaw) - shift_y * sin(ref_yaw));
			  y_point = (shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw));

			  x_point += ref_x;
			  y_point += ref_y;

			  next_x_vals.push_back(x_point);
			  next_y_vals.push_back(y_point);
		  }

		  int sz = next_x_vals.size();
		 // std::cout << "car next_x_vals size is :" << sz << std::endl;


		  /*

		  std::cout << "car xval is :" << car_x << std::endl;
		  std::cout << "car yval is :" << car_y << std::endl;

		  std::cout << "first xval is :" << next_x_vals[0] << std::endl;
		  std::cout << "final yval is :" << next_y_vals[0] << std::endl;

		  std::cout << "final xval is :" << next_x_vals[sz-1] << std::endl;
		  std::cout << "final yval is :" << next_y_vals[sz-1] << std::endl;

		  */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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