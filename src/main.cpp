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
		  // chack if the car finish the lane changing process.
		  if ((next_lane != lane) && (car_d < (2 + 4 * next_lane + .2)) && (car_d > (2 + 4 * next_lane - .2)))
		  {
			  lane = next_lane;
			  changing_lane = false;
			  traveled_in_lane = 0;

		  }

		  // Initilize the following condition
		  bool too_close = false;
		  double distance2_left_lane_front_car = 200.;
		  double distance2_left_lane_rear_car = 200.;
		  double distance2_right_lane_front_car = 200.;
		  double distance2_right_lane_rear_car = 200.;
		  double distance2_my_lane_front_car = 200.;
		  double left_lane_speed = 100.;
		  double right_lane_speed = 100.;
		  double left_lane_rear_speed = 0.;
		  double right_lane_rear_speed = 0.;

		  double my_lane_speed = 100.;


		  int k = 0;
		  if (lane == 0) {
			  distance2_left_lane_front_car = 0.;
			  distance2_left_lane_rear_car = 0.;
		  }

		  if (lane == 2) {
			  distance2_right_lane_front_car =0.;
			  distance2_right_lane_rear_car = 0.;
		  }

		  // keeo track of distance car traveled  
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
					  
					  if (fabs(gap) < 200.)
					  {
						 // check my lane and decide if car disatance to the next car is too close in this lane
						if ((fabs(gap) < 40.) && (check_car_s > car_s) && (d < (car_d + 1.5)) && (d > (car_d - 1.5)))

						  {

							  if (gap < distance2_my_lane_front_car) 
							  {
								  distance2_my_lane_front_car = gap;
								  my_lane_speed = check_speed;
							  }
							  too_close = true;
						  }

						// check left lane and find discante 2 closet front and rear cars in the left lane

						  if ((lane > 0) && (d < (2 + 4 * (lane - 1) + 2)) && (d > (2 + 4 * (lane - 1) - 2)))
						  {

							  if ((gap > 0) && (gap < distance2_left_lane_front_car))
							  {
								  distance2_left_lane_front_car = gap;
								  left_lane_speed = check_speed;
							  }

							  if ((gap < 0) && ((-1.*gap) < distance2_left_lane_rear_car))
							  {
								  distance2_left_lane_rear_car = -1.*gap;
								  left_lane_rear_speed = check_speed;
							  }
						  }
						  // check right lane and find discante 2 closet front and rear cars in the right lane

						  if ((lane < 2) && (d < (2 + 4 * (lane + 1) + 2)) && (d > (2 + 4 * (lane + 1) - 2)))
						  {

							  if ((gap > 0) && (gap < distance2_right_lane_front_car))
							  {
								  distance2_right_lane_front_car = gap;
								  right_lane_speed = check_speed;
							  }

							  if ((gap < 0) && ((-1.*gap) < distance2_right_lane_rear_car))
							  {
								  distance2_right_lane_rear_car = -1.*gap;
								  right_lane_rear_speed = check_speed;
							  }

						  }
					  }
		  }
			
		   
		  // initialize lane change logical conditions
		   bool right_lane_clear = false;
		   bool left_lane_clear = false;
		   bool go_left = false;
		   bool go_right = false;
		   bool turn_left_speed_match = false;
		   bool turn_right_speed_match = false;

		   // find min distance to the cars in the left and right lanes

		   double min_distance2_right_lane = fmin(distance2_right_lane_rear_car, distance2_right_lane_front_car);
		   double min_distance2_left_lane = fmin(distance2_left_lane_rear_car, distance2_left_lane_front_car);

		   // set the  thershold gap to change the lane 
		   double thershold_lane_change = 12.0;

		   // calculte the time ego car needs to change lane 

		   double time_change_lane = (4. / car_speed) + 1.5;

		   // calculte the time rear left car needs to catch-up 

		   double time_rear_left_hit_me = distance2_left_lane_rear_car / left_lane_rear_speed;

		   // calculte the time rear right car needs to catch-up 

		   double time_rear_right_hit_me = distance2_right_lane_rear_car / right_lane_rear_speed;

		  
		   // chack if speed matches to change lane to the left
		   if (((my_lane_speed*.8 < left_lane_speed) && 
			   (car_speed * .7 < left_lane_speed) && 
			   (car_speed * 1.3 > left_lane_rear_speed)) ||
			   (min_distance2_left_lane > 2.5* thershold_lane_change)) {
			   turn_left_speed_match = true;
		   }

		   // chack if speed matches to change lane to the right
		   if (((my_lane_speed *.8 < right_lane_speed) && 
			   (car_speed *.7 < right_lane_speed) && 
			   (car_speed * 1.3 > right_lane_rear_speed)) ||
			   (min_distance2_right_lane > 2.5* thershold_lane_change)) {
			   turn_right_speed_match = true;
		   }

		   // keep the lane and speed unless car is too closed to the next one
		  if (too_close ) {
			  if ((my_lane_speed * 1.3 < car_speed) || (distance2_my_lane_front_car < 7.))
			  { 
				  ref_vel -= .224;
			  }

			  // consider the change lane if 
			  // 1 - car is NOT already changing lane
			  // 2 - its speed is 25 mph 
			  // 3 - Car is already travled more than 250 m in its current lane
			  if ((changing_lane == false) && (ref_vel > 25.) && (traveled_in_lane> 250.))
			  {

				  // check if left lane is clear
				  if ((min_distance2_right_lane > thershold_lane_change) && (time_rear_right_hit_me > time_change_lane)){
					  right_lane_clear = true;
				  }

				  // check if right lane is clear
				  if ((min_distance2_left_lane > thershold_lane_change) && (time_rear_left_hit_me > time_change_lane)) {
					  left_lane_clear = true;
				  }
				  // check if both lane clear
				  if (right_lane_clear && left_lane_clear) {
					  // choose lane with bigger gap
					  if (distance2_right_lane_front_car < distance2_left_lane_front_car) {
						  // check if speed matches for left or right lane change
						  if (turn_left_speed_match) {
							  go_left = true;
						  }else if (turn_right_speed_match) {
							  go_right = true;
						  }
					  }
					  else {
						  // check if speed matches for right lane change
						  if (turn_right_speed_match) {
							  go_right = true;
						  }
					  }

				  }
				  else {
					  // check if left lane is clear
					  if (left_lane_clear) {
						  if (turn_left_speed_match) {
							  go_left = true;
						  }
					  }
					  // check if right lane is clear
					  if (right_lane_clear) {
						  if (turn_right_speed_match) {
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
			  ref_vel += .224/1.5;
		  } 

		  //std::cout << "min gap right  is " << min_distance2_right_lane << std::endl;
		  // std::cout << "min gap left is " << min_distance2_left_lane << std::endl;
		  if (go_right)
		  {
			  std::cout << "----------------------------------------------------------" << std::endl;
			  std::cout << "Changing Right Lane. Min dist. to Right-lane Car:"  << min_distance2_right_lane << std::endl;
			  std::cout << "Travelled distance in the current lane was :" << traveled_in_lane << std::endl;

		  }
		  if (go_left)
		  {
			  std::cout << "----------------------------------------------------------" << std::endl;
			  std::cout << "Changing Left Lane. Min dist. to Left-lane Car:" << min_distance2_left_lane << std::endl;
			  std::cout << "Travelled distance in the current lane was " << traveled_in_lane << std::endl;

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
		  vector<double> next_wp0;
		  vector<double> next_wp1;
		  vector<double> next_wp2;
	
		  // use shorter anchor for keeping the lane
		  if (next_lane == lane){
			  next_wp0 = getXY(car_s + 20., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  next_wp1 = getXY(car_s + 40., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  next_wp2 = getXY(car_s + 100., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  }else{
			  // use longer anchor for changing the lane
			  next_wp0 = getXY(car_s + 40., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  next_wp1 = getXY(car_s + 60., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  next_wp2 = getXY(car_s + 120., (2 + 4 * next_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  }
			  
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
	



		  tk::spline s;
		  s.set_points(ptsx, ptsy);


		  vector<double> next_x_vals;
		  vector<double> next_y_vals;

		 // std::cout << "previous_path_x size is : " << previous_path_x.size() << std::endl;

		  for (int i = 0; i < previous_path_x.size(); ++i) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		 

		  double target_x = 40.;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

		  double x_add_on = 0;
		  double N = (target_dist / (0.02*ref_vel / 2.24));
		  

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