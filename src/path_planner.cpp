#include <string>
#include <fstream>
#include <iostream>
#include <math.h>
#include "path_planner.h"
#include "json.hpp"
#include "helpers.h"
#include "spline.h"

PathPlanner::PathPlanner(const char *csv_file)
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::ifstream in_map_(csv_file, std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
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
        _waypoints.x.push_back(x);
        _waypoints.y.push_back(y);
        _waypoints.s.push_back(s);
        _waypoints.dx.push_back(d_x);
        _waypoints.dy.push_back(d_y);
    }
}


nlohmann::json PathPlanner::ProcessTelemetry(nlohmann::json data)
{
    // Main car's localization Data
    Vehicle ego;
    ego.x = data["x"];
    ego.y = data["y"];
    ego.s = data["s"];
    ego.d = data["d"];
    ego.yaw = deg2rad((double)data["yaw"]);
    ego.speed = mph2mps((double)data["speed"]);

    // Previous path data given to the Planner
    auto previous_path_x = data["previous_path_x"];
    auto previous_path_y = data["previous_path_y"];
    // Previous path's end s and d values 
    double end_path_s = data["end_path_s"];
    double end_path_d = data["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side 
    //   of the road.
    auto sensor_fusion = data["sensor_fusion"];

    std::vector<Vehicle> vehicles;
    for (auto sensor_data: sensor_fusion)
    {
        Vehicle vehicle;
        //vehicle.x = sensor_data[1];
        //vehicle.y = sensor_data[2];
        vehicle.s = sensor_data[5];
        vehicle.d = sensor_data[6];
        float vx = sensor_data[3];
        float vy = sensor_data[4];
        vehicle.speed = sqrt(vx*vx + vy*vy);

        vehicles.push_back(vehicle);
    }


    auto prev_size = previous_path_x.size();

    if (prev_size > 0)
    {
        ego.s = end_path_s;
    }

    bool too_close = false;
    bool lane_busy[3] = {false,};

    for (auto vehicle : vehicles)
    {
        vehicle.s += 0.02 * prev_size * vehicle.speed;

        // Vehicle on a different lane
        if (vehicle.lane() != lane)
        {
            if ((vehicle.isInFrontOf(ego) and vehicle.distanceTo(ego) < 40) or
              (not vehicle.isInFrontOf(ego) and vehicle.distanceTo(ego) < 10))
            {
              lane_busy[vehicle.lane()] = true;
            }

            continue;
        }

        // Vehicle in front of us
        if (vehicle.isInFrontOf(ego) and vehicle.distanceTo(ego) < 30)
        {
            too_close = true;
        }
    }

    if (too_close)
    {
        // Slowly break
        ref_velocity -= 0.224;
        if (lane > 0 and not lane_busy[lane-1])
            lane--;
        else if (lane < 2 and not lane_busy[lane+1])
            lane++;
    }
    else if (ref_velocity < SPEED_LIMIT_MPH)
    {
        // Accelerate
        ref_velocity += 0.224;
    }
    else
    {
        // Go back to the middle lane
        if (not lane_busy[1])
            lane = 1;
    }

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in with more points 
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x, y, yaw states
    // either we will reference the starting point as where the car is or at the previous paths end point
    double ref_x = ego.x;
    double ref_y = ego.y;
    double ref_yaw = ego.yaw;

    // if previous size is almost empty, use the car the the starting reference
    if (prev_size < 2)
    {
        // Use two points that make the path tangent to the car
        double prev_car_x = ego.x - cos(ego.yaw);
        double prev_car_y = ego.y - sin(ego.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ego.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ego.y);
    }
    else
    {
        // Redefine reference state as previous path end point
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    double target_d = 2 + 4*lane;

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(ego.s + 30, target_d, _waypoints.s, _waypoints.x, _waypoints.y);
    vector<double> next_wp1 = getXY(ego.s + 60, target_d, _waypoints.s, _waypoints.x, _waypoints.y);
    vector<double> next_wp2 = getXY(ego.s + 90, target_d, _waypoints.s, _waypoints.x, _waypoints.y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        // shift car reference angle to 0 degree
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }

    // create a spline
    tk::spline s;

    // set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Define the actual (x,y) points we will use for the planner 
    vector<double> next_x_vals;
    vector<double> next_y_vals;


    // start with the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for (int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        double N = target_dist / (0.02 * mph2mps(ref_velocity));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotating back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }


    nlohmann::json msgJson;

    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    return msgJson;
}



