/**
 * implementation of path planner class
*/
#include "path_planner.h"


PathPlanner::PathPlanner(std::string mapFile)
{
    map_ = loadMap(mapFile);
    lane_ = 1;
    vel_ = 0.0;
    ref_vel_ = 49.5;
    delta_t_ = 0.02;
}

PathPlanner::~PathPlanner()
{
}

vector<vector<double>> PathPlanner::getTrajectory(nlohmann::json& j)
{
    /// j[1] is the data JSON object   
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

    int prev_size = previous_path_x.size();

    if (prev_size > 0)
    {
        car_s = end_path_s;
    }
    vector<vector<double>> left_lane_traffic;
    vector<vector<double>> middle_lane_traffic;
    vector<vector<double>> right_lane_traffic;
    for (int i=0; i<sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += (double)prev_size*0.02*check_speed;
        vector<double> check_car = {check_car_s, check_speed};
        if ((check_car_s+5 > car_s) && ((check_car_s-car_s) < 40))
        {
            if (d>0 && d<4) left_lane_traffic.push_back(check_car);
            else if (d>4 && d<8) middle_lane_traffic.push_back(check_car);
            else if(d>8 && d<12) right_lane_traffic.push_back(check_car);
        }

    }
    vector<double> lane_speed{ref_vel_,ref_vel_,ref_vel_};
    bool left_lane = 0;
    bool middle_lane = 0;
    bool right_lane = 0; 
    if (!left_lane_traffic.empty())
    {
        std::sort (left_lane_traffic.begin(), left_lane_traffic.end(),
                [](vector<double> a, vector<double> b){ return (a[0] < b[0]); });
        left_lane = 1;
        lane_speed[0] = left_lane_traffic[0][1]*2.24;
    }
    if (!middle_lane_traffic.empty())
    {
        std::sort (middle_lane_traffic.begin(), middle_lane_traffic.end(),
                [](vector<double> a, vector<double> b){ return (a[0] < b[0]); });
        middle_lane = 1;
        lane_speed[1] = middle_lane_traffic[0][1]*2.24;
    }
    if (!right_lane_traffic.empty())
    {
        std::sort (right_lane_traffic.begin(), right_lane_traffic.end(),
                [](vector<double> a, vector<double> b){ return (a[0] < b[0]); });
        right_lane = 1;
        lane_speed[2] = right_lane_traffic[0][1]*2.24;
    }
  
    bool too_close = false;
    for (int i=0; i<sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        if (d < (2+4*lane_+2) && d > (2+4*lane_-2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            check_car_s += (double)prev_size*0.02*check_speed;

            if ((check_car_s > car_s) && ((check_car_s-car_s) < 25))
            {
                too_close = true;
            }
        }
    }
    
    
    if (too_close)
    {
        if (lane_==0) {
            if (!middle_lane) lane_++;
            else if (vel_>lane_speed[0]) vel_ -= 0.5; //0.224
        }
        else if (lane_==1) {
            if (!left_lane) lane_ = 0;
            else if (!right_lane) lane_ = 2;
            else if (vel_>lane_speed[1]) vel_ -= 0.5; //0.224
        }
        else {
            if (!middle_lane) lane_--;
            else if (vel_>lane_speed[2]) vel_ -= 0.5; //0.224
        }
    } else if (vel_ < ref_vel_)
    {
        vel_ += 0.32; //0.224
        
    } else if (lane_ != 1 && !middle_lane) {
       lane_ = 1;
    }

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    // if previous size is almost empty, use the car as starting reference
    if (prev_size < 2)
    {
        //Use two points that make the path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);

    }

    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane_), map_.pts_s, map_.pts_x, map_.pts_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane_), map_.pts_s, map_.pts_x, map_.pts_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane_), map_.pts_s, map_.pts_x, map_.pts_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (size_t i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

    // create a spline
    tk::spline s;

    // set (x, y) points to the spline
    s.set_points(ptsx, ptsy);

    // set actual (x, y) pointswe will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (size_t i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;

    for (size_t i = 1; i <= 50-previous_path_x.size(); i++) {
        double N = target_dist/(delta_t_*vel_/2.24);
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal
        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return {next_x_vals, next_y_vals};
}