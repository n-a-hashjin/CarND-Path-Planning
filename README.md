# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
![](./data/result.gif)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this we store the last points which have been used so the car can have a smooth transition. previous_path_x, and previous_path_y are used for this transition since they show the last points given to the simulator controller with the processed points already removed. We return a path that extends this previous path.

## spline

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file added to source directory.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Path planning strategy

The Car starts in the middle lane at the begining and speeds up until it gets to the slightly less than the maximum allowed speed, namely 50 mph. If there is a car ahead of the car and its speed is below our car, it checks possible lane change to maintain maximum speed and if it is not possible, reduces its speed to the speed of the car ahead of it. The preferred lane is middle lane, so if the middle lane is safe and faster to drive, car will change to it. It allows us to have more possiblity to select faster and safer action in next comming situation. The corresponding code lines to implement this strategy (a finite state machine) is in path_planner.cpp line 116 to line 137.

```
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
```

## Prediction

It is considered to predict the position of other traffic participents ahead of time, in order to make decision base of lane speeds and colision avoidance using sensor fusion data. We make a list of cars for each line that in near future will be inbetween 5 meters behind and 40 meters ahead of our self driving car. The speed of nearst car ahead of our car in each lane will be save simultaneuosly to track that speed in case we can not pass through the traffic. The corresponding code is from line 48 to line 93 in path_planner.cpp.

```
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
```

## Trajectory generator

We have used spline for generating the path we planned to pass. The code can be addressed in path_planning.cpp file from line 139 to line 235.