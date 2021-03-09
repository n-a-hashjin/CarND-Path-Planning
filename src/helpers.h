#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>


// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If the[re is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

                         // Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);

// Map data container
struct Map {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> pts_x;
  vector<double> pts_y;
  vector<double> pts_s;
  vector<double> pts_dx;
  vector<double> pts_dy;
};

// Load map from .csv file
Map loadMap(string mapFile);

// Load state from .txt file
vector<vector<double> > Load_State(string file_name);

// Load labels from .txt file
vector<string> Load_Label(string file_name);

#endif  // HELPERS_H