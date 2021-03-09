#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <iostream>
using std::cout;
using std::endl;

/**
 * The class for path planning
 */
class PathPlanner
{
private:
    // loaded map 
    Map map_;
    // lane of the car
    int lane_;
    // velocity of the car
    double vel_;
    // max allowed speed
    double ref_vel_;
    // time interval between 2 generated path points
    double delta_t_;
public:
    /**
    * Constructor
    * @param mapFile a string of path of map file 
    */
    PathPlanner(string mapFile);
    /**
    * Destructor
    */
    ~PathPlanner();
    /**
     * Trajectory generator
     * @param j nlohmann::json refernce containing car, path history and sensor fusion data
     */
    vector<vector<double>> getTrajectory(nlohmann::json& j);
};

