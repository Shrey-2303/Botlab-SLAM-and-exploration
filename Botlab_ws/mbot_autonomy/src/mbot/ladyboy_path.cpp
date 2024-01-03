#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include "mbot_lcm_msgs/path2D_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#define M_PI 3.1415927

std::vector<std::vector<double>> hard_path = {{0.00, 0.00},
 {0.30, 0.00}, {0.30, 0.00}, {0.37, 0.01}, {0.44, 0.03}, {0.49, 0.07},
 {0.54, 0.11}, {0.58, 0.17}, {0.60, 0.24}, {0.61, 0.30}, {0.61, 0.30},
 {0.62, 0.37}, {0.64, 0.44}, {0.68, 0.49}, {0.72, 0.54}, {0.78, 0.58},
 {0.85, 0.60}, {0.91, 0.61}, {0.91, 0.61}, {0.98, 0.60}, {1.05, 0.58},
 {1.10, 0.54}, {1.15, 0.49}, {1.19, 0.44}, {1.21, 0.37}, {1.22, 0.30},
 {1.22, 0.30}, {1.23, 0.24}, {1.25, 0.17}, {1.28, 0.11}, {1.33, 0.07},
 {1.39, 0.03}, {1.46, 0.01}, {1.52, 0.00}, {1.83, 0.00}, {1.83, 0.91},
 {1.83, 0.91}, {1.82, 0.98}, {1.80, 1.05}, {1.76, 1.10}, {1.71, 1.15},
 {1.66, 1.19}, {1.59, 1.21}, {1.52, 1.22}, {0.30, 1.22}, {0.30, 1.22},
 {0.24, 1.21}, {0.17, 1.19}, {0.11, 1.15}, {0.07, 1.10}, {0.03, 1.05},
 {0.01, 0.98}, {0.00, 0.91}, {-0.09,0,0.03}};

void insertPoint(double x, double y, double theta, std::vector<mbot_lcm_msgs::pose2D_t>& path)
{
    mbot_lcm_msgs::pose2D_t nextPose;
    //forward
    nextPose.x = x;
    nextPose.y = y;
    nextPose.theta = theta;

    path.push_back(nextPose);
}
int main(int argc, char** argv)
{
    mbot_lcm_msgs::path2D_t path;
    // path.path.resize(8);

    // insertPoint(0,0,0,path.path);

    // insertPoint(0.61,0,0,path.path);
    // insertPoint(0.61,0.61,M_PI/2,path.path);
    // insertPoint(1.22,0.61,0,path.path);
    // insertPoint(1.22,0,-M_PI/2,path.path);
    // insertPoint(1.83,0,0,path.path);
    // insertPoint(1.83,1.22,M_PI/2,path.path);
    // insertPoint(0,1.22,-M_PI,path.path);
    // insertPoint(0,0,-M_PI/2,path.path);
    for (size_t i = 0; i < hard_path.size(); ++i) {
        double x = hard_path[i][0];
        double y = hard_path[i][1];
        double theta = 0;
        insertPoint(x, y, theta, path.path);
    }

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to lady boy: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}