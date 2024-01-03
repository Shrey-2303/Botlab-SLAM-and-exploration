#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include "mbot_lcm_msgs/path2D_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#define M_PI 3.1415927

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

    insertPoint(0,0,0,path.path);

    insertPoint(0.61,0,M_PI/2,path.path);
    insertPoint(0.61,0.61,0,path.path);
    insertPoint(1.22,0.61,-M_PI/2,path.path);
    insertPoint(1.22,0,0,path.path);
    insertPoint(1.83,0,M_PI/2,path.path);
    insertPoint(1.83,1.22,-M_PI,path.path);
    insertPoint(0,1.22,-M_PI/2,path.path);
    insertPoint(-0.09,0,0.03,path.path);

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to sexy: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}