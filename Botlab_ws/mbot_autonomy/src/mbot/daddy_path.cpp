#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include "mbot_lcm_msgs/path2D_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    mbot_lcm_msgs::path2D_t path;
    // path.path.resize(8);

    mbot_lcm_msgs::pose2D_t nextPose;
    //forward
    nextPose.x = 0.61;
    nextPose.y = 0;
    nextPose.theta = 0;

    path.path.push_back(nextPose);

    //right
    nextPose.x = 0.61;
    nextPose.y = -0.61;
    nextPose.theta = 0;

    path.path.push_back(nextPose);
    
    //forward
    nextPose.x = 0.61*2;
    nextPose.y = -0.61;
    nextPose.theta = 0;

    path.path.push_back(nextPose);
    
    //left
    nextPose.x = 0.61*2;
    nextPose.y = 0.61;
    nextPose.theta = 0;

    
    path.path.push_back(nextPose);
    
    //forward
    nextPose.x = 0.61*3;
    nextPose.y = 0.61;
    nextPose.theta = 0;

    path.path.push_back(nextPose);

    
    //right
    nextPose.x = 0.61*3;
    nextPose.y = -0.61;
    nextPose.theta = 0;
    
    path.path.push_back(nextPose);
    
    //forward
    nextPose.x = 0.61*4;
    nextPose.y = -0.61;
    nextPose.theta = 0;

    path.path.push_back(nextPose);

    //left
    nextPose.x = 0.61*4;
    nextPose.y = 0;
    nextPose.theta = 0;

    path.path.push_back(nextPose);
    

    //forward
    nextPose.x = 0.61*5;
    nextPose.y = 0;
    nextPose.theta = 0;

    path.path.push_back(nextPose);

    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to sexy: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}