#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

#define K1_CONST 0.01f
#define K2_CONST 0.01f

ActionModel::ActionModel(void)
: k1_(K1_CONST)
, k2_(K2_CONST)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////

    
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_) {
        previousPose_ = odometry;
        initialized_ = true;
    }
    // Q state covariance is a diagonal matrix
    // Q[0,0] = k1*abs(alpha)
    // Q[1,1] = k2*abs(delta(s))
    // Q[2,2] = k3*abs(dtheta - alpha)
    
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;
    trans_ = sqrt(dx_*dx_ + dy_*dy_); // delta(s)
    rot1_ = angle_diff(std::atan2(dy_,dx_),previousPose_.theta); // alpha


    float direction = 1.0; 
    if(std::abs(rot1_ > M_PI/2.0)){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1;
    }
    
    rot2_ = angle_diff(dtheta_,rot1_);



    moved_ = (dx_ != 0.0 || dy_ != 0.0 || dtheta_ != 0.0);
    if (moved_){
        rot1std_ = std::sqrt(k1_*std::abs(rot1_));
        transstd_ = std::sqrt(k2_*std::abs(trans_));
        rot2std_ = std::sqrt(k1_*std::abs(rot2_));
    }

    trans_ *= direction;
    previousPose_ = odometry;
    utime_ = odometry.utime;

    return moved_;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.


    // mbot_lcm_msgs::particle_t newSample = sample;
    mbot_lcm_msgs::particle_t newSample;

    float sampleEps1 = std::normal_distribution<>(0,rot1std_)(numberGenerator_);
    float sampleEps2 = std::normal_distribution<>(0,transstd_)(numberGenerator_);
    float sampleEps3 = std::normal_distribution<>(0,rot2std_)(numberGenerator_);

    newSample.pose.x = sample.pose.x + (trans_ + sampleEps2)*cos(sample.pose.theta + rot1_ + sampleEps1);
    newSample.pose.y = sample.pose.y + (trans_ + sampleEps2)*sin(sample.pose.theta + rot1_ + sampleEps1);
    newSample.pose.theta = sample.pose.theta + (dtheta_ + sampleEps1+ sampleEps3); // NOTE: if something goes wrong, try wrapping wrap_to_pi()


    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;


    return newSample;
}
