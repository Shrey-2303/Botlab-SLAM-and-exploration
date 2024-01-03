#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/timestamp_t.hpp>
#include <mbot_lcm_msgs/mbot_message_received_t.hpp>
#include <mbot_lcm_msgs/mbot_slam_reset_t.hpp>
#include <utils/timestamp.h>
#include <utils/geometric/angle_functions.hpp>
#include <utils/geometric/pose_trace.hpp>
#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <cassert>
#include <cmath>

#define APRILTAG_CHANNEL "APRILTAG"
#define SLAM_POSE_CHANNEL "SLAM_POSE"
#define CONTROLLER_PATH_CHANNEL "CONTROLLER_PATH"
std::vector<mbot_lcm_msgs::pose2D_t> spline(mbot_lcm_msgs::pose2D_t p0, mbot_lcm_msgs::pose2D_t p1, double step)
{
    double slope_xi, slope_xf;
    double slope_yi, slope_yf;
    double mag = std::sqrt(std::pow(p1.x - p0.x,2) + std::pow(p1.y-p0.y,2));

    slope_xi = std::cos(p0.theta)*mag;
    slope_xf = std::cos(p1.theta)*mag;
    slope_yi = std::sin(p0.theta)*mag;
    slope_yf = std::sin(p1.theta)*mag;

    //calc cubic spline constants
    double ax = 2 * p0.x - 2 * p1.x + slope_xf + slope_xi;
    double bx = -2 * slope_xi - slope_xf - 3 * p0.x + 3 * p1.x;
    double cx = slope_xi;
    double dx = p0.x;

    double ay = 2 * p0.y - 2 * p1.y + slope_yf + slope_yi;
    double by = -2 * slope_yi - slope_yf - 3 * p0.y + 3 * p1.y;
    double cy = slope_yi;
    double dy = p0.y;


    std::vector<mbot_lcm_msgs::pose2D_t> pts;
    for (double t = 0; t <= 1; t+=step)
    {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = ax*(t*t*t) + bx*(t*t) + cx*t + dx;
        pose.y = ay*(t*t*t) + by*(t*t) + cy*t + dy;
        pose.theta = 0;
        pts.push_back(pose);
    }
    pts.at(0).theta = p0.theta;
    pts.at(pts.size()-1).theta = p1.theta;
    printf("pts0: (%f,%f)\n", pts.at(0).x, pts.at(0).y);
    printf("pts1: (%f,%f)\n", pts.at(pts.size()-1).x, pts.at(pts.size()-1).y);


    return pts;
}

class Bitches
{
    private:
    lcm::LCM* lcmInstance;
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    mbot_lcm_msgs::pose2D_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    bool following_path;
    
    public:
    Bitches(lcm::LCM* instance)
    {
        lcmInstance = instance;
        lcmInstance->subscribe(APRILTAG_CHANNEL, &Bitches::handleTag, this);
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &Bitches::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &Bitches::handlePose, this);
        following_path = false;


        odomToGlobalFrame_.x = 0;
        odomToGlobalFrame_.y = 0;
        odomToGlobalFrame_.theta = 0;
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose)
    {
        computeOdometryOffset(*pose);
    }
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* odometry)
    {
        mbot_lcm_msgs::pose2D_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }

    void computeOdometryOffset(const mbot_lcm_msgs::pose2D_t& globalPose)
    {
        mbot_lcm_msgs::pose2D_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
         
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated; 
        odomToGlobalFrame_.theta = deltaTheta;
    }

    mbot_lcm_msgs::pose2D_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        mbot_lcm_msgs::pose2D_t odomPose = odomTrace_.back();
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        
        return pose;
    }

    void handleTag(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* apriltag_msg)
    {
        /**
         * This data is given relative to robot frame
        */

        
        
        int64_t id = apriltag_msg->utime; 
        // printf("id: %ld\n", id);

        //if pose data available
        if ( (id == 5 || id == 50) && !odomTrace_.empty() && !following_path)
        {
            //tag pose defined in robot frame
            mbot_lcm_msgs::pose2D_t tag_pose;
            tag_pose.x = apriltag_msg->x;
            tag_pose.y = apriltag_msg->y;
            tag_pose.theta = apriltag_msg->theta*M_PI/180;

            //robot pose
            mbot_lcm_msgs::pose2D_t pose = currentPose();

            //get tag_pose as global frame
            // pose_global = R_robot * (p_local) + t_robot
            mbot_lcm_msgs::pose2D_t tag_pose_global;
            tag_pose_global.x = (std::cos(pose.theta)*tag_pose.x - std::sin(pose.theta)*tag_pose.y) + pose.x;
            tag_pose_global.y = (std::sin(pose.theta)*tag_pose.x + std::sin(pose.theta)*tag_pose.y) + pose.y;
            tag_pose_global.theta = wrap_to_2pi(tag_pose.theta + pose.theta);


            printf("RobotFrame:\nx,y,theta: (%.4f,%.4f,%.4f)\n", tag_pose.x, tag_pose.y, tag_pose.theta*180/M_PI);
            printf("GlobalFrame:\nx,y,theta: (%.4f,%.4f,%.4f)\n", tag_pose_global.x, tag_pose_global.y, tag_pose_global.theta*180/M_PI);
            printf("RobotPose:\nx,y,theta: (%.4f,%.4f,%.4f)\n\n", pose.x, pose.y, pose.theta*180/M_PI);


            //offset 
            mbot_lcm_msgs::pose2D_t tag_offset;
            tag_offset.x = tag_pose_global.x - std::cos(tag_pose_global.theta)*0.3;
            tag_offset.y = tag_pose_global.y - std::sin(tag_pose_global.theta)*0.3;
            tag_offset.theta = tag_pose_global.theta;

            std::vector<mbot_lcm_msgs::pose2D_t> pts;
            mbot_lcm_msgs::path2D_t path;
            pts.push_back(pose);
            pts.push_back(tag_offset);
            path.path = pts; 
            //path.path = spline(pose, tag_offset, 0.05);
            
            
            path.path_length = path.path.size();
            path.utime = utime_now();
            lcmInstance->publish(CONTROLLER_PATH_CHANNEL, &path);
            following_path = true;

            while (std::hypot(tag_pose_global.x-pose.x, tag_pose_global.y - pose.y) > 0.01 || abs(tag_pose_global.theta - pose.theta) > 0.5) {
                    std::cout << "keep fucking daddy\n";
            }

            printf("took a while daddy");

        }
       
    }
};


void form_control(int action, lcm::LCM* lcmInstance){
    mbot_lcm_msgs::pose2D_t servo_cmd;
    float time = 0, speed = 0;
    if (action == 0) action = 1;
    //std::cout << "Enter action, time and speed for the servos: ";
    //std::cin >> action >> time >> speed;
    servo_cmd.x = action;
    servo_cmd.y = speed;
    servo_cmd.theta = time;
    lcmInstance->publish(MBOT_SERVO_CHANNEL, &servo_cmd);
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    auto bitch = Bitches(&lcmInstance);
    

    while (1)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum   
    }
    return 0;
}