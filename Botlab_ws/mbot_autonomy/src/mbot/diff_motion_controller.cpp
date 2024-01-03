#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>

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
#include <slam/slam_channels.h>

#include "diff_maneuver_controller.h"

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers. 
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////
#define SEX_MACHINE 0.2

class StraightManeuverController : public ManeuverControllerBase
{

private:
    float fwd_pid[3] = {100.0, 0, 0};
    float fwd_sum_error = 0;
    float fwd_last_error = 0;
    float turn_pid[3] = {100, 0.0, 0.0};
    float turn_sum_error = 0;
    float turn_last_error = 0;
public:
    StraightManeuverController() = default; 

    float get_fwd_pidP()
    {
        return fwd_pid[0];
    }  
float get_fwd_pidI()
    {
        return fwd_pid[1];
    }
    float get_fwd_pidD()
    {
        return fwd_pid[2];
    }
    virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_fwd = sqrt(pow(dx,2) + pow(dy,2));
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);

        // PID separately for the fwd and the angular velocity output given the fwd and angular error
        fwd_sum_error += d_fwd;
        float fwd_der = 0;
        if (fwd_last_error > 0)
            fwd_der = (d_fwd - fwd_last_error) / 0.05;
        
        float fwd_vel = fwd_pid[0] * d_fwd + fwd_pid[1] * fwd_sum_error + fwd_pid[2] * fwd_der;
        // fprintf(stdout,"Fwd error: %f\tFwd vel: %f\n", d_fwd, fwd_vel);

        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = angle_diff(d_theta, turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\n", d_theta, turn_vel);

        return {0, fwd_vel, 0, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override
    {
        return ((fabs(pose.x - target.x) < 0.02) && (fabs(pose.y - target.y)  < 0.02));
    }
};

class TurnManeuverController : public ManeuverControllerBase
{
private:
    float turn_pid[3] = {100.0, 0, 0};
    float turn_sum_error = 0;
    float turn_last_error = 0;
public:
    TurnManeuverController() = default;   
    virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);
        // fprintf(stdout,"dx: %f\tdy: %f\td_theta: %f\n", dx, dy, d_theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0.0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\tPose theta: %f\n", d_theta, turn_vel, pose.theta);
        return {0, 0, 0, turn_vel};
    }
    mbot_lcm_msgs::twist2D_t get_command_final_turn(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target)
    {
        float d_theta = angle_diff(target.theta, pose.theta);
        // fprintf(stdout,"dx: %f\tdy: %f\td_theta: %f\n", dx, dy, d_theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\tPose theta: %f\n", d_theta, turn_vel, pose.theta);

        return {0, 0, 0, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(pose.theta, target_heading)) < 0.05);
    }
    bool target_reached_final_turn(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target)
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(target.theta, pose.theta)) < 0.05);
    }
};

class PurePursuit
{
    private:
    float lookahead_radius = 0.25;
    mbot_lcm_msgs::pose2D_t lk_pt;
    mbot_lcm_msgs::pose2D_t start_;
    mbot_lcm_msgs::pose2D_t end_;
    std::vector<mbot_lcm_msgs::pose2D_t> path_;

    double k_w = 0.1;
    double k_x = 10.0;

    float d_end_crit = 0.02;
    float d_end_midsteps = 0.29;
    float angle_end_crit = 0.2;

    public:
    PurePursuit()
    {
        lk_pt.x = 0;
        lk_pt.y = 0;
        lk_pt.theta = 0;
    }

    void set_path(std::vector<mbot_lcm_msgs::pose2D_t> path)
    {
        path_ = path;
        path_.push_back(start_);
        std::reverse(path_.begin(), path_.end());
        
    }
    void set_start(mbot_lcm_msgs::pose2D_t pt) { start_ = pt; }
    void set_end(mbot_lcm_msgs::pose2D_t pt) { end_ = pt; }
    mbot_lcm_msgs::pose2D_t get_end() {return end_;}
    bool update_lk(mbot_lcm_msgs::pose2D_t robot)
    {
        bool ret;
        for (int i = 0; i < path_.size()-1; ++i)
        {
            ret = get_lk(path_.at(i), path_.at(i+1), robot);
            if (ret) break;
        }
        return ret;
        
    }
    bool get_lk(mbot_lcm_msgs::pose2D_t start, mbot_lcm_msgs::pose2D_t end, mbot_lcm_msgs::pose2D_t robot)
    {
        double diff_x, diff_y;
        double rdiff_x, rdiff_y;
        diff_x = end.x - start.x;
        diff_y = end.y - start.y;
        rdiff_x = start.x - robot.x;
        rdiff_y = start.y - robot.y;

        //ax^2 + bx + c = 0 quadratic form
        double a = diff_x * diff_x + diff_y * diff_y;
        double b = 2 * (rdiff_x*diff_x + rdiff_y*diff_y);
        double c = rdiff_x*rdiff_x + rdiff_y*rdiff_y - (lookahead_radius*lookahead_radius);
        double discriminant = b*b - 4 * a * c;


        mbot_lcm_msgs::pose2D_t tmp;
        tmp.x = start.x;
        tmp.y = start.y;
        tmp.theta = 0;
        if (discriminant >= 0)
        {
            discriminant = std::sqrt(discriminant);
            double t1 = (-b-discriminant)/(2*a);
            double t2 = (-b+discriminant)/(2*a);
            if (t1 >= 0 && t1 <= 1)
            {
                tmp.x += t1*diff_x;
                tmp.y += t1*diff_y;
            }
            if (t2 >= 0 && t2 <= 1)
            {
                tmp.x += t2*diff_x;
                tmp.y += t2*diff_y;
            }
            lk_pt = tmp;
        }

        return discriminant >= 0;
    }

    int find_side(mbot_lcm_msgs::pose2D_t robot)
    {
        mbot_lcm_msgs::pose2D_t dir;
        dir.x = robot.x + std::cos(robot.theta);
        dir.y = robot.y + std::sin(robot.theta);

        //cross product
        double tmp = (dir.y - robot.y) * (lk_pt.x - robot.x) - (dir.x - robot.x) * (lk_pt.y - robot.y);

        return (tmp > 0)? 1 : -1;
    }
    mbot_lcm_msgs::twist2D_t calculate_twist(mbot_lcm_msgs::pose2D_t robot, bool is_last_target)
    {
        mbot_lcm_msgs::pose2D_t delta;
        delta.x = lk_pt.x - robot.x;
        delta.y = lk_pt.y - robot.y;

        double a = -std::tan(robot.theta);
        double b = 1;
        double c = std::tan(robot.theta) * robot.x - robot.y;
        double d = std::abs(a * lk_pt.x + b * lk_pt.y + c);
        d = d / std::sqrt(a*a+b*b);

        double curvature = 2*d / std::pow(delta.x * delta.x + delta.y * delta.y,2);
        curvature *= find_side(robot);

        float wz = -1*curvature*k_w;
        float vx;
        if (is_last_target) vx = (delta.x * delta.x + delta.y * delta.y)*0.1;
        // else vx = 0.1;
        vx = (delta.x * delta.x + delta.y * delta.y)*k_x;
        if (is_last_target) vx = 0.4;
        vx = std::min((double)vx, 5.0/std::abs(curvature));

        

        printf("curvature, wz: (%f, %f)\n", curvature, wz);

        return {0, vx, 0, wz};
    }
    mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_last_target)
    {
        // lk_pt = target;

        bool ret = update_lk(pose);

        if (ret) printf("valid lk\n");
        else printf("invalid lk\n");

        if (is_last_target) lk_pt = target;

        // printf("lk_pt: (%f,%f)\n", lk_pt.x, lk_pt.y);
        // printf("start: (%f,%f)\n", start_.x, start_.y);
        // printf("end: (%f,%f)\n\n", end_.x, end_.y);

        return calculate_twist(pose, is_last_target);
    }
    bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)
    {
        float distance = d_end_midsteps;
        if (is_end_pose) distance = d_end_crit;

        return ((fabs(pose.x - target.x) < distance) && (fabs(pose.y - target.y)  < distance));
    }
};
class SmartManeuverController : public ManeuverControllerBase
{

private:
    //kb is nono
    float pid[3] = {4.0, 2.0, 0.0}; //kp, ka, kb
    float d_end_crit = 0.02;
    float d_end_midsteps = 0.08;
    float angle_end_crit = 0.2;
public:
    SmartManeuverController() = default;
    virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target) override
    {
        float vel_sign = 1;
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_fwd = sqrt(dx * dx + dy * dy);
        float alpha = angle_diff(atan2(dy,dx), pose.theta);
        // printf("alpha: %f\n", alpha);

        // // To avoid weird behaviour at alpha=pi/2, because it is a common case
        // float margin = 2 * M_PI / 180;
        // if (fabs(alpha) > M_PI_2 + margin)
        // {
        //     alpha = wrap_to_pi(alpha - M_PI);
        //     vel_sign = -1;
        // }
        float beta = wrap_to_pi(target.theta - (alpha + pose.theta));
        float fwd_vel = vel_sign *  pid[0] * d_fwd;
        float turn_vel = pid[1] * alpha + pid[2] * beta;

        // If alpha is more than 45 degrees, turn in place and then go
        if (fabs(alpha) > M_PI_4)
        {
            fwd_vel = 0;
        }

        // printf("%f,%f\n", fwd_vel, turn_vel);
        return {0, fwd_vel, 0, turn_vel};
    }

    virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override
    {
        float distance = d_end_midsteps;
        if (is_end_pose) distance = d_end_crit;

        return ((fabs(pose.x - target.x) < distance) && (fabs(pose.y - target.y)  < distance));
    }
};

class MotionController
{ 
public: 
    
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

	    time_offset = 0;
	    timesync_initialized_ = false;
        pure_pursuit = PurePursuit();
    } 
    
    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_lcm_msgs::twist2D_t updateCommand(void) 
    {
        mbot_lcm_msgs::twist2D_t cmd {now(), 0.0, 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()) 
        {
            mbot_lcm_msgs::pose2D_t target = targets_.back();
            bool is_last_target = targets_.size() == 1;
            mbot_lcm_msgs::pose2D_t pose = currentPose();

            if (is_last_target) state_ = SMART;

            if (state_ == PURE_PURSUIT)
            {
                if (pure_pursuit.target_reached(pose, target, is_last_target))
                {
                    if (is_last_target)
                        state_ = FINAL_TURN;
                    else if(!assignNextTarget())
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                }
                else cmd = pure_pursuit.get_command(pose, target, is_last_target);   
            }

            if (state_ == SMART) 
            {
                if (smart_controller.target_reached(pose, target, is_last_target))
                {
                    if (is_last_target)
                        state_ = FINAL_TURN;
                    else if(!assignNextTarget())
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                }
                else cmd = smart_controller.get_command(pose, target);
            }

            ///////  TODO: Add different states when adding maneuver controls /////// 
            if(state_ == INITIAL_TURN)
            { 
                if(turn_controller.target_reached(pose, target, is_last_target))
                {
		            state_ = DRIVE;
                } 
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                }
            }
            else if(state_ == DRIVE) 
            {
                if(straight_controller.target_reached(pose, target, is_last_target))
                {
                    state_ = FINAL_TURN;
                    // if(!assignNextTarget())
                    // {
                    //     // std::cout << "\rTarget Reached!\n";
                    //     printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    // }
                }
                else
                { 
                    cmd = straight_controller.get_command(pose, target);
                }
		    }
            else if(state_ == FINAL_TURN)
            { 
                if(turn_controller.target_reached_final_turn(pose, target))
                {
		            if(!assignNextTarget())
                    {
                        // std::cout << "\rTarget Reached!\n";
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                    }
                } 
                else
                {
                    cmd = turn_controller.get_command_final_turn(pose, target);
                }
            }
            // else
            // {
            //     std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            // }
		} 
        return cmd; 
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();
    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::path2D_t* path)
    {
        targets_ = path->path;

        double min_dist = 100000;
        double min_dist_idx = targets_.size()-1;
        auto robot_pose = currentPose();

        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

        
    	std::cout << "received new path at time: " << path->utime << "\n"; 
    	for(auto pose : targets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}
        std::cout << std::endl;
        pure_pursuit.set_start(targets_.back());
        pure_pursuit.set_end(targets_.back());
        assignNextTarget();

        //confirm that the path was received
        mbot_lcm_msgs::mbot_message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* odometry)
    {
        mbot_lcm_msgs::pose2D_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose)
    {
        computeOdometryOffset(*pose);
    }
    StraightManeuverController getStraight()
    {
        return straight_controller;
    }
    
private:
    
    enum State
    {
        INITIAL_TURN,
        DRIVE,
        FINAL_TURN, // to get to the pose heading
        SMART,
        PURE_PURSUIT
    };
    
    mbot_lcm_msgs::pose2D_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<mbot_lcm_msgs::pose2D_t> targets_;
    std::vector<mbot_lcm_msgs::twist2D_t> targetvs_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;
 
    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;
    SmartManeuverController smart_controller;
    PurePursuit pure_pursuit;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }
    
    bool assignNextTarget(void)
    {
        if(!targets_.empty()) 
        { 
            
            targets_.pop_back();
            if (!targets_.empty())
            {
                printf("popped\n");
                printf("back_pts: (%f,%f)\n", targets_.back().x, targets_.back().y);
                pure_pursuit.set_start(pure_pursuit.get_end());
                pure_pursuit.set_end(targets_.back());
                pure_pursuit.set_path(targets_);
            }
        }
        // state_ = PURE_PURSUIT;
        state_ = SMART;
        return !targets_.empty();
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

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    // signal(SIGINT, exit);
    StraightManeuverController straight = controller.getStraight();
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
    	if(controller.timesync_initialized()){

            mbot_lcm_msgs::twist2D_t cmd = controller.updateCommand();
            // Limit command values
            // Fwd vel
            
            if (cmd.vx > SEX_MACHINE) cmd.vx = SEX_MACHINE;
            else if (cmd.vx < -SEX_MACHINE) cmd.vx = -SEX_MACHINE;

            // Angular vel
            float max_ang_vel = M_PI * 2.0 / 3.0;
            if (cmd.wz > max_ang_vel) cmd.wz = max_ang_vel;
            else if (cmd.wz < -max_ang_vel) cmd.wz = -max_ang_vel;

            // printf("%f\t%f\n", cmd.vx, cmd.wz);
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
            // float action, time, speed;
            // std::cout << "Enter action, time and speed for the servos: ";

            // mbot_lcm_msgs::pose2D_t servo_cmd;
            // std::cin >> action >> time >> speed;
            // servo_cmd.x = action;
            // servo_cmd.y = speed;
            // servo_cmd.theta = time;
            // lcmInstance.publish(MBOT_SERVO_CHANNEL, &servo_cmd);

    	}
    }
    
    return 0;
}