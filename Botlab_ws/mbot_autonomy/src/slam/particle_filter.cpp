#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <utils/geometric/angle_functions.hpp>
#include <chrono>


// #define DEBUG_TIME

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleweight = 1.0/ kNumParticles_;
    posteriorPose_ = pose;

    for (auto& p: posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = p.pose;
        p.weight = sampleweight;

    }

}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    #ifdef DEBUG_TIME
    auto t1 = std::chrono::high_resolution_clock::now();
    #endif
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    // if (hasRobotMoved){
        ParticleList prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);

        // std::cout << "p.weight: " << p.weight << std::endl;
    // }
        // reinvigoratePriorDistribution(posterior_);

    posteriorPose_ = estimatePosteriorPose(posterior_);
    // }    
    // else {
    //     posteriorPose_ = odometry;
    // }

    posteriorPose_.utime = odometry.utime;

    #ifdef DEBUG_TIME
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    printf("update_time: %f ms.\n", ms_double.count());
    #endif
    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{

    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        ParticleList prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }


    double x = 0, y = 0, theta = 0;
    mbot_lcm_msgs::pose2D_t expected_val_posterior;
    expected_val_posterior.x = 0;
    expected_val_posterior.y = 0;
    expected_val_posterior.theta = 0;

    for (int i = 0; i < posterior_.size(); ++i)
    {
        
        expected_val_posterior.x += posterior_.at(i).pose.x;
        expected_val_posterior.y += posterior_.at(i).pose.y;
        expected_val_posterior.theta += posterior_.at(i).pose.theta;
    }
    expected_val_posterior.x /= posterior_.size();
    expected_val_posterior.y /= posterior_.size();
    expected_val_posterior.theta /= posterior_.size();

    posteriorPose_ = expected_val_posterior; //change this line
    posteriorPose_.utime = odometry.utime;
    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior = posterior_;
    double sampleWeight = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0,0.01);

    for(auto& p:prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
    }
    return prior;  // Placeholder

    // return lowVarianceSample(posterior_.size(),posterior_);

}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior = posterior_;
    double sampleWeight = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0,0.01);

    for(auto& p:prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
    }

    
    // return prior;
    
    return prior;  // Placeholder

    // return lowVarianceSample(posterior_.size(),posterior_);
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<mbot_lcm_msgs::particle_t> proposal;
    for(auto & p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    
    return proposal;  // Placeholder
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<mbot_lcm_msgs::particle_t> posterior;
    double sumweights = 0.0;
    for(auto& p: proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumweights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto& p: posterior){
        p.weight /= sumweights;
        // std::cout << "p.weight: " << p.weight << std::endl;
    }

    return posterior;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.

    mbot_lcm_msgs::pose2D_t pose;
    double xmean = 0.0;
    double ymean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    double thetaMean = 0.0;
    for(const auto& p: posterior){
        xmean += p.weight * p.pose.x;
        ymean += p.weight * p.pose.y;
        thetaMean += p.weight * p.pose.theta;
    }
    
    pose.x = xmean;
    pose.y = ymean;
    pose.theta = thetaMean;
    return pose;


}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
