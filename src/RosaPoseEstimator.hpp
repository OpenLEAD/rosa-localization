#ifndef __ROSA_POSE_ESTIMATOR_HPP__
#define __ROSA_POSE_ESTIMATOR_HPP__

#include "RosaPoseParticle.hpp"
#include "Configuration.hpp"
#include "RosaOdometry.hpp"

#include <eslam/ParticleFilter.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/intrusive_ptr.hpp>

#include <base/samples/SonarBeam.hpp>
#include <sonaroctomap/SonarOcTree.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Pose.hpp>

#include<odometry/Sampling3D.hpp>

#include <limits>

namespace rosa_localization
{

class RosaPoseEstimator :
    public eslam::ParticleFilter<RosaPoseParticle>
{
public:
    RosaPoseEstimator(odometry::RosaOdometry& odometry, const octomap::SonarOcTree& map, const rosa_localization::Configuration& config);
    ~RosaPoseEstimator();

    void init(int numParticles, const base::Vector3d& mu, const base::Vector3d& sigma, double roll=0, double rollSigma = 0); 
    void project(const base::Orientation& orientation);
    void update(const base::samples::SonarBeam& sonarBeam, Eigen::Affine3d& ptuTransformation);
    base::Pose getCentroid();

private:
    void updateWeights(const base::samples::SonarBeam& sonarBeam, Eigen::Affine3d& ptuTransformation);

    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > rand_uni;
    base::Vector3d samplePose( const base::Vector3d& mu, const base::Vector3d& sigma );

    rosa_localization::Configuration config;
    odometry::RosaOdometry &odometry;
    
    octomap::SonarOcTree map;
   
    double max_weight;
};

}

#endif