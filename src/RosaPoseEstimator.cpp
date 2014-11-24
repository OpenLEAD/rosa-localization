#include "RosaPoseEstimator.hpp"
#include <algorithm>
#include <stdexcept>
#include <set>
#include <omp.h>
#include <boost/bind.hpp>

using namespace rosa_localization;

RosaPoseEstimator::RosaPoseEstimator(odometry::RosaOdometry& odometry, 
			     const octomap::SonarOcTree& map, 
			     const rosa_localization::Configuration& config )
    : ParticleFilter<RosaPoseParticle>(config.seed), 
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    rand_uni(rand_gen, boost::uniform_real<>(0,1.0) ),
    config(config),
    odometry(odometry),
    map(map),
    max_weight(0)
{
  
}

RosaPoseEstimator::~RosaPoseEstimator()
{
}
base::Vector3d RosaPoseEstimator::samplePose( const base::Vector3d& mu, const base::Vector3d& sigma )
{
    double x = rand_norm(), y = rand_norm(), z = rand_norm();

    return base::Vector3d( 
		x * sigma.x() + mu.x(),
		y * sigma.y() + mu.y(),
	        z * sigma.z() + mu.z()) ;
}

//the the particle orientation is going to be considered as the roll angle
void RosaPoseEstimator::init(int numParticles, const base::Vector3d& mu, const base::Vector3d& sigma, double roll, double rollSigma) 
{
    for(int i=0;i<numParticles;i++)
    {
	base::Vector3d sample = samplePose( mu, sigma );

	xi_k.push_back( 
		Particle( 
		    sample,
		    roll,
	            rollSigma
		    ));
    }
}

/**
 * this is a piecewise linear function, with 
 * @param x as the function input
 * @param alpha minimum threshold any value of x below this value will be 1.0
 * @param beta any x above beta will be set to gamma
 * @param gamma result if x is above beta
 *
 * the interval between alpha and beta is linear, so that the function is
 * continous.
 */
double weightingFunction( double x, double alpha = 0.1, double beta = 0.9, double gamma = 0.05 )
{
    if( x < alpha )
	return 1.0;
    if( x < beta )
    {
	double a = (1.0-gamma)/(alpha-beta);
	double b = 1.0-alpha*a;
	return a*x + b; 
    }
    if( x >= beta )
	return gamma;

    return 0.0;
}


/**
 * Update every particle using the odometry data and the absolute value of the inclinometer, with error included
 */
void RosaPoseEstimator::project(const base::Orientation& orientation)
{
    double roll =  base::getRoll( orientation );
    
    /*the orientation will not have an influence on the delta translation, because of the "rail"
     * The orientation will only affect the final orientation, i.e the POV of the sonar */
    Eigen::Affine3d poseDelta = odometry.getPoseDeltaSample().toTransform();
    //const double z_var = 1e-3;
    //TODO why x2?
    double rollVar = odometry.getOrientationError()(1,1) * 2.0;;

    double spread = weightingFunction( max_weight, 0.0, config.spreadThreshold, 0.0 );

    for(size_t i=0;i<xi_k.size();i++)
    {
	Particle &p( xi_k[i] );
	p.position += poseDelta.translation();
	p.roll = roll;
	p.rollSigma = sqrt(p.rollSigma*p.rollSigma + rollVar);

	// the particle with the lowest weight
	// is below a threshold. depending on what sort
	// of mode we are in, do different things.
	if(spread > 0) 
	{
	    // otherwise just spread out the particles and see if we can 
	    // recover this way.
	    const double trans_facx = config.spreadTranslationXFactor * spread;
	    const double trans_facy = config.spreadTranslationYFactor * spread;
	    const double trans_facz = config.spreadTranslationZFactor * spread;
	    base::Vector3d sample = samplePose( 
		    base::Vector3d(), 
		    base::Vector3d(trans_facx, trans_facy ,trans_facz ) );

	    p.position += sample;
	    p.roll = roll;
	}
    }
}

void RosaPoseEstimator::update(const base::samples::SonarBeam& sonarBeam,  Eigen::Affine3d& ptuTransformation)
{
    updateWeights(sonarBeam,ptuTransformation);
    double eff = normalizeWeights();
    if( eff < config.minEffective )
    {
	resample();
    }
}

void RosaPoseEstimator::updateWeights(const base::samples::SonarBeam& sonar_beam, Eigen::Affine3d& ptuTransformation)
{
  
    double last_max_weight = max_weight;
    max_weight = 0;

    for(size_t i=0;i<xi_k.size();i++)
    {
	RosaPoseParticle &pose(xi_k[i]);
	Eigen::Affine3d particle_pose = pose.getPose();
	
	Eigen::Affine3d sonar_pose = particle_pose*ptuTransformation; 
	const double weight = map.evaluateSonarBeam(sonar_pose, sonar_beam);
        xi_k[i].weight *= weight;
	xi_k[i].mprob = weight;

	// store the current maximum weight
	max_weight = std::max( max_weight, weight );

    }

}

base::Pose RosaPoseEstimator::getCentroid()
{
    normalizeWeights();

    // calculate the weighted mean for now
    base::Vector3d mean;
    double rollMean = 0.0;
    double sumWeights = 0.0;
    for(size_t i=0;i<xi_k.size();i++)
    {
	const Particle &particle(xi_k[i]);
	
	
	mean += particle.position * particle.weight;
	rollMean +=  particle.roll*particle.weight;
	sumWeights += particle.weight;
	
    }
    mean /= sumWeights;
    rollMean /= sumWeights;

    // and convert into a 3d position
    base::Pose result( 
	    mean, (base::Quaterniond)Eigen::AngleAxisd( rollMean,  Eigen::MatrixBase<base::Vector3d>::UnitX())) ;

    return result;
}

