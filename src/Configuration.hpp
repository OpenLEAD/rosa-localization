#ifndef __ROSALOCALISATION_CONFIGURATION_HPP__
#define __ROSALOCALISATION_CONFIGURATION_HPP__

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <base/Eigen.hpp>

namespace rosa_localization 
{

struct UpdateThreshold
{
    UpdateThreshold() {};
    UpdateThreshold( double distance, double angle )
	: distance( distance ), angle( angle ) {};

    bool test( double distance, double angle )
    {
	return distance > this->distance || angle > this->angle;
    }

    bool test( const Eigen::Affine3d& pdelta )
    {
	return test( Eigen::AngleAxisd( pdelta.linear() ).angle(), pdelta.translation().norm() );
    }

    double distance;
    double angle;
};

struct Configuration
{
    Configuration() :
	seed( 42u ),
	particleCount( 250 ), 
	minEffective( 50 ), 
	initialRotationError( base::Vector3d(0, 0, 0.1) ),
	initialTranslationError( base::Vector3d(0.1, 0.1, 1.0) ),
	spreadThreshold( 0.9 ),
	spreadTranslationXFactor( 0.1 ),
        spreadTranslationYFactor( 0.1 ),
        spreadTranslationZFactor( 0.1 ),
	maxRollDeviation( 15*M_PI/180.0 ),
	measurementThreshold( 0.1, 10*M_PI/180.0 ),
	logDebug( false ),
	logParticlePeriod( 100 )
    {};

    /** seed for all random processes in the filter */
    unsigned long seed;
    /** number of particles to use in the filter */
    size_t particleCount;
    /** if the effective number of particles goes below this count, 
     * the particles are resampled
     */
    size_t minEffective;
    /** initial sampling spread of the particles. 
     * This vector is split into the rotational (3) and translational (3) parts
     * of the error, so that [r t] is a 6 vector. That vector is effectively the
     * diagonal of the full covariance matrix of the initial error distribution.
     */
    base::Vector3d initialRotationError;
    /** initial sampling spread of the particles. 
     * This vector is split into the rotational (3) and translational (3) parts
     * of the error, so that [r t] is a 6 vector. That vector is effectively the
     * diagonal of the full covariance matrix of the initial error distribution.
     */
    base::Vector3d initialTranslationError;
   
    /** threshold value for particle spreading. Particle spreading is
     * activated when the maximum particle weight is below this value.
     * Set to 0.0 to disable spreading.
     */ 
    double spreadThreshold;
    
    /** spread factor for translational component in the x axis
     */
    double spreadTranslationXFactor;
 
     /** spread factor for translational component in the y axis
     */
    double spreadTranslationYFactor;
    
     /** spread factor for translational component in the z axis
     */
    double spreadTranslationZFactor;
   
    /** when set to a positive value, the filter will try to keep the deviation
     * of the estimated yaw value compared to the yaw value from the odometry
     * to this value
     */
    double maxRollDeviation;
    /** distance and rotation threshold for measurement updates. This provides the
     * minimum distance/rotation after which a new measurement is considered.
     */
    UpdateThreshold measurementThreshold;
    //for now is not used
    
    /** configuration options for the contact model
     */
    /** if set to true, the filter will generate debug information in the particles.
     * this will result in very large log files.
     */
    bool logDebug;
    /** controls how often the particle distribution is logged
     * a value of 1 will log at every step (large log file)
     * a value of 0 will skip logging the particles
     * values greater than 1 will log every nth distribution.
     */
    unsigned int logParticlePeriod;
};

}
#endif
