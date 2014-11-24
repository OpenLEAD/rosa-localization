#ifndef __ROSA_POSEPARTICLE_HPP__
#define __ROSA_POSEPARTICLE_HPP__

#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>

#include <vector>

#include <envire/tools/GaussianMixture.hpp>

namespace rosa_localization
{
struct RosaPoseParticle
{
    RosaPoseParticle() {};
    RosaPoseParticle( const base::Vector3d& position, double roll = 0, double rollSigma= 0)
	: position(position), 
	roll(roll),
	rollSigma(rollSigma), 
	weight(0) {};

    base::Affine3d getPose()
    {
	base::Vector3d pos( position.x(), position.y(), position.z());
	base::Affine3d t = 
	    Eigen::Translation3d( pos ) 
	    * Eigen::AngleAxisd( roll,  Eigen::MatrixBase<base::Vector3d>::UnitX() );
	
	return t;
    }
    
    //position on the xy-plane
    base::Vector3d position;
    
    //angle around the x-axis - roll
    double roll;
    double rollSigma;
    
    double mprob;

    // particle weight
    double weight;
};

struct PoseDistribution
{
    // we need to force the GMM model to use the base types
    // here instead of the generic eigen types
    struct BaseAdapter
    {
	enum { Dimension = 2 };
	typedef double Scalar;
	typedef base::Vector2d Vector;
	typedef base::Matrix2d Matrix;
    };

    typedef envire::GaussianMixture<double, 2, BaseAdapter> GMM;
    // Force instanciation of some of the templated code. This is needed for
    // gccxml (and therefore orogen)
    //
    // It is harmless outside these contexts
    struct gccxml_workaround {
	GMM::Parameter field;
    };

    base::Time time;
    std::vector<RosaPoseParticle> particles;
    GMM gmm;

};
}

#endif
