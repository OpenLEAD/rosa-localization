#include "RosaOdometry.hpp"

const double BAR2METER = 10.06;

using namespace odometry;

RosaOdometry::RosaOdometry(const Configuration& config)
: config(config),
  sampling(config)
{
}

RosaOdometry::~RosaOdometry()
{
}

Eigen::Matrix3d RosaOdometry::getPositionError()
{
    return Eigen::Matrix3d(sampling.poseCov.bottomRightCorner<3,3>());
}

Eigen::Matrix3d RosaOdometry::getOrientationError()
{
    return Eigen::Matrix3d(sampling.poseCov.topLeftCorner<3,3>());
}

Matrix6d RosaOdometry::getPoseError()
{
    return sampling.poseCov;
}

base::Pose RosaOdometry::getPoseDeltaSample()
{
    return base::Pose( sampling.sample() );
}

base::Pose2D RosaOdometry::getPoseDeltaSample2D()
{
    return projectPoseDelta( orientation, getPoseDeltaSample() );
}

base::Pose RosaOdometry::getPoseDelta()
{
    return base::Pose( sampling.poseMean );
}

void RosaOdometry::update(const base::samples::RigidBodyState& depth, const Eigen::Quaterniond& orientation)
{
    // update state
    state.update( depth );
    this->orientation = orientation;

    if( !state.isValid() )
    {
	state.update( depth );
	prevOrientation = orientation;
    }

    // get relative rotation between updates
    // this is assumed to be the correct rotation (with error of course)
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );
    
    //the delta in the xy-plane is considered zero due the mechanics of the system
    base::Pose deltaPose = base::Pose(state.getPrevious().position - state.getCurrent().position, delta_rotq);

    // TODO calculate error matrix
    
    double tilt = acos( Eigen::MatrixBase<base::Vector3d>::UnitZ().dot(orientation*Eigen::MatrixBase<base::Vector3d>::UnitZ() ) )  ;
    double d = deltaPose.position.norm();
;
    double dtheta = Eigen::AngleAxisd( delta_rotq ).angle();

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	d * config.distError.toVector4d() +
	tilt * config.tiltError.toVector4d() +
	dtheta * config.dthetaError.toVector4d();
    
    //TODO check useZeroVelocity
    double zeroCheck = d;	
    const double zeroVelocityThreshold = 1e-9;
    if( config.useZeroVelocity && zeroCheck < zeroVelocityThreshold )
    {
	deltaPose = base::Pose();
	vec.setZero();
    }

    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( deltaPose.toVector6d(), var.asDiagonal() );

    prevOrientation = orientation;
}