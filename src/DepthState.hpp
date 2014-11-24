#ifndef __ODOMETRY_DEPTH_STATE_HPP__
#define __ODOMETRY_DEPTH_STATE_HPP__

#include <base/time.h>
namespace odometry{
struct DepthState
{
    /** @brief timestamp */
    base::Time time;
    
    /** @brief pressure aquired from the depth sensor, unit bar */
    double pressure;
        
};
}
#endif
