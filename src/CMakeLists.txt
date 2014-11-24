set(HDRS
    RosaPoseEstimator.hpp
    RosaPoseParticle.hpp
    RosaOdometry.hpp
    DepthState.hpp
    Configuration.hpp
    )
    
set(SCRS
    RosaPoseEstimator.cpp
    RosaOdometry.cpp
    )    

rock_library(rosa_localization
    SOURCES ${SCRS}
    HEADERS ${HDRS}
        DEPS_PKGCONFIG base-types odometry eslam
        DEPS_CMAKE octomap)

rock_executable(rosa_localization_bin Main.cpp
    DEPS rosa_localization)

