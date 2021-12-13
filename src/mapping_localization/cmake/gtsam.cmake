find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})
list(APPEND ALL_TARGET_LIBRARIES gtsam)

# gtsam has dependency on Boost::Timer
find_package (Boost 1.55.0 REQUIRED COMPONENTS system timer)
list(APPEND ALL_TARGET_LIBRARIES ${Boost_LIBRARIES})


