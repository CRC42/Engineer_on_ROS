find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_library(wit_ros wit_ros.cpp)
target_link_libraries(wit_ros wit_driver ${catkin_LIBRARIES} )
