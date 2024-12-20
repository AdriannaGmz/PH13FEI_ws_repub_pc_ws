cmake_minimum_required(VERSION 2.8.3)
project(repub_pc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  genmsg                #  for cpp msgs and srv
)

# Add C++11 norm and features (like unique_ptr)
# add_definitions( -std=c++0x )

###################################
## catkin specific configuration ##
###################################
catkin_package(
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(repPC_node src/repub_PC.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(repPC_node ${catkin_LIBRARIES})

