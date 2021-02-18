cmake_minimum_required(VERSION 2.8.3)
project(flight_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#        nav_core
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        geometry_msgs
        pcl_ros
        quadrotor_msgs
        tf2
        webots_ros
        )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES trajectory_planning
        CATKIN_DEPENDS nav_core roscpp std_msgs
        #  DEPENDS system_lib
)



###########
## Webots ##
###########

set(SL_PREFI "lib")
set(SL_SUFFIX ".so")
set(WEBOTS_HOME "/usr/local/webots")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/controllers/se3controller)

include_directories(
        ${WEBOTS_HOME}/include/controller/cpp
        ${WEBOTS_HOME}/include/controller/c
        ${catkin_INCLUDE_DIRS}
        include
)


###########
## Webots ##
###########

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${WEBOTS_HOME}/lib/controller/${SL_PREFI}Controller${SL_SUFFIX}
        ${WEBOTS_HOME}/lib/controller/${SL_PREFI}CppController${SL_SUFFIX}
        ${PCL_LIBRARIES}
        )

add_executable(se3controller
        App/main.cpp
        src/flightController.cpp
        src/SO3Control.cpp
        )
target_link_libraries(se3controller
        ${THIRD_PARTY_LIBS}
        ${catkin_LIBRARIES}
        /usr/lib/x86_64-linux-gnu/libm.so
        )


