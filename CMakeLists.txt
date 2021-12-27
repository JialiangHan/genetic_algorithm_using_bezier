cmake_minimum_required(VERSION 3.0.2)
project(genetic_algorithm_using_bezier)


find_package(catkin REQUIRED)

## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    steering_functions
    roslaunch
    )
roslaunch_add_file_check(launch)
find_package(PythonLibs REQUIRED)
FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(Gflags)
find_package(Python COMPONENTS Interpreter Development NumPy)
if (${Python_FOUND})
  include_directories(${Python_INCLUDE_DIRS})
  include_directories(${Python_NumPy_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
set(HEADERS

    ${CMAKE_CURRENT_SOURCE_DIR}/include/collision_detection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cubic_bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/genetic_algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/matplotlibcpp.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/parameter_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_evaluator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path_publisher.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/piecewise_cubic_bezier.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utility.h
    )
set(SOURCES
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision_detection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cubic_bezier.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_evaluator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_publisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility.cpp
    )

add_library(GAB ${SOURCES} ${HEADERS} )
## Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)


include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ) 


add_executable(tf_broadcaster src/tf_broadcaster.cpp   )
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES} glog::glog)
# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

add_executable(genetic_algorithm_using_bezier src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(genetic_algorithm_using_bezier
                        ${PYTHON_LIBRARIES}                    
                        ${catkin_LIBRARIES} 
                    ${OMPL_LIBRARIES} 
                    glog::glog 
                        ${GFLAGS_LIBRARY} 
                    ) 
add_executable(node_path_evaluator 
            src/node_path_evaluator.cpp 
            ${SOURCES}
            ${HEADERS}
            )
target_link_libraries(node_path_evaluator 
                        ${PYTHON_LIBRARIES}
                        ${catkin_LIBRARIES} 
                        ${OMPL_LIBRARIES}
                        glog::glog 
                        ${GFLAGS_LIBRARY} 
)
add_executable(node_test 
            test/test.cpp 
            ${SOURCES}
            ${HEADERS}
            )
target_link_libraries(node_test 
                        ${PYTHON_LIBRARIES}
                        ${catkin_LIBRARIES} 
                        ${OMPL_LIBRARIES}
                        glog::glog 
                        ${GFLAGS_LIBRARY} 
)

