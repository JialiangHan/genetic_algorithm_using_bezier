/**
 * @file main.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief  GENETIC ALGORITHM USING Piecewise cubic bezier curve
 * @version 0.1
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include "planner.h"

int main(int argc, char **argv)
{

   google::InitGoogleLogging(argv[0]);

   google::ParseCommandLineFlags(&argc, &argv, true);

   google::InstallFailureSignalHandler();
   google::EnableLogCleaner(3);
   ros::init(argc, argv, "genetic_algorithm_using_bezier");

   GeneticAlgorithm::Planner planner;
   planner.MakePlan();

   ros::spin();
   return 0;
}
