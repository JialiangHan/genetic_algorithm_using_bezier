/**
 * @file node_path_evaluator.cpp
 * @author your name (you@domain.com)
 * @brief just plot path evaluator
 * @version 0.1
 * @date 2021-12-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <cstring>

#include <ros/ros.h>

#include "path_evaluator.h"

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    google::ParseCommandLineFlags(&argc, &argv, true);

    google::InstallFailureSignalHandler();

    ros::init(argc, argv, "path_evaluator");
    std::string path_topic = "/path";
    // std::string smooth_path_topic = "/sPath";
    PathEvaluator::PathEvaluator path_evaluator(path_topic);
    // PathEvaluator::PathEvaluator path_evaluator(path_topic, smooth_path_topic);

    while (ros::ok())
    {
        path_evaluator.Plot();
        ros::spinOnce();
    }

    return 0;
}