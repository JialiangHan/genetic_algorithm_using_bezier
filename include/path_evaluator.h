/**
 * @file path_evaluator.h
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.1
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021
 * 
**/
#pragma once
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "genetic_algorithm_using_bezier/FitnessMsgVec.h"
#include "utility.h"
namespace PathEvaluator
{
    class PathEvaluator
    {
    public:
        PathEvaluator(){};
        PathEvaluator(const std::string &path_topic)
        {
            sub_map_ = nh_.subscribe("/map", 1, &PathEvaluator::CallbackSetMap, this);
            sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, path_topic));
            sub_fitness_ = nh_.subscribe<genetic_algorithm_using_bezier::FitnessMsgVec>("/fitness", 1, &PathEvaluator::CallbackFitness, this);
        };
        PathEvaluator(const std::string &path_topic, const std::string &smoothed_path_topic)
        {
            sub_map_ = nh_.subscribe("/map", 1, &PathEvaluator::CallbackSetMap, this);
            sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, path_topic));
            sub_smoothed_path_ = nh_.subscribe<nav_msgs::Path>(smoothed_path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, smoothed_path_topic));
        };

        /**
         * @brief plot all the metrics for the path.
         * 
         * @return int 
         */
        void Plot();

    private:
        void CallbackFitness(const genetic_algorithm_using_bezier::FitnessMsgVecConstPtr &fitness_vec);
        void CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name);
        /**
         * @brief as name suggest, this function convert a ROS message path to a vector of node3d
         * 
         * @param path 
         * @param node_3d_vec 
         */

        void CallbackSetMap(const nav_msgs::OccupancyGridConstPtr &map);

        /**
         * @brief calculate curvature for the path 
         * 
         * @param path got from planner
         * @return std::vector<double> 
         */
        int CalculateCurvature(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name);

        int CalculateClearance(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name);

        int CalculateSmoothness(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name);

        int CalculateSteeringAngle(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name);

        int CalculateMetricMap();

    private:
        ros::NodeHandle nh_;

        ros::Subscriber sub_path_;

        ros::Subscriber sub_smoothed_path_;

        ros::Subscriber sub_map_;

        ros::Subscriber sub_fitness_;

        nav_msgs::OccupancyGridConstPtr map_;
        /**
         * @brief key is topic name for all four maps; 
         * 
         */
        std::unordered_map<std::string, std::vector<double>> clearance_map_;

        std::unordered_map<std::string, std::vector<double>> curvature_map_;

        std::unordered_map<std::string, std::vector<double>> smoothness_map_;

        std::unordered_map<std::string, std::vector<double>> steering_angle_map_;

        std::unordered_map<std::string, std::vector<double>> fitness_map_;
        /**
         * @brief key is metric name, value is above map
         * 
         */
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<double>>> metric_map_;

        // /some kind of map is need for the clearacne
    };
}
