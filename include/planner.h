/**
 * @file planner.h
 * @author Jialiang han
 * @brief use GA to plan a path
 * @version 0.1
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "collision_detection.h"
#include "genetic_algorithm.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "utility.h"
#include "path_publisher.h"
#include "cubic_bezier.h"
#include "parameter_manager.h"
namespace GeneticAlgorithm
{
   /*!
   \brief A class that creates the interface for the genetic algorithm.
*/
   class Planner
   {
   public:
      /// The default constructor
      Planner();

      /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
      void SetMap(const nav_msgs::OccupancyGrid::Ptr &map);

      /*!
     \brief SetStart
     \param start the start pose
  */
      void SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &start);

      /*!
     \brief SetGoal
     \param goal the goal pose
  */
      void SetGoal(const geometry_msgs::PoseStamped::ConstPtr &goal);

      /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
      void MakePlan();

      void SetPlannerParams(const ParameterPlanner &params);

   private:
      /// The node handle
      ros::NodeHandle nh_;
      /// A publisher publishing the start position for RViz
      ros::Publisher pub_start_;
      /// A subscriber for receiving map updates
      ros::Subscriber sub_map_;
      /// A subscriber for receiving goal updates
      ros::Subscriber sub_goal_;
      /// A subscriber for receiving start updates
      ros::Subscriber sub_start_;
      /// A listener that awaits transforms
      tf::TransformListener listener_;
      /// A transform for moving start positions
      tf::StampedTransform transform_;
      /// A pointer to the grid_ the planner runs on
      nav_msgs::OccupancyGrid::Ptr grid_;
      /// The start pose set through RViz
      geometry_msgs::PoseWithCovarianceStamped start_;
      /// The goal pose set through RViz
      geometry_msgs::PoseStamped goal_;
      /// Flags for allowing the planner to plan
      bool valid_start_ = false;
      /// Flags for allowing the planner to plan
      bool valid_goal_ = false;
      //parameter manager, load param from *.yaml file
      std::shared_ptr<ParameterManager> param_manager_;

      ParameterPlanner params_;

      std::shared_ptr<GeneticAlgorithm> algorithm_ptr_;

      std::shared_ptr<CollisionDetection> collision_detection_ptr_;

      std::shared_ptr<PathPublisher> path_publisher_ptr_;

      // CubicBezier::CubicBezier cubic_bezier_;

      PiecewiseCubicBezier piecewise_cubic_bezier_;

      std::shared_ptr<GeneticAlgorithm> genetic_algorithm_ptr_;
   };
}
