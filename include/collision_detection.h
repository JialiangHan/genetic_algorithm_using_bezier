/**
 * @file collision_detection.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief 
 * @version 0.1
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include "parameter_manager.h"
#include "piecewise_cubic_bezier.h"
#include <nav_msgs/OccupancyGrid.h>
#include "glog/logging.h"
#include "gflags/gflags.h"

namespace GeneticAlgorithm
{
    class CollisionDetection
    {
    public:
        CollisionDetection(){};
        CollisionDetection(const ParameterCollisionDetection &param)
        {
            params_ = param;
        };

        CollisionDetection(const nav_msgs::OccupancyGrid::ConstPtr &map)
        {
            SetMap(map);
        };

        void SetMap(const nav_msgs::OccupancyGrid::ConstPtr &map) { grid_ = map; };
        /**
         * @brief determine if point 2d is collision with obstacle or not
         * 
         * @param point_2d 
         * @return true collision
         * @return false not collsion
         */
        bool IsCollsion(const Eigen::Vector2d &point_2d);

        bool IsCollsion(const Eigen::Vector3d &point_3d);

        bool IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec);

        bool IsCollsion(CubicBezier::CubicBezier &cubic_bezier);

        /**
         * @brief find the index of cubic bezier in piecewise cubic bezier is first in collision
         * 
         * @param piecewise_cubic_bezier 
         * @return int should be in [0,size of cubic bezier list], -1 means no collision
         */
        int FindCollsionIndex(PiecewiseCubicBezier &piecewise_cubic_bezier);
        /**
         * @brief find out how many times a piecewise cubic bezier curve will encouter obstacle in the map.
         * 
         * @param piecewise_cubic_bezier 
         * @return int 
         */
        int GetTimesInCollision(PiecewiseCubicBezier &piecewise_cubic_bezier);

    private:
        ParameterCollisionDetection params_;
        /// A pointer to the grid_ the planner runs on
        nav_msgs::OccupancyGrid::ConstPtr grid_;
    };
}