/**
 * @file collision_detection.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief 
 * @version 0.1
 * @date 2021-12-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "collision_detection.h"

namespace GeneticAlgorithm
{
    void CollisionDetection::SetMap(const nav_msgs::OccupancyGrid::ConstPtr &map) {}

    bool CollisionDetection::IsCollsion(const Eigen::Vector2d &point_2d)
    {
        return true;
    }

    bool CollisionDetection::IsCollsion(const Eigen::Vector3d point_3d)
    {
        return true;
    }
    //TODO need to make clear, these points are real points of curve or just control points?
    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec)
    {
        return true;
    }
    //TODO need to make clear, these points are real points of curve or just control points?
    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector3d> &point_3d_vec)
    {
        return true;
    }

};
