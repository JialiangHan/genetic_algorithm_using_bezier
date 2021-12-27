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
        }

        void SetMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

        bool IsCollsion(const Eigen::Vector2d &point_2d);

        bool IsCollsion(const Eigen::Vector3d point_3d);
        //TODO need to make clear, these points are real points of curve or just control points?
        bool IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec);
        //TODO need to make clear, these points are real points of curve or just control points?
        bool IsCollsion(const std::vector<Eigen::Vector3d> &point_3d_vec);

    private:
        ParameterCollisionDetection params_;
        /// A pointer to the grid_ the planner runs on
        nav_msgs::OccupancyGrid::ConstPtr grid_;
    };
}