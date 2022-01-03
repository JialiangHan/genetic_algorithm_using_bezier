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
    void CollisionDetection::GenerateFreePointVec()
    {
        DLOG(INFO) << "GenerateFreePointVec in:";
        free_point_vec_.clear();
        for (uint i = 0; i < grid_->data.size(); ++i)
        {
            // DLOG(INFO) << i << "th map data is " << (int)grid_->data[i];
            if (grid_->data[i])
            {
                // DLOG(INFO) << "obstacle point is " << i % grid_->info.width << " " << i / grid_->info.width;
                continue;
            }
            else
            {
                int x = i % grid_->info.width;
                int y = i / grid_->info.width;
                Eigen::Vector3d free_point(x, y, 0);
                free_point_vec_.emplace_back(free_point);
                // DLOG(INFO) << "free point is " << x << " " << y;
            }
        }
        DLOG(INFO) << "GenerateFreePointVec out.";
    }

    bool CollisionDetection::IsCollsion(const Eigen::Vector2d &point_2d)
    {
        DLOG(INFO) << "IsCollision Eigen::Vector2d in:";
        //ensure point is on the map;
        if (point_2d.x() >= 0 &&
            point_2d.x() < grid_->info.width &&
            point_2d.y() >= 0 &&
            point_2d.y() < grid_->info.height)
        {
            uint xi = (int)point_2d.x();
            uint yi = (int)point_2d.y();
            if (grid_->data[yi * grid_->info.width + xi])
            {
                DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is in collision!";
                DLOG(INFO) << "IsCollision Eigen::Vector2d out.";
                return true;
            }
            DLOG(INFO) << "node is collision free.";
            DLOG(INFO) << "IsCollision Eigen::Vector2d out.";
            return false;
        }
        else
        {
            DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is outside map!";
            DLOG(INFO) << "IsCollision Eigen::Vector2d out.";
            return true;
        }
    }

    bool CollisionDetection::IsCollsion(const Eigen::Vector3d &point_3d)
    {
        DLOG(INFO) << "IsCollision Eigen::Vector3d in:";
        Eigen::Vector2d point_2d = Utility::ConvertVector3dToVector2d(point_3d);
        DLOG(INFO) << "IsCollision Eigen::Vector3d out.";
        return IsCollsion(point_2d);
    }

    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec)
    {
        DLOG(INFO) << "IsCollision vector<Eigen::Vector2d> in:";
        for (const auto &point2d : point_2d_vec)
        {
            if (IsCollsion(point2d))
            {
                DLOG(INFO) << "vector is in collision.";
                DLOG(INFO) << "IsCollision vector<Eigen::Vector2d> out.";
                return true;
            }
        }
        DLOG(INFO) << "vector is collision free.";
        DLOG(INFO) << "IsCollision vector<Eigen::Vector2d> out.";
        return false;
    }

    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector3d> &point_3d_vec)
    {
        DLOG(INFO) << "IsCollision vector<Eigen::Vector3d> in:";
        for (const auto &point3d : point_3d_vec)
        {
            if (IsCollsion(point3d))
            {
                DLOG(INFO) << "vector is in collision.";
                DLOG(INFO) << "IsCollision  vector<Eigen::Vector3d> out.";
                return true;
            }
        }
        DLOG(INFO) << "vector is collision free.";
        DLOG(INFO) << "IsCollision  vector<Eigen::Vector3d> out.";
        return false;
    }

    bool CollisionDetection::IsCollsion(CubicBezier::CubicBezier &cubic_bezier)
    {
        DLOG(INFO) << "IsCollision CubicBezier in:";
        std::vector<Eigen::Vector3d> anchor_points_vec = cubic_bezier.GetAnchorPoints();
        // DLOG(INFO) << "collision for anchor points " << IsCollsion(anchor_points_vec);
        if (IsCollsion(anchor_points_vec))
        {
            DLOG(INFO) << "anchor point is in collision.";
            DLOG(INFO) << "IsCollision CubicBezier out.";
            return true;
        }
        for (int t = 0; t < 100; ++t)
        {
            // DLOG(INFO) << "cubic bezier value at " << t / 100.0 << " is " << cubic_bezier.GetValueAt(t / 100.0) << " is in collision " << IsCollsion(cubic_bezier.GetValueAt(t / 100.0));

            if (IsCollsion(cubic_bezier.GetValueAt(t / 100.0)))
            {
                DLOG(INFO) << "cubic bezier path is in collision.";
                DLOG(INFO) << "IsCollision CubicBezier out.";
                return true;
            }
        }
        DLOG(INFO) << "IsCollision CubicBezier out.";
        // DLOG(INFO) << "cubic bezier is collision free.";
        return false;
    }

    //TODO combine this function and the one below
    int CollisionDetection::FindCollsionIndex(PiecewiseCubicBezier piecewise_cubic_bezier)
    {
        DLOG(INFO) << "FIndCollisionIndex in:";
        std::vector<CubicBezier::CubicBezier> cubic_bezier_vec = piecewise_cubic_bezier.GetCubicBezierVector();
        // DLOG(INFO) << "size of cubic bezier vec is " << cubic_bezier_vec.size();
        for (uint i = 0; i < cubic_bezier_vec.size(); ++i)
        {
            bool collision = IsCollsion(cubic_bezier_vec[i]);
            // DLOG(INFO) << "collision for " << i << "th cubic bezier is " << collision;
            if (collision)
            {
                // DLOG(INFO) << i << "th cubic bezier is in collision.";
                DLOG(INFO) << "FIndCollisionIndex out:";
                return i;
            }
        }
        DLOG(INFO) << "FIndCollisionIndex out.";
        return -1;
    }

    int CollisionDetection::GetTimesInCollision(PiecewiseCubicBezier &piecewise_cubic_bezier)
    {
        DLOG(INFO) << "GetTimesInCollision in:";
        int times = 0;
        std::vector<CubicBezier::CubicBezier> cubic_bezier_vec = piecewise_cubic_bezier.GetCubicBezierVector();
        for (uint i = 0; i < cubic_bezier_vec.size(); ++i)
        {
            if (IsCollsion(cubic_bezier_vec[i]))
            {
                DLOG(INFO) << i << "th cubic bezier is in collision.";
                times++;
            }
        }
        DLOG(INFO) << "GetTimesInCollision out.";
        return times;
    }
};
