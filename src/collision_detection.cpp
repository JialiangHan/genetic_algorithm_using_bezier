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
        for (uint i = 0; i < grid_->data.size(); ++i)
        {
            if (grid_->data[i])
            {
                continue;
            }
            else
            {
                int x = i % grid_->info.width;
                int y = i / grid_->info.height;
                Eigen::Vector3d free_point(x, y, 0);
                free_point_vec_.emplace_back(free_point);
            }
        }
    }

    bool CollisionDetection::IsCollsion(const Eigen::Vector2d &point_2d)
    {
        uint xi = (int)point_2d.x();
        uint yi = (int)point_2d.y();
        //ensure point is on the map;
        if (xi >= 0 &&
            xi < grid_->info.width &&
            yi >= 0 &&
            yi < grid_->info.width)
        {
            if (grid_->data[yi * grid_->info.width + xi])
            {
                // DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is in collision!";
                return true;
            }
            // DLOG(INFO) << "node is collision free.";
            return false;
        }
        else
        {
            // DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is outside map!";
            return true;
        }
    }

    bool CollisionDetection::IsCollsion(const Eigen::Vector3d &point_3d)
    {
        Eigen::Vector2d point_2d = Utility::ConvertVector3dToVector2d(point_3d);
        return IsCollsion(point_2d);
    }

    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec)
    {
        for (auto point2d : point_2d_vec)
        {
            if (IsCollsion(point2d))
            {
                // DLOG(INFO) << "vector is in collision.";
                return true;
            }
        }
        // DLOG(INFO) << "vector is collision free.";
        return false;
    }

    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector3d> &point_3d_vec)
    {
        for (auto point3d : point_3d_vec)
        {
            if (IsCollsion(point3d))
            {
                // DLOG(INFO) << "vector is in collision.";
                return true;
            }
        }
        // DLOG(INFO) << "vector is collision free.";
        return false;
    }

    bool CollisionDetection::IsCollsion(CubicBezier::CubicBezier &cubic_bezier)
    {
        std::vector<Eigen::Vector3d> anchor_points_vec = cubic_bezier.GetAnchorPoints();
        if (IsCollsion(anchor_points_vec))
        {
            // DLOG(INFO) << "anchor point is in collision.";
            return true;
        }
        for (int t = 0; t < 100; ++t)
        {
            if (IsCollsion(cubic_bezier.GetValueAt(t / 100.0)))
            {
                // DLOG(INFO) << "cubic bezier path is in collision.";
                return true;
            }
        }

        // DLOG(INFO) << "cubic bezier is collision free.";
        return false;
    }

    //TODO combine this function and the one below
    int CollisionDetection::FindCollsionIndex(const PiecewiseCubicBezier &piecewise_cubic_bezier)
    {
        std::vector<CubicBezier::CubicBezier> cubic_bezier_vec = piecewise_cubic_bezier.GetCubicBezierVector();
        for (uint i = 0; i < cubic_bezier_vec.size(); ++i)
        {
            if (IsCollsion(cubic_bezier_vec[i]))
            {
                // DLOG(INFO) << i << "th cubic bezier is in collision.";
                return i;
            }
        }
        return -1;
    }

    int CollisionDetection::GetTimesInCollision(PiecewiseCubicBezier &piecewise_cubic_bezier)
    {
        int times = 0;
        std::vector<CubicBezier::CubicBezier> cubic_bezier_vec = piecewise_cubic_bezier.GetCubicBezierVector();
        for (uint i = 0; i < cubic_bezier_vec.size(); ++i)
        {
            if (IsCollsion(cubic_bezier_vec[i]))
            {
                // DLOG(INFO) << i << "th cubic bezier is in collision.";
                times++;
            }
        }
        return times;
    }
};
