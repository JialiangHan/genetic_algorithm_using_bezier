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
                DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is in collision!";
                return true;
            }
        }
        else
        {
            DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is outside map!";
        }
        DLOG(INFO) << "node is collision free.";
        return false;
    }
    bool CollisionDetection::IsCollsion(const Eigen::Vector3d &point_3d)
    {
        Eigen::Vector2d point_2d = Utility::ConvertVector3dToVector2d(point_3d);
        if (IsCollsion(point_2d))
        {
            DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is in collision!";
            return true;
        }
        else
        {
            DLOG(INFO) << "point: " << point_2d.x() << " " << point_2d.y() << " is collision free!";
            return false;
        }
    }
    bool CollisionDetection::IsCollsion(const std::vector<Eigen::Vector2d> &point_2d_vec)
    {
        for (auto point2d : point_2d_vec)
        {
            if (IsCollsion(point2d))
            {
                DLOG(INFO) << "vector is in collision.";
                return true;
            }
        }
        DLOG(INFO) << "vector is collision free.";
        return false;
    }

    bool CollisionDetection::IsCollsion(CubicBezier::CubicBezier &cubic_bezier)
    {
        std::vector<Eigen::Vector2d> anchor_points_vec = cubic_bezier.GetAnchorPoints();
        if (IsCollsion(anchor_points_vec))
        {
            DLOG(INFO) << "anchor point is in collision.";
            return true;
        }
        if (IsCollsion(cubic_bezier.ConvertCubicBezierToVector2d()))
        {
            DLOG(INFO) << "cubic bezier path is in collision.";
            return true;
        }
        DLOG(INFO) << "cubic bezier is collision free.";
        return false;
    }
    //TODO combine this function and the one below
    int CollisionDetection::FindCollsionIndex(PiecewiseCubicBezier &piecewise_cubic_bezier)
    {
        std::vector<CubicBezier::CubicBezier> cubic_bezier_vec = piecewise_cubic_bezier.GetCubicBezierVector();
        for (uint i = 0; i < cubic_bezier_vec.size(); ++i)
        {
            if (IsCollsion(cubic_bezier_vec[i]))
            {
                DLOG(INFO) << i << "th cubic bezier is in collision.";
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
                DLOG(INFO) << i << "th cubic bezier is in collision.";
                times++;
            }
        }
        return times;
    }
};
