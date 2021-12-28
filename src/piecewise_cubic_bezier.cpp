/**
 * @file piecewise_cubic_bezier.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this class generate a piecewise cubic bezier curve
 * @version 0.1
 * @date 2021-12-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "piecewise_cubic_bezier.h"

namespace GeneticAlgorithm
{
    std::vector<Eigen::Vector3d> PiecewiseCubicBezier::ConvertPiecewiseCubicBezierToVector3d()
    {
        std::vector<Eigen::Vector3d> out;
        int i;
        for (i = 0; i < 100; ++i)
        {
            Eigen::Vector3d point3d;
            Eigen::Vector2d point;
            // DLOG(INFO) << " i/100 = " << i / 100.0;
            point = GetValueAt(i / 100.0);
            point3d = Utility::ConvertVector2dToVector3d(point);
            point3d.z() = GetAngleAt(i / 100.0);
            out.emplace_back(point3d);
        }
        return out;
    }

    float PiecewiseCubicBezier::GetAngleAt(const float &u)
    {
        float angle;
        int total_number_of_bezier = cubic_bezier_vec_.size();
        if (total_number_of_bezier == 0)
        {
            DLOG(INFO) << "No bezier, Please calculate first";
            angle = -100000;
        }
        else
        {
            int current_index_of_bezier;
            current_index_of_bezier = (int)std::floor(u * total_number_of_bezier);
            float current_factor = total_number_of_bezier * u - current_index_of_bezier;
            angle = cubic_bezier_vec_[current_index_of_bezier].GetAngleAt(current_factor);
        }
        return angle;
    }
    Eigen::Vector2d PiecewiseCubicBezier::GetValueAt(const float &u)
    {
        Eigen::Vector2d out;

        int total_number_of_bezier = cubic_bezier_vec_.size();
        if (total_number_of_bezier == 0)
        {
            DLOG(INFO) << "No bezier, Please calculate first";
            out.x() = -10000;
            out.y() = -10000;
        }
        else
        {
            int current_index_of_bezier;

            current_index_of_bezier = (int)std::floor(u * total_number_of_bezier);
            float current_factor = total_number_of_bezier * u - current_index_of_bezier;
            DLOG(INFO) << "u is " << u << " total number of bezier is " << total_number_of_bezier << " current index of bezier is " << current_index_of_bezier << " current factor is " << current_factor;
            out = cubic_bezier_vec_[current_index_of_bezier].GetValueAt(current_factor);
        }
        return out;
    }
    void PiecewiseCubicBezier::CalculateCubicBezier()
    {
        cubic_bezier_vec_.clear();
        std::vector<Eigen::Vector2d> points_lists;
        for (uint i = 0; i < points_vec_.size(); ++i)
        {
            points_lists.emplace_back(points_vec_[i]);
            // DLOG(INFO) << "points are " << points_vec_[i].x() << " " << points_vec_[i].y();
            if (points_lists.size() == 4)
            {
                cubic_bezier_vec_.emplace_back(CubicBezier::CubicBezier(points_lists));
                i = i - 1;
                points_lists.clear();
            }
        }
    }
    void PiecewiseCubicBezier::CalculateControlPoints()
    {
        points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector2d direction_start;
        direction_start.x() = std::cos(start_angle);
        direction_start.y() = std::sin(start_angle);
        Eigen::Vector2d direction_goal;
        direction_goal.x() = std::cos(goal_angle);
        direction_goal.y() = std::sin(goal_angle);
        Eigen::Vector2d start_2d = Utility::ConvertVector3dToVector2d(start_point_);
        Eigen::Vector2d goal_2d = Utility::ConvertVector3dToVector2d(goal_point_);
        Eigen::Vector2d first_control_point;
        Eigen::Vector2d last_control_point;
        points_vec_.emplace_back(start_2d);
        DLOG(INFO) << "start is " << start_2d.x() << " " << start_2d.y();
        float t_start, t_goal;
        if (anchor_points_vec_.size() == 0)
        {
            DLOG(INFO) << "free anchor points size is zero, only one bezier!";

            //use the way in paper
            t_start = (goal_2d - start_2d).norm() / 3;
            first_control_point = start_2d + direction_start * t_start;
            t_goal = (goal_2d - start_2d).norm() / 3;
            last_control_point = goal_2d - direction_goal * t_goal;

            points_vec_.emplace_back(first_control_point);
            points_vec_.emplace_back(last_control_point);
        }
        else if (anchor_points_vec_.size() == 1)
        {
            // DLOG(INFO) << "free anchor points size is 1, two bezier!";
            t_start = (anchor_points_vec_.front() - start_2d).norm() / 3;

            first_control_point = start_2d + direction_start * t_start;
            t_goal = (anchor_points_vec_.back() - goal_2d).norm() / 3;
            points_vec_.emplace_back(first_control_point);
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            last_control_point = goal_2d - direction_goal * t_goal;
            Eigen::Vector2d c2 = (first_control_point - last_control_point + 4 * anchor_points_vec_.front()) / 4;
            // DLOG(INFO) << "second control point is " << c2.x() << " " << c2.y();
            points_vec_.emplace_back(c2);
            points_vec_.emplace_back(anchor_points_vec_.front());
            // DLOG(INFO) << "anchor  point is " << anchor_points_vec_.front().x() << " " << anchor_points_vec_.front().y();
            Eigen::Vector2d c3 = 2 * anchor_points_vec_.front() - c2;
            points_vec_.emplace_back(c3);
            // DLOG(INFO) << "c3 control point is " << c3.x() << " " << c3.y();
            points_vec_.emplace_back(last_control_point);
            // DLOG(INFO) << "c4 control point is " << last_control_point.x() << " " << last_control_point.y();
        }
        points_vec_.emplace_back(goal_2d);
        // DLOG(INFO) << "goal is " << goal_2d.x() << " " << goal_2d.y();
        // DLOG(INFO) << "points vec size is " << points_vec_.size();
        //TODO need handle anchor points larger than 1
    }

    void PiecewiseCubicBezier::CalculateLength()
    {
        if (cubic_bezier_vec_.size() == 0)
        {
            length_ = 0;
        }
        else
        {
            for (auto cubic_bezier : cubic_bezier_vec_)
            {
                length_ += cubic_bezier.GetLength();
            }
        }
    }

}