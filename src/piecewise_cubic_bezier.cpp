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

    void PiecewiseCubicBezier::GetAnchorPointDirection()
    {
        // anchor_points2d_vec_.clear();
        anchor_points_dir_vec_.clear();
        if (anchor_points3d_vec_.size() == 0)
        {
            // DLOG(WARNING) << "anchor points are zero!!!";
        }
        else
        {
            for (auto point3d : anchor_points3d_vec_)
            {
                // anchor_points2d_vec_.emplace_back(Utility::ConvertVector3dToVector2d(point3d));
                float angle = point3d.z();
                Eigen::Vector3d direction;
                direction.x() = std::cos(angle);
                direction.y() = std::sin(angle);
                anchor_points_dir_vec_.emplace_back(direction);
            }
        }
    }

    void PiecewiseCubicBezier::CalculateControlPoints()
    {
        control_points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector3d direction_start;
        direction_start.x() = std::cos(start_angle);
        direction_start.y() = std::sin(start_angle);
        Eigen::Vector3d direction_goal;
        direction_goal.x() = std::cos(goal_angle);
        direction_goal.y() = std::sin(goal_angle);
        Eigen::Vector3d first_control_point, last_control_point;
        uint anchor_points_vec_size = anchor_points3d_vec_.size();

        // DLOG(INFO) << "free anchor points size is " << anchor_points_vec_size << "; " << anchor_points_vec_size + 1 << " bezier!";
        float t_start, t_goal;
        if (anchor_points_vec_size == 0)
        {
            //use the way in paper
            t_start = Utility::ConvertVector3dToVector2d((goal_point_ - start_point_)).norm() / 3;
            first_control_point = start_point_ + direction_start * t_start;
            t_goal = Utility::ConvertVector3dToVector2d((goal_point_ - start_point_)).norm() / 3;
            last_control_point = goal_point_ - direction_goal * t_goal;

            control_points_vec_.emplace_back(first_control_point);
            control_points_vec_.emplace_back(last_control_point);
            // DLOG(INFO) << "start is " << start_point_.x() << " " << start_point_.y() << " angle " << Utility::ConvertRadToDeg(start_point_.z()) << " first control point is " << first_control_point.x() << " " << first_control_point.y() << " t_start is " << t_start << " direction start is " << direction_start.x() << " " << direction_start.y() << " " << direction_start.z();
        }
        else if (anchor_points_vec_size == 1)
        {
            Eigen::Vector3d anchor_point_3d = anchor_points3d_vec_.front();

            t_start = Utility::ConvertVector3dToVector2d((anchor_point_3d - start_point_)).norm() / 3;

            first_control_point = start_point_ + direction_start * t_start;
            t_goal = Utility::ConvertVector3dToVector2d((anchor_point_3d - goal_point_)).norm() / 3;
            control_points_vec_.emplace_back(first_control_point);

            last_control_point = goal_point_ - direction_goal * t_goal;
            Eigen::Vector3d c2 = (first_control_point - last_control_point + 4 * anchor_point_3d) / 4;

            control_points_vec_.emplace_back(c2);

            Eigen::Vector3d c3 = 2 * anchor_point_3d - c2;
            control_points_vec_.emplace_back(c3);

            control_points_vec_.emplace_back(last_control_point);
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            // DLOG(INFO) << "second control point is " << c2.x() << " " << c2.y();
            // DLOG(INFO) << "anchor  point is " << anchor_point_3d.x() << " " << anchor_point_3d.y();
            // DLOG(INFO) << "c3 control point is " << c3.x() << " " << c3.y();
            // DLOG(INFO) << "c4 control point is " << last_control_point.x() << " " << last_control_point.y();
        }

        // DLOG(INFO) << "goal is " << goal_2d.x() << " " << goal_2d.y();
        // DLOG(INFO) << "points vec size is " << points_vec_.size();
        else
        {
            //basically is Ax=b, x is the distance factor, its pre and succ control point is control point pre= anchor-distance_factor*direction.
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * anchor_points_vec_size, 3 * anchor_points_vec_size);
            Eigen::VectorXd b(3 * anchor_points_vec_size);
            //calculate first and last control points
            Eigen::Vector3d first_anchor_point_3d = anchor_points3d_vec_.front();
            Eigen::Vector3d last_anchor_point_3d = anchor_points3d_vec_.back();
            t_start = Utility::ConvertVector3dToVector2d((first_anchor_point_3d - start_point_)).norm() / 3;
            first_control_point = start_point_ + direction_start * t_start;
            t_goal = Utility::ConvertVector3dToVector2d((last_anchor_point_3d - goal_point_)).norm() / 3;
            control_points_vec_.emplace_back(first_control_point);
            last_control_point = goal_point_ - direction_goal * t_goal;

            for (uint i = 0; i < anchor_points_vec_size; ++i)
            {
                //for first row
                if (i == 0)
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i + 1) = anchor_points_dir_vec_[i + 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = anchor_points3d_vec_[i + 1] - first_control_point;
                }
                // for last row
                else if (i == anchor_points_vec_size - 1)
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i - 1) = anchor_points_dir_vec_[i - 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = last_control_point - anchor_points3d_vec_[i - 1];
                }
                else
                {
                    //initialize A
                    A.block<3, 1>(3 * i, 3 * i + 1) = anchor_points_dir_vec_[i + 1];
                    A.block<3, 1>(3 * i, 3 * i - 1) = anchor_points_dir_vec_[i - 1];
                    //initialize b
                    b.block<3, 1>(3 * i, 0) = anchor_points3d_vec_[i + 1] - anchor_points3d_vec_[i - 1];
                }
                A.block<3, 1>(3 * i, 3 * i) = 4 * anchor_points_dir_vec_[i];
            }
            Eigen::VectorXd distance_factor(3 * anchor_points_vec_size);
            distance_factor = A.colPivHouseholderQr().solve(b);

            for (uint i = 0; i < anchor_points_vec_size; ++i)
            {
                Eigen::Vector3d current_dir = anchor_points_dir_vec_[i];

                Eigen::Vector3d pre_control_point = anchor_points3d_vec_[i] - current_dir * distance_factor[3 * i];
                Eigen::Vector3d succ_control_point = anchor_points3d_vec_[i] + current_dir * distance_factor[3 * i];
                control_points_vec_.emplace_back(pre_control_point);
                control_points_vec_.emplace_back(succ_control_point);
            }
            control_points_vec_.emplace_back(last_control_point);
        }
    }

    void PiecewiseCubicBezier::CalculatePointsVec()
    {
        points_vec_.clear();

        points_vec_.emplace_back(start_point_);

        uint count = 0;
        uint anchor_point_index = 0;
        for (uint control_point_index = 0; control_point_index < control_points_vec_.size(); ++control_point_index)
        {
            points_vec_.emplace_back(control_points_vec_[control_point_index]);
            // DLOG(INFO) << "control point is " << control_points_vec_[control_point_index].x() << " " << control_points_vec_[control_point_index].y();
            count++;
            if (count % 3 == 2)
            {
                if (anchor_points3d_vec_.size() != 0 && anchor_point_index < anchor_points3d_vec_.size())
                {

                    Eigen::Vector3d anchor_point_3d = anchor_points3d_vec_[anchor_point_index];
                    points_vec_.emplace_back(anchor_point_3d);
                    // DLOG(INFO) << "anchor point 2d is " << anchor_point_3d.x() << " " << anchor_point_3d.y();
                    // DLOG(INFO) << "anchor point index is " << anchor_point_index;
                    anchor_point_index++;
                    count++;
                }
            }
        }

        points_vec_.emplace_back(goal_point_);
        // for (auto point : points_vec_)
        // {
        //     DLOG(INFO) << "points are " << point.x() << " " << point.y();
        // }
    }

    void PiecewiseCubicBezier::CalculateCubicBezier()
    {
        cubic_bezier_vec_.clear();
        std::vector<Eigen::Vector3d> points_lists;
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

    Eigen::Vector3d PiecewiseCubicBezier::GetValueAt(const float &u)
    {
        Eigen::Vector3d out;

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
            // DLOG(INFO) << "u is " << u << " total number of bezier is " << total_number_of_bezier << " current index of bezier is " << current_index_of_bezier << " current factor is " << current_factor;
            out = cubic_bezier_vec_[current_index_of_bezier].GetValueAt(current_factor);
        }
        return out;
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

    std::vector<Eigen::Vector3d> PiecewiseCubicBezier::ConvertPiecewiseCubicBezierToVector3d()
    {
        std::vector<Eigen::Vector3d> out;
        for (auto bezier : cubic_bezier_vec_)
        {
            std::vector<Eigen::Vector3d> path;
            path = bezier.ConvertCubicBezierToVector3d();
            out.insert(out.end(), path.begin(), path.end());
        }
        return out;
    }
}