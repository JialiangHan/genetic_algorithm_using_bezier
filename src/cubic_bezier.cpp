/**
 * @file cubic_bezier.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief for cubic_bezier
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "cubic_bezier.h"
#include "utility.h"
#include <cstdlib>
#include <math.h>
namespace CubicBezier
{
    Eigen::Vector4d CubicBezier::CalculateCoefficient(const float &t)
    {
        Eigen::Vector4d out;

        for (int i = 0; i < out.size(); ++i)
        {
            out[i] = std::pow(t, i);
            DLOG(INFO) << i << "th element in coefficient matrix is " << out[i];
        }
        return out;
    }

    // void CubicBezier::CalculateCoefficient(const float &t)
    // {
    //     coefficient_.clear();
    //     float coefficient;
    //     coefficient = std::pow((1 - t), 3);
    //     coefficient_.emplace_back(coefficient);
    //     coefficient = 3 * t * std::pow(1 - t, 2);
    //     coefficient_.emplace_back(coefficient);
    //     coefficient = 3 * std::pow(t, 2) * (1 - t);
    //     coefficient_.emplace_back(coefficient);
    //     coefficient = std::pow(t, 3);
    //     coefficient_.emplace_back(coefficient);
    // }
    Eigen::Vector4d CubicBezier::CalculateFirstOrderDerivativeCoefficient(const float &t)
    {
        Eigen::Vector4d out;

        for (int i = 0; i < out.size(); ++i)
        {
            out[i] = i * (i - 1 < 0 ? 0 : std::pow(t, i - 1));
            DLOG(INFO) << i << "th element in first order derivation coefficient matrix is " << out[i];
        }
        return out;
    }
    // void CubicBezier::CalculateFirstOrderDerivativeCoefficient(const float &t)
    // {
    //     first_order_derivative_coefficient_.clear();
    //     float coefficient;
    //     coefficient = -3 * std::pow((1 - t), 2);
    //     first_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = 3 * std::pow(1 - t, 2) - 6 * t * (1 - t);
    //     first_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = 6 * t * (1 - t) - 3 * std::pow(t, 2);
    //     first_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = 3 * std::pow(t, 2);
    //     first_order_derivative_coefficient_.emplace_back(coefficient);
    // }

    // void CubicBezier::CalculateSecondOrderDerivativeCoefficient(const float &t)
    // {
    //     second_order_derivative_coefficient_.clear();
    //     float coefficient;
    //     coefficient = 6 * std::pow((1 - t), 1);
    //     second_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = -6 * std::pow(1 - t, 1) - 6 * (1 - t) + 6 * t;
    //     second_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = 6 * (1 - t) - 6 * t - 6 * std::pow(t, 1);
    //     second_order_derivative_coefficient_.emplace_back(coefficient);
    //     coefficient = 6 * std::pow(t, 1);
    //     second_order_derivative_coefficient_.emplace_back(coefficient);
    // }

    Eigen::Vector3d CubicBezier::GetFirstOrderDerivativeValueAt(const float &t)
    {
        Eigen::Vector3d out;

        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateFirstOrderDerivativeCoefficient(t);

        // CalculateFirstOrderDerivativeCoefficient(t);

        // out = first_order_derivative_coefficient_[0] * start_point_ + first_order_derivative_coefficient_[1] * control_points_vec_[0] + first_order_derivative_coefficient_[2] * control_points_vec_[1] + first_order_derivative_coefficient_[3] * goal_point_;
        return out;
    }

    void CubicBezier::CalculateControlPoints()
    {

        control_points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector3d first_control_point, second_control_point, direction;
        direction.x() = std::cos(start_angle);
        direction.y() = std::sin(start_angle);
        if (use_random_)
        {

            std::vector<Eigen::Vector2d> polygon;
            polygon = Utility::CreatePolygon(map_width_, map_height_);
            // DLOG(INFO) << " map width is " << map_width_ << " height is " << map_height_;
            //t is some random number;
            int t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
            first_control_point = start_point_ + direction * t;
            // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            int flag = Utility::IsInsidePolygon(polygon, first_control_point);
            while (flag < 1)
            {
                t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
                first_control_point = start_point_ + direction * t;
                // DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
                flag = Utility::IsInsidePolygon(polygon, first_control_point);
                // DLOG(INFO) << " is this point inside map? " << flag;
            }
            direction.x() = std::cos(goal_angle);
            direction.y() = std::sin(goal_angle);
            //t is another random number;
            t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
            second_control_point = goal_point_ - direction * t;
            // DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
            flag = Utility::IsInsidePolygon(polygon, second_control_point);
            while (flag < 1)
            {
                t = rand() % ((int)std::sqrt(map_width_ * map_width_ + map_height_ * map_height_));
                second_control_point = goal_point_ - direction * t;
                // DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
                flag = Utility::IsInsidePolygon(polygon, second_control_point);
                // DLOG(INFO) << " is this point inside map? " << flag;
            }
        }
        else
        { //use the way in paper
            direction.x() = std::cos(start_angle);
            direction.y() = std::sin(start_angle);
            float t = (Utility::ConvertVector3dToVector2d(goal_point_ - start_point_)).norm() / 3;
            first_control_point = start_point_ + direction * t;
            direction.x() = std::cos(goal_angle);
            direction.y() = std::sin(goal_angle);
            second_control_point = goal_point_ - direction * t;
        }
        control_points_vec_.emplace_back(first_control_point);
        control_points_vec_.emplace_back(second_control_point);

        geometrical_constraint_matrix_.block<3, 1>(0, 0) = start_point_;
        geometrical_constraint_matrix_.block<3, 1>(0, 1) = first_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 2) = second_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 3) = goal_point_;
    }

    // void CubicBezier::CalculateAnchorPoints()
    // {
    //     anchor_points_vec_.clear();
    //     anchor_points_vec_.emplace_back(start_point_);
    //     anchor_points_vec_.emplace_back(goal_point_);
    // }

    Eigen::Vector3d CubicBezier::GetValueAt(const float &t)
    {
        Eigen::Vector3d out;
        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateCoefficient(t);
        // CalculateCoefficient(t);

        // DLOG_IF(WARNING, coefficient_.size() != 4) << " coefficient size is not four!!! ";

        // out = coefficient_[0] * start_point_ + coefficient_[1] * control_points_vec_[0] + coefficient_[2] * control_points_vec_[1] + coefficient_[3] * goal_point_;
        // DLOG(INFO) << "cubic bezier value at : " << t << " is : " << out.x() << " " << out.y();
        return out;
    }

    float CubicBezier::GetAngleAt(const float &t)
    {
        Eigen::Vector3d derivative_value = GetFirstOrderDerivativeValueAt(t);
        float angle = std::atan2(derivative_value.y(), derivative_value.x());
        return angle;
    }
    void CubicBezier::CalculateLength()
    {
        for (uint i = 0; i < 100; ++i)
        {
            length_ += (GetValueAt((i + 1) / 100.0) - GetValueAt(i / 100.0)).norm();
        }
    }
    std::vector<Eigen::Vector3d> CubicBezier::ConvertCubicBezierToVector3d()
    {
        std::vector<Eigen::Vector3d> out;
        uint i = 0;
        uint size = GetLength() / 0.5;
        // DLOG(INFO) << "path size is " << size;
        for (i = 0; i < size + 1; ++i)
        {
            Eigen::Vector3d point3d;
            // DLOG(INFO) << " i/size = " << (float)i / size;
            point3d = GetValueAt((float)i / size);
            point3d.z() = GetAngleAt((float)i / size);
            // DLOG(INFO) << "point3d is " << point3d.x() << " " << point3d.y() << " " << point3d.z();
            out.emplace_back(point3d);
        }
        return out;
    }

    // std::vector<Eigen::Vector2d> CubicBezier::ConvertCubicBezierToVector2d()
    // {
    //     std::vector<Eigen::Vector2d> out;
    //     int i = 0;
    //     for (i = 0; i < 100; ++i)
    //     {
    //         Eigen::Vector2d point;
    //         // DLOG(INFO) << " i/100 = " << i / 100.0;
    //         point = GetValueAt(i / 100.0);
    //         out.emplace_back(point);
    //     }
    //     return out;
    // }
}