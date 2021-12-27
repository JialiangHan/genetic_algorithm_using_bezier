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
namespace CubicBezier
{
    void CubicBezier::CalculateCoefficient(const float &t)
    {
        coefficient_.clear();
        float coefficient;
        coefficient = std::pow((1 - t), 3);
        coefficient_.emplace_back(coefficient);
        coefficient = 3 * t * std::pow(1 - t, 2);
        coefficient_.emplace_back(coefficient);
        coefficient = 3 * std::pow(t, 2) * (1 - t);
        coefficient_.emplace_back(coefficient);
        coefficient = std::pow(t, 3);
        coefficient_.emplace_back(coefficient);
    }

    void CubicBezier::CalculateFirstOrderDerivativeCoefficient(const float &t)
    {
        float coefficient;
        coefficient = -3 * std::pow((1 - t), 2);
        first_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = 3 * std::pow(1 - t, 2) - 6 * t * (1 - t);
        first_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = 6 * t * (1 - t) - 3 * std::pow(t, 2);
        first_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = 3 * std::pow(t, 2);
        first_order_derivative_coefficient_.emplace_back(coefficient);
    }

    Eigen::Vector2d CubicBezier::GetFirstOrderDerivativeValueAt(const float &t)
    {
        Eigen::Vector2d out;

        CalculateFirstOrderDerivativeCoefficient(t);

        Eigen::Vector2d vector2d_start = Utility::ConvertVector3dToVector2d(start_point_);
        Eigen::Vector2d vector2d_goal = Utility::ConvertVector3dToVector2d(goal_point_);
        out = first_order_derivative_coefficient_[0] * vector2d_start + first_order_derivative_coefficient_[1] * control_points_vec_[0] + first_order_derivative_coefficient_[2] * control_points_vec_[1] + first_order_derivative_coefficient_[3] * vector2d_goal;
        return out;
    }

    void CubicBezier::CalculateControlPoints()
    {
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
    }

    Eigen::Vector2d CubicBezier::GetValueAt(const float &t)
    {
        Eigen::Vector2d out;

        CalculateCoefficient(t);
        for (auto weight : coefficient_)
        {
            DLOG(INFO) << " coefficient is " << weight;
        }
        Eigen::Vector2d vector2d_start = Utility::ConvertVector3dToVector2d(start_point_);
        Eigen::Vector2d vector2d_goal = Utility::ConvertVector3dToVector2d(goal_point_);

        out = coefficient_[0] * vector2d_start + coefficient_[1] * control_points_vec_[0] + coefficient_[2] * control_points_vec_[1] + coefficient_[3] * vector2d_goal;
        DLOG(INFO) << "cubic bezier value at : " << t << " is : " << out.x() << " " << out.y();
        return out;
    }

    float CubicBezier::GetAngleAt(const float &t)
    {
        Eigen::Vector2d derivative_value = GetFirstOrderDerivativeValueAt(t);
        float angle = std::atan2(derivative_value.y(), derivative_value.x()) + M_PI;
        return angle;
    }
}