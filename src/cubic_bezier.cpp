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
        first_order_derivative_coefficient_.clear();
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

    void CubicBezier::CalculateSecondOrderDerivativeCoefficient(const float &t)
    {
        second_order_derivative_coefficient_.clear();
        float coefficient;
        coefficient = 6 * std::pow((1 - t), 1);
        second_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = -6 * std::pow(1 - t, 1) - 6 * (1 - t) + 6 * t;
        second_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = 6 * (1 - t) - 6 * t - 6 * std::pow(t, 1);
        second_order_derivative_coefficient_.emplace_back(coefficient);
        coefficient = 6 * std::pow(t, 1);
        second_order_derivative_coefficient_.emplace_back(coefficient);
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

    void CubicBezier::CalculateControlPoints(int width, int height)
    {
        control_points_vec_.clear();
        float start_angle = start_point_.z();
        float goal_angle = goal_point_.z();
        Eigen::Vector2d first_control_point;
        Eigen::Vector2d second_control_point;
        Eigen::Vector2d direction;
        direction.x() = std::cos(start_angle);
        direction.y() = std::sin(start_angle);
        std::vector<Eigen::Vector2d> polygon;
        polygon.emplace_back(Eigen::Vector2d(0, 0));
        polygon.emplace_back(Eigen::Vector2d(width, 0));
        polygon.emplace_back(Eigen::Vector2d(width, height));
        polygon.emplace_back(Eigen::Vector2d(0, height));
        polygon.emplace_back(Eigen::Vector2d(0, 0));
        DLOG(INFO) << " map width is " << width << " height is " << height;
        //t is some random number;
        int t = rand() % ((int)std::sqrt(width * width + height * height));
        first_control_point = Utility::ConvertVector3dToVector2d(start_point_) + direction * t;
        LOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
        int flag = Utility::IsInsidePolygon(polygon, first_control_point);
        while (flag < 1)
        {
            t = rand() % ((int)std::sqrt(width * width + height * height));
            first_control_point = Utility::ConvertVector3dToVector2d(start_point_) + direction * t;
            DLOG(INFO) << "first control point is " << first_control_point.x() << " " << first_control_point.y();
            flag = Utility::IsInsidePolygon(polygon, first_control_point);
            DLOG(INFO) << " is this point inside map? " << flag;
        }
        direction.x() = std::cos(goal_angle);
        direction.y() = std::sin(goal_angle);
        //t is another random number;
        t = rand() % ((int)std::sqrt(width * width + height * height));
        second_control_point = Utility::ConvertVector3dToVector2d(goal_point_) - direction * t;
        DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
        flag = Utility::IsInsidePolygon(polygon, second_control_point);
        while (flag < 1)
        {
            t = rand() % ((int)std::sqrt(width * width + height * height));
            second_control_point = Utility::ConvertVector3dToVector2d(goal_point_) - direction * t;
            DLOG(INFO) << "second control point is " << second_control_point.x() << " " << second_control_point.y();
            flag = Utility::IsInsidePolygon(polygon, second_control_point);
            DLOG(INFO) << " is this point inside map? " << flag;
        }
        control_points_vec_.emplace_back(first_control_point);
        control_points_vec_.emplace_back(second_control_point);
    }

    std::vector<Eigen::Vector2d> CubicBezier::GetControlPointsAndAnchorPoints()
    {
        std::vector<Eigen::Vector2d> out;
        out.emplace_back(Utility::ConvertVector3dToVector2d(start_point_));
        for (auto point : control_points_vec_)
        {
            out.emplace_back(point);
        }
        out.emplace_back(Utility::ConvertVector3dToVector2d(goal_point_));
        return out;
    }

    Eigen::Vector2d CubicBezier::GetValueAt(const float &t)
    {
        Eigen::Vector2d out;

        CalculateCoefficient(t);

        DLOG_IF(WARNING, coefficient_.size() != 4) << " coefficient size is not four!!! ";

        Eigen::Vector2d vector2d_start = Utility::ConvertVector3dToVector2d(start_point_);
        Eigen::Vector2d vector2d_goal = Utility::ConvertVector3dToVector2d(goal_point_);

        out = coefficient_[0] * vector2d_start + coefficient_[1] * control_points_vec_[0] + coefficient_[2] * control_points_vec_[1] + coefficient_[3] * vector2d_goal;
        // DLOG(INFO) << "cubic bezier value at : " << t << " is : " << out.x() << " " << out.y();
        return out;
    }

    float CubicBezier::GetAngleAt(const float &t)
    {
        Eigen::Vector2d derivative_value = GetFirstOrderDerivativeValueAt(t);
        float angle = std::atan2(derivative_value.y(), derivative_value.x()) + M_PI;
        return angle;
    }
}