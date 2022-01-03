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
    Eigen::Vector4d CubicBezier::CalculateCoefficient(const double &t)
    {
        Eigen::Vector4d out;

        for (int i = 0; i < out.size(); ++i)
        {
            out[i] = std::pow(t, i);
            // DLOG(INFO) << i << "th element in coefficient matrix is " << out[i];
        }
        return out;
    }

    Eigen::Vector4d CubicBezier::CalculateFirstOrderDerivativeCoefficient(const double &t)
    {
        Eigen::Vector4d out;

        for (int i = 0; i < out.size(); ++i)
        {
            out[i] = i * (i - 1 < 0 ? 0 : std::pow(t, i - 1));
            // DLOG(INFO) << "t: " << t << " i-1 " << i - 1 << " std::pow(t, i - 1)" << (std::pow(t, i - 1));
            // DLOG(INFO) << i << "th element in first order derivation coefficient matrix is " << out[i];
        }
        return out;
    }
    Eigen::Vector4d CubicBezier::CalculateSecondOrderDerivativeCoefficient(const double &t)
    {
        Eigen::Vector4d out;

        for (int i = 0; i < out.size(); ++i)
        {
            out[i] = i * (i - 1) * (i - 2 < 0 ? 0 : std::pow(t, i - 2));
            // DLOG(INFO) << "t: " << t << " i-1 " << i - 1 << " std::pow(t, i - 1)" << (std::pow(t, i - 1));
            // DLOG(INFO) << i << "th element in first order derivation coefficient matrix is " << out[i];
        }
        return out;
    }
    Eigen::Vector3d CubicBezier::GetFirstOrderDerivativeValueAt(const double &t)
    {
        Eigen::Vector3d out;
        // DLOG(INFO) << "t " << t;
        Eigen::Vector4d coefficient = CalculateFirstOrderDerivativeCoefficient(t);
        out = geometrical_constraint_matrix_ * basis_matrix_ * coefficient;
        // DLOG_IF(INFO, out.x() == 0 && out.y() == 0) << "first order derivative coefficient is " << coefficient[0] << " " << coefficient[1] << " " << coefficient[2] << " " << coefficient[3];
        return out;
    }
    Eigen::Vector3d CubicBezier::GetSecondOrderDerivativeValueAt(const double &t)
    {
        Eigen::Vector3d out;
        // DLOG(INFO) << "t " << t;
        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateSecondOrderDerivativeCoefficient(t);

        return out;
    }

    void CubicBezier::CalculateControlPoints()
    {

        // control_points_vec_.clear();
        double start_angle = start_point_.z();
        double goal_angle = goal_point_.z();
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
            double t = (Utility::ConvertVector3dToVector2d(goal_point_ - start_point_)).norm() / 3;
            first_control_point = start_point_ + direction * t;
            direction.x() = std::cos(goal_angle);
            direction.y() = std::sin(goal_angle);
            second_control_point = goal_point_ - direction * t;
        }
        // control_points_vec_.emplace_back(first_control_point);
        // control_points_vec_.emplace_back(second_control_point);

        geometrical_constraint_matrix_.block<3, 1>(0, 0) = start_point_;
        geometrical_constraint_matrix_.block<3, 1>(0, 1) = first_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 2) = second_control_point;
        geometrical_constraint_matrix_.block<3, 1>(0, 3) = goal_point_;
    }

    void CubicBezier::CalculateAnchorPoints()
    {
        anchor_points_vec_.clear();
        anchor_points_vec_.emplace_back(start_point_);
        anchor_points_vec_.emplace_back(goal_point_);
        // for (const auto &point : anchor_points_vec_)
        // {
        //     DLOG(INFO) << "anchor points is " << point.x() << " " << point.y();
        // }
    }

    Eigen::Vector3d CubicBezier::GetValueAt(const double &t)
    {
        Eigen::Vector3d out;
        out = geometrical_constraint_matrix_ * basis_matrix_ * CalculateCoefficient(t);

        return out;
    }

    double CubicBezier::GetAngleAt(const double &t)
    {
        // DLOG(INFO) << "t " << t;
        Eigen::Vector3d derivative_value = GetFirstOrderDerivativeValueAt(t);
        double angle = std::atan2(derivative_value.y(), derivative_value.x());
        return angle;
    }
    void CubicBezier::CalculateLength()
    {
        // DLOG(INFO) << "In calculateLength()";
        for (uint i = 0; i < 100; ++i)
        {
            length_ += Utility::ConvertVector3dToVector2d(GetValueAt((i + 1) / 100.0) - GetValueAt(i / 100.0)).norm();
        }
        // DLOG(INFO) << "length is " << length_;
    }
    std::vector<Eigen::Vector3d> CubicBezier::ConvertCubicBezierToVector3d(const int &number_of_points)
    {
        std::vector<Eigen::Vector3d> out;
        uint i = 0;
        CalculateLength();
        // uint size = GetLength() / 2;
        uint size = number_of_points;
        DLOG_IF(WARNING, size == 0) << "path size is zero!!!";
        for (i = 0; i < size + 1; ++i)
        {
            Eigen::Vector3d point3d;
            // DLOG(INFO) << " i/size = " << (double)i / size;
            point3d = GetValueAt((double)i / size);
            point3d.z() = GetAngleAt((double)i / size);
            // DLOG(INFO) << "point3d is " << point3d.x() << " " << point3d.y() << " " << point3d.z();
            out.emplace_back(point3d);
        }
        return out;
    }

    double CubicBezier::GetTotalCurvature()
    {
        double total_curvature = 0;
        // DLOG(INFO) << "In calculateLength()";
        for (uint i = 0; i < 100; ++i)
        {
            total_curvature += GetCurvatureAt(i / 100.0);
        }
        // DLOG(INFO) << "total curvature is " << total_curvature;
        return total_curvature;
    }

    double CubicBezier::GetCurvatureAt(const double &t)
    {
        double curvature = 0;
        Eigen::Vector2d first_order_derivative = Utility::ConvertVector3dToVector2d(GetFirstOrderDerivativeValueAt(t));
        Eigen::Vector2d second_order_derivative = Utility::ConvertVector3dToVector2d(GetSecondOrderDerivativeValueAt(t));
        if (first_order_derivative.norm() != 0)
        {
            curvature = std::abs(Utility::CrossProduct(first_order_derivative, second_order_derivative)) / std::pow(first_order_derivative.norm(), 3);
        }
        else
        {
            curvature = 100000;
        }

        DLOG_IF(INFO, std::isnan(curvature)) << "curvature at " << t << " is " << curvature << " its first derivative norm is " << first_order_derivative.norm() << " first derivative is " << first_order_derivative.x() << " " << first_order_derivative.y();
        return curvature;
    }
}