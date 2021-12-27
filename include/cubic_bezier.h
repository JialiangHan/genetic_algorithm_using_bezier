/**
 * @file bezier.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this is a basic bezier class
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "glog/logging.h"
#include "gflags/gflags.h"
namespace CubicBezier
{
    class CubicBezier
    {
    public:
        CubicBezier(){};
        CubicBezier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
        {
            start_point_ = start;
            goal_point_ = goal;
            // CalculateControlPoints();
        };

        void SetControlPoints(const std::vector<Eigen::Vector2d> &control_points) { control_points_vec_ = control_points; };

        Eigen::Vector2d GetValueAt(const float &t);

        float GetAngleAt(const float &t);

    private:
        void CalculateCoefficient(const float &t);

        void CalculateControlPoints();

        void CalculateFirstOrderDerivativeCoefficient(const float &t);

        Eigen::Vector2d GetFirstOrderDerivativeValueAt(const float &t);

    private:
        /**
         * @brief for a cubic bezier, control points are two
         * 
         */
        std::vector<Eigen::Vector2d> control_points_vec_;

        Eigen::Vector3d start_point_;
        Eigen::Vector3d goal_point_;
        /**
         * @brief length is four including start and goal points 
         * 
         */
        std::vector<float> coefficient_;

        std::vector<float> first_order_derivative_coefficient_;
    };
}