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
#include "utility.h"
namespace CubicBezier
{
    class CubicBezier
    {
    public:
        CubicBezier(){};
        CubicBezier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, int width, int height)
        {
            start_point_ = start;
            goal_point_ = goal;
            map_width_ = width;
            map_height_ = height;
            CalculateControlPoints();
        };
        CubicBezier(const std::vector<Eigen::Vector3d> &points_vec)
        {
            if (points_vec.size() != 4)
            {
                DLOG(WARNING) << "points_vec size is not correct!!!";
            }
            else
            {
                start_point_ = points_vec[0];
                control_points_vec_.clear();
                control_points_vec_.emplace_back(points_vec[1]);
                control_points_vec_.emplace_back(points_vec[2]);
                goal_point_ = points_vec[3];
            }
        }

        void SetControlPoints(const std::vector<Eigen::Vector3d> &control_points) { control_points_vec_ = control_points; };

        Eigen::Vector3d GetValueAt(const float &t);

        float GetAngleAt(const float &t);

        std::vector<Eigen::Vector3d> GetControlPoints() { return control_points_vec_; };
        std::vector<Eigen::Vector3d> GetAnchorPoints() { return anchor_points_vec_; };

        float GetLength() { return length_; };

        std::vector<Eigen::Vector3d> ConvertCubicBezierToVector3d();

        // std::vector<Eigen::Vector2d> ConvertCubicBezierToVector2d();

    private:
        /**
         * @brief split curve into 100, sum all euler distance.
         * 
         */
        void CalculateLength();

        void CalculateCoefficient(const float &t);

        void CalculateControlPoints();

        void CalculateAnchorPoints();

        void CalculateFirstOrderDerivativeCoefficient(const float &t);

        void CalculateSecondOrderDerivativeCoefficient(const float &t);

        Eigen::Vector3d GetFirstOrderDerivativeValueAt(const float &t);

    private:
        /**
         * @brief for a cubic bezier, control points are two
         * 
         */
        std::vector<Eigen::Vector3d> control_points_vec_;
        /**
         * @brief actually is start and goal points
         * 
         */
        std::vector<Eigen::Vector3d> anchor_points_vec_;

        Eigen::Vector3d start_point_;
        Eigen::Vector3d goal_point_;
        /**
         * @brief length is four including start and goal points 
         * 
         */
        std::vector<float> coefficient_;

        std::vector<float> first_order_derivative_coefficient_;

        std::vector<float> second_order_derivative_coefficient_;

        float length_ = 0;

        float map_width_;
        float map_height_;
        /**
         * @brief determine how to calculate control points, random or use the way in paper
         * 
         */
        bool use_random_ = false;
    };
}