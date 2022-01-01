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
        CubicBezier()
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
        };
        CubicBezier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, int width, int height)
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
            start_point_ = start;
            goal_point_ = goal;
            map_width_ = width;
            map_height_ = height;
            CalculateControlPoints();
            CalculateLength();
        };
        CubicBezier(const std::vector<Eigen::Vector3d> &points_vec)
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
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
            CalculateLength();
        }

        void SetControlPoints(const std::vector<Eigen::Vector3d> &control_points) { control_points_vec_ = control_points; };

        Eigen::Vector3d GetValueAt(const float &t);

        float GetAngleAt(const float &t);

        std::vector<Eigen::Vector3d> GetControlPoints() const { return control_points_vec_; };
        std::vector<Eigen::Vector3d> GetAnchorPoints() const { return anchor_points_vec_; };

        float GetLength() const { return length_; };

        std::vector<Eigen::Vector3d> ConvertCubicBezierToVector3d();

        // std::vector<Eigen::Vector2d> ConvertCubicBezierToVector2d();

    private:
        /**
         * @brief split curve into 100, sum all euler distance.
         * 
         */
        void CalculateLength();

        // void CalculateCoefficient(const float &t);

        Eigen::Vector4d CalculateCoefficient(const float &t);

        Eigen::Vector4d CalculateFirstOrderDerivativeCoefficient(const float &t);

        void CalculateControlPoints();

        // void CalculateAnchorPoints();

        // void CalculateFirstOrderDerivativeCoefficient(const float &t);

        // void CalculateSecondOrderDerivativeCoefficient(const float &t);

        Eigen::Vector3d GetFirstOrderDerivativeValueAt(const float &t);

    private:
        /**
         * @brief this is matrix form of anchor and control points
         * row is 3 due to eigen vector3d, column is 4 due to this is a cubic bezier
         */
        Eigen::Matrix<double, 3, 4> geometrical_constraint_matrix_;

        Eigen::Matrix<double, 4, 4> basis_matrix_;

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
        // std::vector<float> coefficient_;

        // std::vector<float> first_order_derivative_coefficient_;

        // std::vector<float> second_order_derivative_coefficient_;

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