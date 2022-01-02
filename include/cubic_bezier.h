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
            // CalculateControlPoints();
            // CalculateLength();
            CalculateAnchorPoints();
        };
        CubicBezier(const Eigen::Matrix<double, 3, 4> &point_matrix)
        {
            basis_matrix_ << 1, -3, 3, -1,
                0, 3, -6, 3,
                0, 0, 3, -3,
                0, 0, 0, 1;
            if (point_matrix.rows() != 3 || point_matrix.cols() != 4)
            {
                // DLOG(WARNING) << "points_vec size is not correct!!!";
            }
            else
            {
                geometrical_constraint_matrix_ = point_matrix;
             }
            // CalculateLength();
            CalculateAnchorPoints();
        }

        Eigen::Vector3d GetValueAt(const double &t);

        double GetAngleAt(const double &t);

        // std::vector<Eigen::Vector3d> GetControlPoints() const { return control_points_vec_; };
        std::vector<Eigen::Vector3d> GetAnchorPoints() const { return anchor_points_vec_; };

        double GetLength()
        {
            CalculateLength();
            return length_;
        };

        double GetCurvatureAt(const double &t);

        double GetTotalCurvature();

        std::vector<Eigen::Vector3d> ConvertCubicBezierToVector3d(const int &number_of_points);

    private:
        /**
         * @brief split curve into 100, sum all euler distance.
         * 
         */
        void CalculateLength();

        Eigen::Vector4d CalculateCoefficient(const double &t);

        Eigen::Vector4d CalculateFirstOrderDerivativeCoefficient(const double &t);

        Eigen::Vector4d CalculateSecondOrderDerivativeCoefficient(const double &t);

        void CalculateControlPoints();

        void CalculateAnchorPoints();

        Eigen::Vector3d GetFirstOrderDerivativeValueAt(const double &t);

        Eigen::Vector3d GetSecondOrderDerivativeValueAt(const double &t);

    private:
        /**
         * @brief this is matrix form of anchor and control points
         * row is 3 due to eigen vector3d, column is 4 due to this is a cubic bezier
         */
        Eigen::Matrix<double, 3, 4> geometrical_constraint_matrix_;

        Eigen::Matrix<double, 4, 4> basis_matrix_;

        /**
         * @brief actually is start and goal points
         * 
         */
        std::vector<Eigen::Vector3d> anchor_points_vec_;

        Eigen::Vector3d start_point_;
        Eigen::Vector3d goal_point_;

        double length_ = 0;

        double map_width_;
        double map_height_;
        /**
         * @brief determine how to calculate control points, random or use the way in paper
         * 
         */
        bool use_random_ = false;
    };
}