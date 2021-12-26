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
#include <cassert>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>

namespace CubicBezier
{
    class CubicBezier
    {
    public:
        CubicBezier(){};

    private:
        std::vector<Eigen::Vector2d> control_points_vec_;
        Eigen::Vector3d start_point_;
        Eigen::Vector3d goal_point_;
    }
}