/**
 * @file piecewise_cubic_bezier.h
 * @author Jialiang Han
 * @brief this class generate a piecewise cubic bezier curve
 * @version 0.1
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>

namespace GeneticAlgorithm
{
  class PiecewiseCubicBezier
  {
  public:
    /**
   * @brief Construct a new Piecewise Cubic Bezier object
   * 
   */
    PiecewiseCubicBezier(){};
    /**
     * @brief Construct a new Piecewise Cubic Bezier object
     * 
     * @param control_points 
     */
    PiecewiseCubicBezier(const std::vector<Eigen::Vector2d> &control_points)
    {
      control_points_vec_ = control_points;
    };

    /**
     * @brief get specific point on this bezier curve
     * 
     * @param u 
     * @return Eigen::Vector2d 
     */
    Eigen::Vector2d ValueAt(const float &u);
    /**
 * @brief Get the Length of bezier curve
 * 
 * @return float 
 */
    float GetLength() { return length_; };

  private:
    //list of control point, include start and goal points;
    std::vector<Eigen::Vector2d> control_points_vec_;
    /**
     * @brief curve length
     * 
     */
    float length_;
  };
}
