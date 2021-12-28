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
#include "cubic_bezier.h"
#include "utility.h"
#include <cmath>
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
    PiecewiseCubicBezier(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, int width, int height)
    {
      start_point_ = start;
      goal_point_ = goal;
      map_width_ = width;
      map_height_ = height;
      CalculateControlPoints();
      CalculateCubicBezier();
    };

    /**
     * @brief get specific point on this bezier curve
     * 
     * @param u 
     * @return Eigen::Vector2d 
     */
    Eigen::Vector2d GetValueAt(const float &u);

    float GetAngleAt(const float &u);

    void SetAnchorPoints(const std::vector<Eigen::Vector2d> &anchor_points_vec)
    {
      anchor_points_vec_ = anchor_points_vec;
      CalculateControlPoints();
      CalculateCubicBezier();
    }

    /**
     * @brief Get the Length of bezier curve
     * 
     * @return float 
     */
    float GetLength() { return length_; };

    std::vector<Eigen::Vector3d> ConvertPiecewiseCubicBezierToVector3d();

  private:
    void CalculateLength();
    /**
     * @brief calculate control points according to anchor points, put all points into points_vec_
     * 
     */
    void CalculateControlPoints();
    /**
     * @brief calculate cubic bezier object using points_vec_
     * 
     */
    void CalculateCubicBezier();

  private:
    // int number_of_cubic_bezier_;
    std::vector<CubicBezier::CubicBezier> cubic_bezier_vec_;
    // points list contains both free,fixed anchor points and control points
    std::vector<Eigen::Vector2d> points_vec_;

    // list free anchor points(P), anchor points are the points which bezier curve pass through
    std::vector<Eigen::Vector2d> anchor_points_vec_;

    Eigen::Vector3d start_point_;
    Eigen::Vector3d goal_point_;
    /**
     * @brief curve length
     * 
     */
    float length_ = 0;

    float map_width_;
    float map_height_;
  };
}
