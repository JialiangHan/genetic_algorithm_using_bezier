/**
 * @file utility.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief include some type conversion function and some computational geometry
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef UTILITY
#define UTILITY
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <nav_msgs/Path.h>
#include <cubic_bezier.h>
#include <Eigen/Dense>

namespace Utility
{
    std::vector<Eigen::Vector3d> ConvertCubicBezierToVector3d(CubicBezier::CubicBezier &cubic_bezier);

    void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3d> &vector_3d_vec);

    Eigen::Vector2d ConvertIndexToEigenVector2d(const int &index, const int &map_width);

    float GetDistanceFromVector2dToVector3d(const Eigen::Vector3d &vector_3d, const Eigen::Vector2d &vector_2d);
    /**
     * @brief convert angle in deg into [0,360)
     * 
     * @param deg 
     * @return float 
     */
    float DegNormalization(const float &deg);

    float RadNormalization(const float &rad);

    Eigen::Vector2d ConvertVector3dToVector2d(const Eigen::Vector3d &vector_3d);

    Eigen::Vector3d ConvertVector2dToVector3d(const Eigen::Vector2d &vector_2d);

    float ConvertRadToDeg(const float &rad);
    /**
     * @brief check if p2 lines on p1-p2 
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @return true 
     * @return false 
     */
    bool OnSegment(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);
    /**
     * @brief determine if two segment(p1-p2, p3-p4) are intersected? 
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @param p4 
     * @return int 
     */
    int IsIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3, Eigen::Vector2d p4);
    /**
     * @brief this is only work for vector2d, for 3d please cross in eigen
     * 
     * @param p1 
     * @param p2 
     * @return float 
     */
    float CrossProduct(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);
}

#endif // UTILITY
