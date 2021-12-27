/**
 * @file utility.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief include some type conversion function
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
}

#endif // UTILITY
